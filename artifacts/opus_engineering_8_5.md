# Anti-Cogging Calibration & Feedforward Module for SimpleFOC (ESP32-S3)

I don't have context for what "Cycle 8" refers to, such as a project iteration or spec revision. If it implies specific requirements, tell me and I'll adapt the code. This is a complete, self-contained implementation based on these assumptions:

- **SimpleFOC ≥ 2.3.x**. It uses `motor.current_sp`, `motor.loopFOC()`, `motor.move()`, `sensor->getMechanicalAngle()`, `motor.PID_velocity` and `motor.P_angle`.
- **512-point table** indexed by the sensor's mechanical angle. The table is built from a bidirectional quasi-static sweep, then smoothed with a truncated Fourier (sinusoidal harmonic) fit.
- **Injection** happens in torque space. The module runs the velocity and angle loops externally, using the motor's own PID objects, and calls `motor.move()` in torque mode with `PID_output + cogging_ff`. Stock SimpleFOC has no hook for adding torque inside its velocity loop, so this approach avoids modifying the library.
- **Persistence** uses ESP32 NVS (`Preferences`) with a CRC and a sensor-signature check.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
    static constexpr uint16_t TABLE_SIZE = 512;
    static constexpr uint32_t STORAGE_VERSION = 0x41430108; // 'AC' v1.8

    struct Config {
        uint16_t settle_ms        = 30;    // wait after each step before sampling
        uint16_t samples_per_bin  = 40;    // averaged current_sp samples per step
        uint16_t sample_period_us = 250;   // spacing between samples
        uint8_t  harmonics        = 48;    // keep harmonics 1..N (0 = no smoothing)
        float    max_comp         = 2.0f;  // clamp |ff| (A or V, same units as current_sp)
        float    cal_P_angle      = 40.0f; // stiff position loop during calibration
        float    cal_vel_limit    = 2.0f;  // rad/s, during calibration
        bool     bidirectional    = true;  // forward+reverse pass cancels friction
    };

    AntiCogging(BLDCMotor& motor, Sensor& sensor);

    // --- Calibration ---------------------------------------------------
    // Blocking. Motor must be initialised (initFOC done) and able to hold position.
    // Returns false on abort/failure. Optional progress callback (0..100).
    bool calibrate(const Config& cfg, void (*progress)(uint8_t pct) = nullptr);

    // --- Runtime --------------------------------------------------------
    void  enable(bool en) { enabled_ = en && valid_; }
    bool  isEnabled() const { return enabled_; }
    bool  isValid()   const { return valid_; }
    void  setGain(float g) { gain_ = g; }   // 0..1+ blend factor

    // Feedforward value for a given mechanical angle [0, 2pi).
    float IRAM_ATTR compensation(float mech_angle) const;

    // Compensation for the current sensor reading.
    float IRAM_ATTR compensationNow() const;

    // Replacement for motor.move() that injects feedforward.
    // mode: torque / velocity / angle. target in that mode's units.
    // Call at your motion-loop rate after motor.loopFOC().
    void move(MotionControlType mode, float target);

    // --- Persistence (ESP32 NVS) --------------------------------------
    bool save(const char* ns = "anticog");
    bool load(const char* ns = "anticog");
    void clear();

    // --- Diagnostics ------------------------------------------------
    const float* table() const { return table_; }
    void dump(Stream& s) const;

private:
    BLDCMotor& motor_;
    Sensor&    sensor_;

    float   table_[TABLE_SIZE];
    bool    valid_   = false;
    bool    enabled_ = false;
    float   gain_    = 1.0f;

    // Signature: detect a remounted sensor or a changed pole-pair setup.
    float   sig_zero_offset_ = 0;
    int     sig_direction_   = 0;
    int     sig_pole_pairs_  = 0;

    uint32_t last_move_us_ = 0;

    void serviceFor(uint32_t ms);
    void fillGaps(float* sum, uint16_t* cnt);
    void harmonicSmooth(uint8_t n_harm);
    void removeMean();
    static uint32_t crc32(const uint8_t* d, size_t n);
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& motor, Sensor& sensor)
    : motor_(motor), sensor_(sensor) {
    clear();
}

void AntiCogging::clear() {
    memset(table_, 0, sizeof(table_));
    valid_ = enabled_ = false;
}

// ---------------------------------------------------------------------
// Keep FOC + position loop alive while waiting.
// ---------------------------------------------------------------------
void AntiCogging::serviceFor(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        motor_.loopFOC();
        motor_.move();
    }
}

// ---------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------
bool AntiCogging::calibrate(const Config& cfg, void (*progress)(uint8_t)) {
    if (motor_.motor_status != FOCMotorStatus::motor_ready) {
        SIMPLEFOC_DEBUG("ACOG: motor not ready");
        return false;
    }

    // Save and override controller settings.
    MotionControlType prev_ctrl = motor_.controller;
    float prev_P      = motor_.P_angle.P;
    float prev_vlim   = motor_.velocity_limit;
    bool  prev_en     = enabled_;
    enabled_ = false;

    motor_.controller     = MotionControlType::angle;
    motor_.P_angle.P      = cfg.cal_P_angle;
    motor_.velocity_limit = cfg.cal_vel_limit;

    // Heap buffers (4 KB total) instead of stack.
    float*    sum = (float*)   calloc(TABLE_SIZE, sizeof(float));
    uint16_t* cnt = (uint16_t*)calloc(TABLE_SIZE, sizeof(uint16_t));
    if (!sum || !cnt) { free(sum); free(cnt); return false; }

    const float step   = _2PI / TABLE_SIZE;
    const float start  = motor_.shaft_angle;
    const int   passes = cfg.bidirectional ? 2 : 1;
    const uint32_t total_steps = (uint32_t)TABLE_SIZE * passes;
    uint32_t done = 0;

    motor_.target = start;
    serviceFor(300);

    for (int pass = 0; pass < passes; ++pass) {
        const float dir = (pass == 0) ? 1.0f : -1.0f;
        // Pre-roll a few steps to take up backlash/hysteresis before sampling.
        for (int k = 1; k <= 8; ++k) {
            motor_.target += dir * step;
            serviceFor(cfg.settle_ms);
        }
        for (uint16_t i = 0; i < TABLE_SIZE; ++i) {
            motor_.target += dir * step;
            serviceFor(cfg.settle_ms);

            float acc = 0, ang_s = 0, ang_c = 0;
            for (uint16_t s = 0; s < cfg.samples_per_bin; ++s) {
                uint32_t ts = micros();
                motor_.loopFOC();
                motor_.move();
                acc += motor_.current_sp;               // torque command holding position
                float a = sensor_.getMechanicalAngle();
                ang_s += sinf(a); ang_c += cosf(a);     // circular mean (wrap-safe)
                while (micros() - ts < cfg.sample_period_us) motor_.loopFOC();
            }
            float a_mean = atan2f(ang_s, ang_c);
            if (a_mean < 0) a_mean += _2PI;
            int bin = (int)lroundf(a_mean / step) % TABLE_SIZE;
            sum[bin] += acc / cfg.samples_per_bin;
            cnt[bin]++;

            if (progress && (++done % 16 == 0)) progress((uint8_t)(100 * done / total_steps));

            // Abort on runaway.
            if (fabsf(motor_.shaft_velocity) > 4.0f * cfg.cal_vel_limit) {
                SIMPLEFOC_DEBUG("ACOG: abort, overspeed");
                free(sum); free(cnt);
                motor_.controller = prev_ctrl; motor_.P_angle.P = prev_P;
                motor_.velocity_limit = prev_vlim; enabled_ = prev_en;
                return false;
            }
        }
        // Overshoot pre-roll back for the reverse pass.
        if (cfg.bidirectional && pass == 0) {
            motor_.target += 8 * step;
            serviceFor(200);
        }
    }

    fillGaps(sum, cnt);
    free(sum); free(cnt);

    removeMean();                         // DC = friction/load, not cogging
    if (cfg.harmonics) harmonicSmooth(cfg.harmonics);

    for (uint16_t i = 0; i < TABLE_SIZE; ++i)
        table_[i] = _constrain(table_[i], -cfg.max_comp, cfg.max_comp);

    // Record signature.
    sig_zero_offset_ = motor_.zero_electric_angle;
    sig_direction_   = (int)motor_.sensor_direction;
    sig_pole_pairs_  = motor_.pole_pairs;

    // Restore state.
    motor_.controller     = prev_ctrl;
    motor_.P_angle.P      = prev_P;
    motor_.velocity_limit = prev_vlim;
    motor_.target         = (prev_ctrl == MotionControlType::angle) ? motor_.shaft_angle : 0;

    valid_   = true;
    enabled_ = true;
    if (progress) progress(100);
    SIMPLEFOC_DEBUG("ACOG: calibration complete");
    return true;
}

// Average the bins and linearly interpolate any bins that received no sample.
void AntiCogging::fillGaps(float* sum, uint16_t* cnt) {
    int first = -1;
    for (int i = 0; i < TABLE_SIZE; ++i) {
        if (cnt[i]) { table_[i] = sum[i] / cnt[i]; if (first < 0) first = i; }
    }
    if (first < 0) { memset(table_, 0, sizeof(table_)); return; }

    int prev = first;
    for (int k = 1; k <= TABLE_SIZE; ++k) {
        int i = (first + k) % TABLE_SIZE;
        if (!cnt[i] && i != first) continue;
        int gap = (i - prev + TABLE_SIZE) % TABLE_SIZE;
        if (gap == 0) gap = TABLE_SIZE;
        for (int g = 1; g < gap; ++g) {
            float t = (float)g / gap;
            table_[(prev + g) % TABLE_SIZE] = table_[prev] * (1 - t) + table_[i] * t;
        }
        prev = i;
    }
}

void AntiCogging::removeMean() {
    float m = 0;
    for (auto v : table_) m += v;
    m /= TABLE_SIZE;
    for (auto& v : table_) v -= m;
}

// Truncated Fourier series: keep harmonics 1..N (sinusoidal reconstruction).
// Cogging is periodic in LCM(slots, poles), so its energy sits in a few
// harmonics. This rejects sample noise.
void AntiCogging::harmonicSmooth(uint8_t n_harm) {
    if (n_harm >= TABLE_SIZE / 2) return;
    float* a = (float*)calloc(n_harm + 1, sizeof(float));
    float* b = (float*)calloc(n_harm + 1, sizeof(float));
    if (!a || !b) { free(a); free(b); return; }

    const float w = _2PI / TABLE_SIZE;
    for (int k = 1; k <= n_harm; ++k) {
        float sa = 0, sb = 0;
        for (int i = 0; i < TABLE_SIZE; ++i) {
            float ph = w * k *