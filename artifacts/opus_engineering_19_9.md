# SimpleFOC Anti-Cogging Module (512-point, ESP32-S3)

I don't have context for "Cycle 19" or any earlier cycles, so this is a self-contained implementation based on these assumptions:

- **SimpleFOC ≥ 2.3** (`BLDCMotor`, `motor.current_sp`, `motor.voltage.q`, `motor.shaft_angle`, `motor.sensor`).
- **Absolute mechanical sensor** (MT6701, AS5047, AS5600, or a magnetic encoder with index). Cogging is periodic in *mechanical* angle, so the table is indexed by `sensor->getMechanicalAngle()`.
- **Arduino-ESP32 core**, with NVS persistence via `Preferences`.
- "Sinusoidal" means the raw 512-sample map is decomposed by DFT. Only the dominant harmonics are kept, and the 512-point LUT is rebuilt from that sum of sinusoids. This removes noise and friction artifacts.

If your earlier cycles defined different interfaces, such as a custom motor wrapper or a different storage layer, share them and I'll adapt the code.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

class AntiCogging {
public:
    static constexpr uint16_t TABLE_SIZE   = 512;
    static constexpr uint16_t MAX_HARMONIC = 128;     // keep below TABLE_SIZE/2
    static constexpr uint32_t STORE_MAGIC  = 0xAC6C0019; // "cycle 19" format tag
    static constexpr uint16_t STORE_VER    = 1;

    struct Config {
        // --- calibration ---
        float    settle_ms        = 60.0f;   // wait after each step before sampling
        uint16_t samples_per_bin  = 40;      // samples averaged per bin
        uint16_t sample_interval_us = 250;   // spacing between samples
        float    cal_P = 40.0f, cal_I = 800.0f, cal_D = 0.0f; // stiff hold controller
        float    cal_torque_limit = 1.0f;    // A (current mode) or V (voltage mode)
        bool     bidirectional    = true;    // fwd+rev sweep cancels Coulomb friction
        // --- harmonic fitting ---
        uint16_t max_harmonic     = 64;      // highest harmonic kept
        float    min_harmonic_amp = 0.002f;  // drop components below this amplitude
        // --- runtime injection ---
        float    gain             = 1.0f;    // global FF scale (0..~1.2)
        float    fade_start_rad_s = 30.0f;   // full FF below this speed
        float    fade_end_rad_s   = 120.0f;  // zero FF above this speed
        float    latency_s        = 120e-6f; // loop latency used for angle lookahead
    };

    struct Harmonic { uint16_t k; float a; float b; }; // a*cos(k*th) + b*sin(k*th)

    explicit AntiCogging(BLDCMotor& motor);

    void   setConfig(const Config& c) { cfg_ = c; }
    Config& config()                  { return cfg_; }

    // Blocking calibration. Motor must already be initFOC()'d.
    // progress callback (optional): called with 0..100.
    bool calibrate(void (*progress)(uint8_t pct) = nullptr);

    // Call every loop: motor.loopFOC(); motor.move(target); ac.apply();
    void IRAM_ATTR apply();

    // Pure lookup (linear interpolation), angle in rad (any range).
    float IRAM_ATTR lookup(float mech_angle) const;

    bool save(const char* ns = "acog");
    bool load(const char* ns = "acog");
    void clear();

    void enable(bool en)       { enabled_ = en && valid_; }
    bool enabled() const       { return enabled_; }
    bool valid()   const       { return valid_; }
    float lastCompensation() const { return last_ff_; }

    const float* table() const { return lut_; }
    uint16_t harmonicCount() const { return n_harm_; }
    const Harmonic* harmonics() const { return harm_; }
    void printReport(Print& out) const;

private:
    void  runFocFor(float target, uint32_t us);
    float sampleHold(float target);
    bool  sweep(float* out, bool reverse, float base, void (*progress)(uint8_t), uint8_t p0, uint8_t p1);
    void  fitHarmonics(const float* raw);
    void  rebuildLut();
    static uint32_t crc32(const uint8_t* d, size_t n);
    float commandedTorque() const;

    BLDCMotor& m_;
    Config cfg_;
    float  lut_[TABLE_SIZE + 1];  // +1 wrap entry for branchless interpolation
    Harmonic harm_[MAX_HARMONIC];
    uint16_t n_harm_ = 0;
    bool  valid_   = false;
    volatile bool enabled_ = false;
    float last_ff_ = 0.0f;
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <Preferences.h>
#include <math.h>

static constexpr float TWO_PI_F = 6.28318530718f;

AntiCogging::AntiCogging(BLDCMotor& motor) : m_(motor) {
    memset(lut_, 0, sizeof(lut_));
}

// ---------------------------------------------------------------------------
// Torque command produced by the controller (what's needed to hold position)
// ---------------------------------------------------------------------------
float AntiCogging::commandedTorque() const {
    // current_sp is the torque setpoint output by the angle/velocity cascade.
    // It's in Amps for foc/dc current modes, Volts in voltage mode.
    return m_.current_sp;
}

void AntiCogging::runFocFor(float target, uint32_t us) {
    uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < us) {
        m_.loopFOC();
        m_.move(target);
    }
}

float AntiCogging::sampleHold(float target) {
    runFocFor(target, (uint32_t)(cfg_.settle_ms * 1000.0f));
    double acc = 0.0;
    for (uint16_t s = 0; s < cfg_.samples_per_bin; ++s) {
        runFocFor(target, cfg_.sample_interval_us);
        acc += commandedTorque();
    }
    return (float)(acc / cfg_.samples_per_bin);
}

// One full mechanical revolution, bin-by-bin. out[] indexed by absolute bin.
bool AntiCogging::sweep(float* out, bool reverse, float base,
                        void (*progress)(uint8_t), uint8_t p0, uint8_t p1) {
    const float step = TWO_PI_F / TABLE_SIZE;
    // base = shaft angle that corresponds to sensor mechanical angle 0
    for (uint16_t n = 0; n < TABLE_SIZE; ++n) {
        uint16_t i = reverse ? (TABLE_SIZE - 1 - n) : n;
        float target = base + i * step + (reverse ? TWO_PI_F : 0.0f);
        // Reverse sweep starts one turn ahead so motion is continuous.
        if (reverse) target -= TWO_PI_F;
        out[i] = sampleHold(target);

        if (fabsf(out[i]) >= cfg_.cal_torque_limit * 0.98f) {
            Serial.printf("[AC] saturated at bin %u – raise limit or check mechanics\n", i);
            return false;
        }
        if ((n & 15) == 0) {
            vTaskDelay(1); // feed watchdog / let IDLE run
            if (progress) progress(p0 + (uint8_t)((p1 - p0) * n / TABLE_SIZE));
        }
    }
    return true;
}

bool AntiCogging::calibrate(void (*progress)(uint8_t)) {
    if (!m_.sensor) return false;

    // --- save motor state ---
    MotionControlType prev_ctrl = m_.controller;
    PIDController prev_pid = m_.P_angle;
    PIDController prev_vel = m_.PID_velocity;
    float prev_vlim = m_.velocity_limit;
    bool  prev_en   = enabled_;
    enabled_ = false;

    // --- stiff angle hold: P_angle -> velocity PID with integrator ---
    m_.controller = MotionControlType::angle;
    m_.P_angle.P = 20.0f; m_.P_angle.I = 0; m_.P_angle.D = 0;
    m_.PID_velocity.P = cfg_.cal_P * 0.01f;
    m_.PID_velocity.I = cfg_.cal_I * 0.01f;
    m_.PID_velocity.D = cfg_.cal_D;
    m_.PID_velocity.limit = cfg_.cal_torque_limit;
    m_.velocity_limit = 2.0f;
    m_.PID_velocity.reset(); m_.P_angle.reset();

    // Map sensor mechanical angle 0 to shaft-angle frame.
    m_.loopFOC();
    float mech0 = m_.sensor->getMechanicalAngle();       // 0..2π
    float shaft = m_.shaft_angle;
    // shaft_angle includes sensor_direction; mechanical angle is raw.
    int dir = (m_.sensor_direction == Direction::CCW) ? -1 : 1;
    float base = shaft - dir * mech0;                       // shaft angle where mech==0
    if (dir < 0) {
        Serial.println("[AC] CCW sensor direction: sweep frame inverted");
    }

    static float fwd[TABLE_SIZE];
    static float rev[TABLE_SIZE];
    bool ok;

    // Move to start and pre-settle
    runFocFor(base, 500000);

    // Sweep in "mechanical" order. For CCW sensors, step negative in shaft space.
    auto doSweep = [&](float* buf, bool reverse, uint8_t a, uint8_t b) {
        if (dir > 0) return sweep(buf, reverse, base, progress, a, b);
        // invert: bin i corresponds to shaft = base - i*step
        const float step = TWO_PI_F / TABLE_SIZE;
        for (uint16_t n = 0; n < TABLE_SIZE; ++n) {
            uint16_t i = reverse ? (TABLE_SIZE - 1 - n) : n;
            buf[i] = -sampleHold(base - i * step); // torque sign flips too
            if (fabsf(buf[i]) >= cfg_.cal_torque_limit * 0.98f) return false;
            if ((n & 15) == 0) { vTaskDelay(1);
                if (progress) progress(a + (uint8_t)((b - a) * n / TABLE_SIZE)); }
        }
        return true;
    };

    ok = doSweep(fwd, false, 0, cfg_.bidirectional ? 45 : 90);
    if (ok && cfg_.bidirectional) {
        ok = doSweep(rev, true, 45, 90);
        for (uint16_t i = 0; i < TABLE_SIZE; ++i) fwd[i] = 0.5f * (fwd[i] + rev[i]);
    }

    // --- restore motor state ---
    m_.controller = prev_ctrl;
    m_.P_angle = prev_pid;
    m_.PID_velocity = prev_vel;
    m_.velocity_limit = prev_vlim;
    m_.P_angle.reset(); m_.PID_velocity.reset();

    if (!ok) { enabled_ = prev_en && valid_; return false; }

    // Remove DC (gravity/load bias, residual friction)
    double mean = 0; for (auto v : fwd) mean += v; mean /= TABLE_SIZE;
    for (auto& v : fwd) v -= (float)mean;

    fitHarmonics(fwd);
    rebuildLut();
    valid_ = true;
    enabled_ = true;
    if (progress) progress(100);
    return true;
}

// ---------------------------------------------------------------------------
// DFT -> keep dominant harmonics -> sinusoidal reconstruction
// ---------------------------------------------------------------------------
void AntiCogging::fitHarmonics(const float* raw) {
    n_harm_ = 0;
    uint16_t kmax = min<uint16_t>(cfg_.max_harmonic, MAX_HARMONIC);
    kmax = min<uint16_t>(kmax, TABLE_SIZE / 2 - 1);

    for (uint16_t k = 1; k <= kmax; ++k) {
        double a = 0, b = 0;
        for (uint16_t n = 0; n < TABLE_SIZE; ++n) {
            float th = TWO_PI_F * k * n / TABLE_SIZE;
            a += raw[n] * cosf(th);
            b += raw[n] * sinf(th);
        }
        a *= 2.0 / TABLE_SIZE;
        b *= 2.0 / TABLE_SIZE;
        float amp = sqrtf((float)(a * a + b * b));
        if (amp >= cfg_.min_harmonic_amp && n_harm_ < MAX_HARMONIC) {
            harm_[n_harm_++