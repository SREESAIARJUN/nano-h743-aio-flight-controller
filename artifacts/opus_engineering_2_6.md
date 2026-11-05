# SimpleFOC Anti-Cogging Module (512-point, ESP32-S3)

I don't have your project's "Cycle 2" spec, so this implementation rests on the assumptions below. Tell me where your spec differs and I'll adjust.

**Assumptions**
- **Indexing:** the table is indexed by the **sensor's mechanical angle** (0…2π, one revolution). Calibration and runtime use the same raw measurement, so sensor direction and zero offset don't matter.
- **Calibration method:**
  1. Slow quasi-static position sweep in angle mode, forward then reverse.
  2. Record the steady-state holding torque (`current_sp`) into 512 bins, using the *measured* position.
  3. Average the forward and reverse passes to cancel Coulomb friction.
  4. Remove the DC offset (gravity or load bias).
  5. Apply a **sinusoidal (harmonic) fit** via DFT, keeping only the strongest N harmonics.
  6. Reconstruct the smoothed 512-point table.
- **Injection:** no library patching is needed. Call `apply()` after `motor.move()` and before the next `motor.loopFOC()`:
  - In current modes it adds to `motor.current_sp`.
  - In voltage mode it adds to `motor.voltage.q`.
- **Velocity fade:** feedforward is faded out at higher speed, where cogging is filtered by inertia and table phase lag hurts more than it helps.
- **Persistence:** the table is stored in ESP32 NVS (`Preferences`).
- **Safety:** calibration is blocking. Run it with the rotor free (no load) and a sane `voltage_limit`/`current_limit`.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
    static constexpr uint16_t N = 512;            // table size (power of 2)
    static constexpr uint16_t MAX_HARMONICS = 64; // DFT search range (harmonic orders 1..MAX_HARMONICS*? see cfg)

    struct Config {
        uint16_t settle_ms        = 4;     // dwell per commanded step
        uint16_t samples_per_step = 8;     // loopFOC/move cycles sampled per step
        uint16_t steps_per_rev    = 2048;  // commanded steps per sweep (>= N for full bin coverage)
        float    sweep_vel_limit  = 2.0f;  // rad/s, velocity limit during calibration
        uint16_t max_harmonic_order = 256; // highest harmonic order considered (< N/2)
        uint8_t  keep_harmonics   = 12;    // number of dominant harmonics kept in fit
        float    gain             = 1.0f;  // runtime feedforward gain
        float    fade_start_vel   = 20.0f; // rad/s: full FF below this
        float    fade_end_vel     = 60.0f; // rad/s: zero FF above this
        float    max_ff           = 0.0f;  // clamp (A or V); 0 = use motor limit
    };

    explicit AntiCogging(BLDCMotor& motor);

    Config cfg;

    // Blocking calibration. Returns true on success.
    bool calibrate(Print* log = &Serial);

    // Call every loop: motor.loopFOC(); motor.move(target); ac.apply();
    void IRAM_ATTR apply();

    // Feedforward for a given mechanical angle [rad], with gain applied (no velocity fade).
    float IRAM_ATTR lookup(float mech_angle) const;

    bool save(const char* ns = "anticog");
    bool load(const char* ns = "anticog");

    void enable(bool en)   { enabled_ = en && valid_; }
    bool isEnabled() const { return enabled_; }
    bool isValid()   const { return valid_; }
    const float* table() const { return table_; }
    void dump(Print& out) const;

private:
    BLDCMotor& m_;
    float table_[N];
    bool valid_   = false;
    bool enabled_ = false;
    float last_ff_ = 0.0f;

    struct Harmonic { uint16_t k; float a; float b; float mag; };

    void runFOC(uint32_t cycles);
    bool sweep(int dir, float* sum, uint16_t* cnt, Print* log);
    void harmonicFit(float* data, Print* log);
    static inline uint16_t angleToIndexFrac(float a, float& frac);
    uint32_t checksum() const;
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>
#include <string.h>

static constexpr float TWO_PI_F = 6.28318530718f;

AntiCogging::AntiCogging(BLDCMotor& motor) : m_(motor) {
    memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------- helpers
inline uint16_t AntiCogging::angleToIndexFrac(float a, float& frac) {
    a = fmodf(a, TWO_PI_F);
    if (a < 0) a += TWO_PI_F;
    float pos = a * (N / TWO_PI_F);
    uint16_t i = (uint16_t)pos;
    frac = pos - (float)i;
    return i & (N - 1);
}

void AntiCogging::runFOC(uint32_t cycles) {
    for (uint32_t c = 0; c < cycles; ++c) {
        m_.loopFOC();
        m_.move();
    }
}

// ---------------------------------------------------------------- runtime
float IRAM_ATTR AntiCogging::lookup(float mech_angle) const {
    float frac;
    uint16_t i0 = angleToIndexFrac(mech_angle, frac);
    uint16_t i1 = (i0 + 1) & (N - 1);
    return cfg.gain * (table_[i0] + frac * (table_[i1] - table_[i0]));
}

void IRAM_ATTR AntiCogging::apply() {
    if (!enabled_ || !m_.sensor) { last_ff_ = 0; return; }

    float v = fabsf(m_.shaft_velocity);
    float fade;
    if (v <= cfg.fade_start_vel)    fade = 1.0f;
    else if (v >= cfg.fade_end_vel) fade = 0.0f;
    else fade = 1.0f - (v - cfg.fade_start_vel) / (cfg.fade_end_vel - cfg.fade_start_vel);
    if (fade <= 0.0f) { last_ff_ = 0; return; }

    float ff = fade * lookup(m_.sensor->getMechanicalAngle());

    float lim = cfg.max_ff;
    if (lim <= 0.0f)
        lim = (m_.torque_controller == TorqueControlType::voltage) ? m_.voltage_limit
                                                                    : m_.current_limit;
    ff = _constrain(ff, -lim, lim);
    last_ff_ = ff;

    if (m_.torque_controller == TorqueControlType::voltage) {
        // move() has already written voltage.q from current_sp
        if (_isset(m_.phase_resistance))
            m_.voltage.q = _constrain(m_.voltage.q + ff * m_.phase_resistance,
                                      -m_.voltage_limit, m_.voltage_limit);
        else
            m_.voltage.q = _constrain(m_.voltage.q + ff,
                                      -m_.voltage_limit, m_.voltage_limit);
    } else {
        // dc_current / foc_current: loopFOC() feeds current_sp into the q-axis current PID
        m_.current_sp = _constrain(m_.current_sp + ff, -m_.current_limit, m_.current_limit);
    }
}

// ---------------------------------------------------------------- calibration
bool AntiCogging::sweep(int dir, float* sum, uint16_t* cnt, Print* log) {
    const float step   = TWO_PI_F / cfg.steps_per_rev;
    const float start  = m_.shaft_angle;
    const uint32_t settle_cycles = 0; // time-based settle below

    for (uint32_t s = 0; s <= cfg.steps_per_rev; ++s) {
        m_.target = start + dir * step * s;

        uint32_t t0 = millis();
        while (millis() - t0 < cfg.settle_ms) { m_.loopFOC(); m_.move(); }
        (void)settle_cycles;

        for (uint16_t k = 0; k < cfg.samples_per_step; ++k) {
            m_.loopFOC();
            m_.move();
            float frac;
            uint16_t idx = angleToIndexFrac(m_.sensor->getMechanicalAngle(), frac);
            if (frac >= 0.5f) idx = (idx + 1) & (N - 1);  // nearest bin
            // current_sp = torque demand of velocity PID (A, or V in voltage mode w/o R)
            sum[idx] += m_.current_sp;
            cnt[idx]++;
        }

        if (log && (s % (cfg.steps_per_rev / 8) == 0))
            log->printf("  %s %3u%%\n", dir > 0 ? "fwd" : "rev",
                        (unsigned)(100UL * s / cfg.steps_per_rev));
    }
    return true;
}

bool AntiCogging::calibrate(Print* log) {
    if (!m_.sensor || m_.motor_status != FOCMotorStatus::motor_ready) {
        if (log) log->println("AC: motor not initialised / no sensor");
        return false;
    }

    // Allocate scratch from heap (PSRAM-friendly on S3)
    float*    fs = (float*)   heap_caps_calloc(N, sizeof(float),    MALLOC_CAP_8BIT);
    float*    rs = (float*)   heap_caps_calloc(N, sizeof(float),    MALLOC_CAP_8BIT);
    uint16_t* fc = (uint16_t*)heap_caps_calloc(N, sizeof(uint16_t), MALLOC_CAP_8BIT);
    uint16_t* rc = (uint16_t*)heap_caps_calloc(N, sizeof(uint16_t), MALLOC_CAP_8BIT);
    if (!fs || !rs || !fc || !rc) {
        if (log) log->println("AC: alloc failed");
        free(fs); free(rs); free(fc); free(rc);
        return false;
    }

    // Save state
    bool  was_enabled  = enabled_;
    auto  saved_ctrl   = m_.controller;
    float saved_vlim   = m_.velocity_limit;
    float saved_target = m_.target;
    enabled_ = false;

    m_.controller     = MotionControlType::angle;
    m_.velocity_limit = cfg.sweep_vel_limit;
    m_.target         = m_.shaft_angle;
    runFOC(2000); // settle at start

    if (log) log->println("AC: forward sweep");
    sweep(+1, fs, fc, log);
    runFOC(1000);
    if (log) log->println("AC: reverse sweep");
    sweep(-1, rs, rc, log);

    // Restore controller
    m_.target         = m_.shaft_angle;
    runFOC(500);
    m_.controller     = saved_ctrl;
    m_.velocity_limit = saved_vlim;
    m_.target         = (saved_ctrl == MotionControlType::angle) ? m_.shaft_angle : saved_target;

    // Combine: bidirectional mean cancels friction (sign flips with direction)
    float*   data = fs; // reuse
    uint16_t missing = 0;
    for (uint16_t i = 0; i < N; ++i) {
        bool hf = fc[i] > 0, hr = rc[i] > 0;
        if (hf && hr)   data[i] = 0.5f * (fs[i] / fc[i] + rs[i] / rc[i]);
        else if (hf)    data[i] = fs[i] / fc[i];
        else if (hr)    data[i] = rs[i] / rc[i];
        else          { data[i] = NAN; missing++; }
    }

    // Fill empty bins by circular linear interpolation
    if (missing == N) {
        if (log) log->println("AC: no samples collected");
        free(fs); free(rs); free(fc); free(rc);
        enabled_ = was_enabled;
        return false;
    }
    for (uint16_t i = 0; i < N; ++i) {
        if (!isnan(data[i])) continue;
        uint16_t p = i, q = i, dp = 0, dq = 0;
        do { p = (p - 1) & (N - 1); dp++; } while (isnan(data[p]));
        do { q = (q + 1) & (N - 1); dq++; } while (isnan(data[q]));
        data[i] = data[p] + (data[q] - data[p]) * (float)dp / (float)(dp + dq);
    }
    if (log && missing) log->printf("AC: interpolated %u empty bins\n", missing);

    // Remove DC (load/gravity bias)
    float mean = 0;
    for (uint16_t i = 0; i < N; ++i) mean += data[i];
    mean /= N;
    for (uint16_t i = 0; i < N; ++