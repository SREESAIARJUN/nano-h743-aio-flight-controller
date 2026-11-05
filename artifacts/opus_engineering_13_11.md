# Anti-Cogging Calibration and Feedforward Module for SimpleFOC (ESP32-S3)

I don't have the context of your earlier "cycles," so this is a self-contained module. It targets **SimpleFOC 2.3.x** on the ESP32-S3 (Arduino core 2.x or 3.x). If your project already has conventions for naming, storage keys, or task layout, the integration points are isolated so you can adapt them.

## Overview

**Calibration (blocking, about 50 s by default):**
1. The motor is held in stiff angle control. It steps through one mechanical revolution forward, then one in reverse.
2. At each step it waits to settle, then averages `motor.current_sp`. In steady state, `current_sp` is exactly the torque needed to hold against cogging.
3. Samples are binned by the **measured raw sensor angle** into 512 bins.
4. The forward and reverse maps are averaged. This cancels Coulomb friction, which has opposite sign in each direction.
5. Empty bins are filled by circular interpolation, and the DC offset (gravity or bias) is removed.
6. **Sinusoidal fit:** a DFT is taken and only harmonics `1..maxHarmonic` above a noise threshold are kept. The 512-point table is rebuilt from that sum of sinusoids, which rejects measurement noise.
7. The table is saved to NVS with a CRC, a version number, and a sensor-direction check.

**Runtime:**
- `compensation()` is IRAM-resident. It does an O(1) lookup with linear interpolation, phase advance for loop latency, and fades out with speed.
- `move(target)` replaces `motor.move(target)`. It injects the feedforward in **all** motion modes by adding to `current_sp`, and to `voltage.q` in voltage-torque mode.

---

## `CoggingCompensator.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class CoggingCompensator {
public:
    static constexpr uint16_t TABLE_SIZE   = 512;
    static constexpr uint32_t STORAGE_VER  = 0x434F4731; // "COG1"

    struct Config {
        // --- calibration motion ---
        float    cal_P_angle        = 40.0f;   // stiff position loop during calibration
        float    cal_P_vel          = 0.3f;
        float    cal_I_vel          = 8.0f;
        float    cal_vel_limit      = 4.0f;    // rad/s
        uint16_t settle_ms          = 30;
        uint16_t sample_ms          = 20;
        float    overscan_bins      = 8;       // extra steps to cover bins at wrap
        // --- sinusoidal fit ---
        bool     harmonic_fit       = true;
        uint16_t max_harmonic       = 96;      // < TABLE_SIZE/2
        float    harmonic_rel_thresh= 0.03f;   // keep |c_k| >= thresh * max|c|
        // --- runtime injection ---
        float    gain               = 1.0f;
        float    fade_start_vel     = 30.0f;   // rad/s: full FF below this
        float    fade_end_vel       = 80.0f;   // rad/s: zero FF above this
        float    latency_s          = 100e-6f; // phase-advance for loop delay
        float    max_ff             = 2.0f;    // clamp, units of current_sp
        // --- storage ---
        const char* nvs_namespace   = "cogcomp";
    };

    explicit CoggingCompensator(BLDCMotor& motor);
    CoggingCompensator(BLDCMotor& motor, const Config& cfg);

    // Blocking calibration. Motor must be initFOC()'d. Returns true on success.
    bool calibrate(Print* log = &Serial);

    // Runtime
    float IRAM_ATTR compensation();                 // FF value for current angle
    float IRAM_ATTR lookup(float mech_angle) const; // raw table interp
    void  move(float target);                       // replaces motor.move()

    // Persistence
    bool save();
    bool load();
    void clearStorage();

    // Control
    void  enable(bool en)          { enabled_ = en && valid_; }
    bool  isEnabled() const        { return enabled_; }
    bool  isValid() const          { return valid_; }
    void  setGain(float g)         { cfg_.gain = g; }
    Config& config()               { return cfg_; }
    const float* table() const     { return table_; }
    float lastFF() const           { return last_ff_; }
    void  dumpTable(Print& out) const;

private:
    BLDCMotor& motor_;
    Config     cfg_;
    float      table_[TABLE_SIZE];
    bool       valid_   = false;
    bool       enabled_ = false;
    volatile float last_ff_ = 0.0f;

    static inline float wrap2pi(float a) {
        a = fmodf(a, _2PI);
        return a < 0 ? a + _2PI : a;
    }
    void   sweep(int dir, float* sum, uint16_t* cnt, Print* log);
    void   runFor(uint32_t ms);
    static bool fillGaps(float* t, const uint16_t* cnt);
    void   removeDC(float* t);
    void   harmonicFit(float* t, Print* log);
    uint32_t crc32(const uint8_t* d, size_t n) const;
};
```

---

## `CoggingCompensator.cpp`

```cpp
#include "CoggingCompensator.h"

CoggingCompensator::CoggingCompensator(BLDCMotor& m) : motor_(m), cfg_() {
    memset(table_, 0, sizeof(table_));
}
CoggingCompensator::CoggingCompensator(BLDCMotor& m, const Config& c)
    : motor_(m), cfg_(c) {
    memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------
float IRAM_ATTR CoggingCompensator::lookup(float mech_angle) const {
    float pos  = wrap2pi(mech_angle) * (TABLE_SIZE / _2PI);
    uint32_t i0 = (uint32_t)pos;
    float frac = pos - (float)i0;
    i0 &= (TABLE_SIZE - 1);
    uint32_t i1 = (i0 + 1) & (TABLE_SIZE - 1);
    return table_[i0] + frac * (table_[i1] - table_[i0]);
}

float IRAM_ATTR CoggingCompensator::compensation() {
    if (!enabled_ || !motor_.sensor) { last_ff_ = 0; return 0; }

    float vel = motor_.shaft_velocity;
    float avel = fabsf(vel);
    if (avel >= cfg_.fade_end_vel) { last_ff_ = 0; return 0; }

    // Table is indexed by raw sensor mechanical angle; shaft_velocity is in
    // shaft frame, so convert direction for phase advance.
    float raw = motor_.sensor->getMechanicalAngle();
    float raw_vel = vel * (float)motor_.sensor_direction;
    float ff = lookup(raw + raw_vel * cfg_.latency_s);

    // Table holds torque needed in raw-sensor frame; convert to shaft frame.
    ff *= (float)motor_.sensor_direction;

    if (avel > cfg_.fade_start_vel) {
        float k = (cfg_.fade_end_vel - avel) /
                  (cfg_.fade_end_vel - cfg_.fade_start_vel);
        ff *= k;
    }
    ff *= cfg_.gain;
    ff = _constrain(ff, -cfg_.max_ff, cfg_.max_ff);
    last_ff_ = ff;
    return ff;
}

void CoggingCompensator::move(float target) {
    // Torque mode: inject into target directly (cleanest path).
    if (motor_.controller == MotionControlType::torque) {
        motor_.move(target + compensation());
        return;
    }
    // Velocity / angle modes: let SimpleFOC compute current_sp, then add FF.
    motor_.move(target);
    float ff = compensation();
    if (ff == 0.0f) return;

    motor_.current_sp += ff;
    if (motor_.torque_controller == TorqueControlType::voltage) {
        // In voltage mode move() already computed voltage.q from current_sp.
        float dv = _isset(motor_.phase_resistance) ? ff * motor_.phase_resistance : ff;
        motor_.voltage.q = _constrain(motor_.voltage.q + dv,
                                      -motor_.voltage_limit, motor_.voltage_limit);
    }
    // Current modes: loopFOC() consumes current_sp on next call.
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
void CoggingCompensator::runFor(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        motor_.loopFOC();
        motor_.move();
    }
}

void CoggingCompensator::sweep(int dir, float* sum, uint16_t* cnt, Print* log) {
    const float step  = _2PI / TABLE_SIZE;
    const int   steps = TABLE_SIZE + (int)cfg_.overscan_bins;
    float target = motor_.shaft_angle;

    for (int s = 0; s < steps; ++s) {
        target += dir * step;
        motor_.target = target;
        runFor(cfg_.settle_ms);

        // Average over the sample window
        double acc = 0, ang_s = 0, ang_c = 0;
        uint32_t n = 0, t0 = millis();
        while (millis() - t0 < cfg_.sample_ms) {
            motor_.loopFOC();
            motor_.move();
            float raw = motor_.sensor->getMechanicalAngle();
            acc   += motor_.current_sp;
            ang_s += sinf(raw);   // circular mean of angle
            ang_c += cosf(raw);
            ++n;
        }
        if (!n) continue;
        float raw_mean = wrap2pi(atan2f((float)ang_s, (float)ang_c));
        // current_sp is in shaft frame; store in raw-sensor frame.
        float tq = (float)(acc / n) * (float)motor_.sensor_direction;

        uint32_t bin = ((uint32_t)(raw_mean * (TABLE_SIZE / _2PI) + 0.5f))
                       & (TABLE_SIZE - 1);
        