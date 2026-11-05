I don't have context for what "Cycle 26" refers to (a project milestone, internal spec, or earlier conversation), so this is a self-contained implementation based on standard anti-cogging practice for SimpleFOC 2.3.x. Let me know if your spec differs in table format, storage, or injection point.

## Design Overview

**Calibration**
1. Hold the rotor in angle mode at 512 evenly spaced mechanical positions.
2. At each point, wait for it to settle, then average the torque command needed to hold position.
3. Sweep forward and backward and average the two to cancel friction and hysteresis.
4. Remove the mean (load/friction offset).
5. Run a DFT and keep only the dominant harmonics ("sinusoidal" reconstruction). This rejects noise and makes the feedforward smooth.
6. Store the result in ESP32 NVS with a CRC.

**Runtime**
- An IRAM-resident, linearly interpolated table lookup on mechanical angle.
- The compensation is injected after `motor.move()`, either into `current_sp` (current modes) or `voltage.q` (voltage mode).
- Optional velocity-dependent fade-out.

---

## `AntiCogging.h`

```cpp
#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

#ifndef ANTICOG_TABLE_SIZE
#define ANTICOG_TABLE_SIZE 512
#endif

static_assert((ANTICOG_TABLE_SIZE & (ANTICOG_TABLE_SIZE - 1)) == 0,
              "ANTICOG_TABLE_SIZE must be a power of two");

class AntiCogging {
public:
    struct Config {
        uint16_t settle_ms        = 60;    // wait after moving to each point
        uint16_t samples_per_pt   = 40;    // samples averaged per point
        uint16_t sample_period_us = 500;   // spacing between samples
        uint8_t  max_harmonics    = 24;    // harmonics kept in sinusoidal fit
        float    harmonic_min_rel = 0.02f; // drop harmonics < 2% of largest
        float    gain             = 1.0f;  // runtime feedforward gain
        float    fade_start_rad_s = 30.0f; // start fading comp above this speed
        float    fade_end_rad_s   = 80.0f; // comp fully off above this speed
        float    max_comp         = 0.0f;  // clamp |comp|; 0 = auto (limit)
        const char* nvs_namespace = "anticog";
    };

    enum class Status : uint8_t {
        Uninitialized, Ready, Calibrating, Failed
    };

    using ProgressCb = void (*)(uint16_t idx, uint16_t total, uint8_t pass);

    explicit AntiCogging(BLDCMotor& motor);

    void   begin(const Config& cfg);
    void   begin() { begin(Config{}); }

    // Blocking calibration. Motor must be initFOC()'d and unloaded.
    bool   calibrate(ProgressCb cb = nullptr);

    // Call immediately AFTER motor.move() each loop.
    void   IRAM_ATTR apply();

    // Pure lookup (mechanical angle in rad -> torque cmd in A or V).
    float  IRAM_ATTR lookup(float mech_angle) const;

    bool   save();
    bool   load();
    void   clear();

    void   enable(bool en)        { enabled_ = en && status_ == Status::Ready; }
    bool   isEnabled() const      { return enabled_; }
    Status status() const         { return status_; }
    void   setGain(float g)       { cfg_.gain = g; }
    float  lastComp() const       { return last_comp_; }
    const float* table() const    { return table_; }

    void   dump(Stream& s) const;

private:
    static constexpr uint16_t N   = ANTICOG_TABLE_SIZE;
    static constexpr uint32_t MAGIC   = 0xAC0C1026;
    static constexpr uint16_t VERSION = 1;

    struct __attribute__((packed)) Blob {
        uint32_t magic;
        uint16_t version;
        uint16_t size;
        uint8_t  torque_mode;
        float    table[N];
        uint32_t crc;
    };

    BLDCMotor& motor_;
    Config     cfg_;
    Status     status_  = Status::Uninitialized;
    bool       enabled_ = false;
    float      last_comp_ = 0.0f;

    float table_[N];               // final compensation table
    float raw_fwd_[N];             // calibration scratch
    float raw_bwd_[N];

    float sampleHold(float target_angle);
    void  driveTo(float target_angle, uint32_t ms);
    void  sinusoidalFit(float* data);
    float readTorqueCmd() const;
    static uint32_t crc32(const uint8_t* d, size_t n);
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>
#include <esp_rom_crc.h>

AntiCogging::AntiCogging(BLDCMotor& motor) : motor_(motor) {
    memset(table_, 0, sizeof(table_));
}

void AntiCogging::begin(const Config& cfg) {
    cfg_ = cfg;
    if (cfg_.max_comp <= 0.0f) {
        cfg_.max_comp = (motor_.torque_controller == TorqueControlType::voltage)
                        ? motor_.voltage_limit * 0.5f
                        : motor_.current_limit * 0.5f;
    }
    if (load()) {
        status_ = Status::Ready;
        enabled_ = true;
    } else {
        status_ = Status::Uninitialized;
        enabled_ = false;
    }
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------

float IRAM_ATTR AntiCogging::lookup(float mech_angle) const {
    // normalize to [0, 2pi)
    float a = fmodf(mech_angle, _2PI);
    if (a < 0.0f) a += _2PI;

    float pos  = a * (float)N / _2PI;
    uint32_t i0 = (uint32_t)pos;
    float frac  = pos - (float)i0;
    i0 &= (N - 1);
    uint32_t i1 = (i0 + 1) & (N - 1);

    return table_[i0] + frac * (table_[i1] - table_[i0]);
}

void IRAM_ATTR AntiCogging::apply() {
    if (!enabled_ || !motor_.sensor) { last_comp_ = 0.0f; return; }

    float vel = fabsf(motor_.shaft_velocity);
    float fade = 1.0f;
    if (vel >= cfg_.fade_end_rad_s) {
        last_comp_ = 0.0f;
        return;
    } else if (vel > cfg_.fade_start_rad_s) {
        fade = 1.0f - (vel - cfg_.fade_start_rad_s) /
                      (cfg_.fade_end_rad_s - cfg_.fade_start_rad_s);
    }

    // Table is indexed by sensor mechanical angle (no direction/offset applied),
    // matching how calibration recorded it.
    float ang  = motor_.sensor->getMechanicalAngle();
    float comp = lookup(ang) * cfg_.gain * fade;
    comp = _constrain(comp, -cfg_.max_comp, cfg_.max_comp);
    last_comp_ = comp;

    switch (motor_.torque_controller) {
        case TorqueControlType::voltage:
            motor_.voltage.q = _constrain(motor_.voltage.q + comp,
                                          -motor_.voltage_limit,
                                           motor_.voltage_limit);
            break;
        case TorqueControlType::dc_current:
        case TorqueControlType::foc_current:
            motor_.current_sp = _constrain(motor_.current_sp + comp,
                                           -motor_.current_limit,
                                            motor_.current_limit);
            break;
    }
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

float AntiCogging::readTorqueCmd() const {
    return (motor_.torque_controller == TorqueControlType::voltage)
           ? motor_.voltage.q
           : motor_.current_sp;
}

void AntiCogging::driveTo(float target_angle, uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        motor_.loopFOC();
        motor_.move(target_angle);
    }
}

float AntiCogging::sampleHold(float target_angle) {
    driveTo(target_angle, cfg_.settle_ms);

    double acc = 0.0;
    for (uint16_t s = 0; s < cfg_.samples_per_pt; s++) {
        uint32_t t0 = micros();
        while (micros() - t0 < cfg_.sample_period_us) {
            motor_.loopFOC();
            motor_.move(target_angle);
        }
        acc += readTorqueCmd();
    }
    return (float)(acc / cfg_.samples_per_pt);
}

bool AntiCogging::calibrate(ProgressCb cb) {
    if (!motor_.sensor || motor_.motor_status != FOCMotorStatus::motor_ready) {
        status_ = Status::Failed;
        return false;
    }

    status_  = Status::Calibrating;
    enabled_ = false;

    // Save and override controller settings.
    MotionControlType prev_ctrl = motor_.controller;
    float prev_vlim = motor_.velocity_limit;
    float prev_P    = motor_.P_angle.P;

    motor_.controller     = MotionControlType::angle;
    motor_.velocity_limit = 2.0f;                      // slow, gentle moves
    motor_.P_angle.P      = max(prev_P, 20.0f);        // stiff hold
    motor_.PID_velocity.reset();
    motor_.P_angle.reset();

    // Mechanical angle -> shaft angle relation.
    // shaft_angle = sensor_direction * sensor angle (offset ignored here,
    // compensated by reading the sensor directly).
    const float step = _2PI / (float)N;
    const int   dir  = (int)motor_.sensor_direction;

    // Start reference: current shaft angle aligned to sensor index 0.
    motor_.loopFOC();
    motor_.move();
    float mech0  = motor_.sensor->getMechanicalAngle();
    float shaft0 = motor_.shaft_angle - dir * mech0;   // shaft at mech angle 0

    auto shaftFor = [&](float mech) { return shaft0 + dir * mech; };

    // Pre-roll to index 0 and one extra revolution backlash take-up.
    driveTo(shaftFor(-step * 8), 300);
    driveTo(shaftFor(0.0f), 300);

    // Forward pass
    for (uint16_t i = 0; i < N; i++) {
        float target = shaftFor(i * step);
        float v = sampleHold(target);
        // Bin by actually measured position for robustness
        float m = motor_.sensor->getMechanicalAngle();
        uint16_t idx = (uint16_t)lroundf(m / step) & (N - 1);
        raw_fwd_[idx] = v;
        if (i != idx) raw_fwd_[i] = v;   // ensure coverage if slight mismatch
        if (cb) cb(i, N, 0);
    }

    // Overshoot then backward pass
    driveTo(shaftFor(_2PI + step * 8), 300);
    for (int i = N - 1; i >= 0; i--) {
        float target = shaftFor(i * step);
        float v = sampleHold(target);
        float m = motor_.sensor->getMechanicalAngle();
        uint16_t idx = (uint16_t)lroundf(m / step) & (N - 1);
        raw_bwd_[idx] = v;
        if ((uint16_t)i != idx) raw_bwd_[i] = v;
        if (cb) cb(N - 1 - i, N, 1);
    }

    // Restore settings
    motor_.controller     = prev_ctrl;
    motor_.velocity_limit = prev_vlim;
    motor_.P_angle.P      = prev_P;
    motor_.PID_velocity.reset();
    motor_.P_angle.reset();
    motor_.target = (prev_ctrl == MotionControlType::angle)
                    ? motor_.shaft_angle : 0.0f;

    // Average directions (cancels Coulomb friction), remove DC mean.
    double mean = 0.0;
    for (uint16_t i = 0; i < N; i++) {
        table_[i] = 0.5f * (raw_fwd_[i] + raw_bwd_[i]);
        mean += table_[i];
    }
    mean /= N;
    for (uint16_t i = 0; i < N; i++) table_[i] -= (float)mean;

    // Express in sensor-frame sign so apply() can add directly.
    // Torque in motor frame is already what we inject, so no sign change.
    sinusoidalFit(table_);

    status_  = Status::Ready;
    enabled_ = true;
    return true;
}

// Keep the strongest harmonics only (DFT -> threshold -> inverse).
void AntiCogging::sinusoidalFit(float* data) {
    const uint16_t K = N / 2;
    // Reuse scratch buffers: raw_fwd_ = a_k, raw_bwd_ = b_k (k = 1..K-1)
    float* a = raw_fwd_;
    float* b = raw_bwd_;
    float  mag_max = 0.0f;

    for (uint16_t k = 1; k < K; k++) {
        double sa = 0.0, sb = 0.0;
        for (uint16_t n = 0; n < N; n++) {
            float ph = _2PI * k * n / (float)N;
            sa += data[n] * cosf(ph);
            sb += data[n] * sinf(ph);
        }
        a[k] = (float)(2.0 * sa / N);
        b[k] = (float)(2.0 * sb / N);
        float m = sqrtf(a[k] * a[k] + b[k] * b[k]);
        if (m > mag_max) mag_max = m;
    }

    // Select top-N harmonics above relative threshold.
    bool keep[K];
    memset(keep, 0, sizeof(keep));
    for (uint8_t sel = 0; sel < cfg_.max_harmonics; sel++) {
        float best = 0.0f; int bi = -1;
        for (uint16_t k = 1; k < K; k++) {
            if (keep[k]) continue;
            float m = sqrtf(a[k] * a[k] + b[k] * b[k]