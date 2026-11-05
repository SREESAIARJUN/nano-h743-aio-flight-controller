# Anti-Cogging Compensation Module for SimpleFOC on ESP32-S3

I don't have any context for what "Cycle 17" refers to. It may be a project plan or an earlier conversation I can't see. Below is a complete, self-contained implementation that meets the technical spec you described. If Cycle 17 has specific requirements, tell me and I'll adapt the code. Examples include a particular harmonic count, storage format, or injection point.

## How It Works

**Calibration**
1. The motor is held in closed-loop angle mode at each of 512 evenly spaced mechanical positions.
2. At each position, the module averages the torque command the controller needs to hold still (`motor.current_sp`, in amps or volts depending on torque mode).
3. It sweeps forward and then in reverse, and averages the two passes to cancel Coulomb friction and hysteresis.
4. It fits a truncated Fourier series (the "sinusoidal" model) to the raw table. This removes DC offset, noise, and stiction artifacts.
5. It writes the reconstructed 512-point table to NVS flash.

**Runtime**
- It looks up the table by mechanical angle with linear interpolation, applies a gain and a velocity fade-out, and adds the result to the torque command.
- The lookup runs from IRAM and is O(1). That makes it safe to call every FOC loop.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

class AntiCogging {
public:
    static constexpr int   TABLE_SIZE = 512;
    static constexpr float TWO_PI_F   = 6.28318530718f;

    struct Config {
        uint16_t settle_ms       = 60;    // wait after each move before sampling
        uint16_t samples         = 200;   // FOC-loop samples averaged per point
        uint8_t  harmonics       = 48;    // Fourier harmonics kept in fit (1..TABLE_SIZE/2)
        bool     bidirectional   = true;  // forward+reverse sweep to cancel friction
        float    max_abs_comp    = 0.0f;  // clamp on table magnitude (0 = no clamp)
        uint32_t timeout_ms      = 600000;// overall calibration timeout
    };

    explicit AntiCogging(BLDCMotor& motor);

    // ---- Calibration (blocking; motor must be initialized with initFOC()) ----
    bool calibrate(const Config& cfg = Config(), Stream* log = &Serial);

    // ---- Runtime ----
    // Returns feedforward torque (same units as motor torque target) for the current shaft position.
    float IRAM_ATTR feedforward();
    // Returns feedforward at an explicit mechanical angle [rad].
    float IRAM_ATTR lookup(float mech_angle) const;
    // Convenience wrapper: call in loop() instead of motor.move() when in MotionControlType::torque.
    void  move(float torque_target);

    // ---- Tuning ----
    void  enable(bool en)            { enabled_ = en; }
    bool  isEnabled() const          { return enabled_ && valid_; }
    void  setGain(float g)           { gain_ = g; }
    // Linear fade-out of compensation between v_start and v_end [rad/s]
    void  setVelocityFade(float v_start, float v_end) { fade_start_ = v_start; fade_end_ = v_end; }

    // ---- Persistence (ESP32 NVS) ----
    bool save(const char* ns = "acog");
    bool load(const char* ns = "acog");
    void clear();

    // ---- Diagnostics ----
    void dump(Stream& s) const;      // CSV: index,angle,raw,fitted
    float peakToPeak() const;
    bool isValid() const             { return valid_; }

private:
    BLDCMotor& motor_;
    float table_[TABLE_SIZE];        // fitted table used at runtime
    float raw_[TABLE_SIZE];          // raw measured table (diagnostics)
    bool  valid_      = false;
    bool  enabled_    = true;
    float gain_       = 1.0f;
    float fade_start_ = 50.0f;
    float fade_end_   = 150.0f;

    float readMechAngle() const;
    bool  holdAndSample(float target_shaft_angle, const Config& cfg, float& out, uint32_t t_deadline);
    void  fourierFit(uint8_t harmonics);
    static float wrap2pi(float a);
    uint32_t crc32(const uint8_t* data, size_t len) const;
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <Preferences.h>
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& motor) : motor_(motor) {
    clear();
}

void AntiCogging::clear() {
    for (int i = 0; i < TABLE_SIZE; ++i) { table_[i] = 0.0f; raw_[i] = 0.0f; }
    valid_ = false;
}

float AntiCogging::wrap2pi(float a) {
    a = fmodf(a, TWO_PI_F);
    return (a < 0.0f) ? a + TWO_PI_F : a;
}

float AntiCogging::readMechAngle() const {
    // Sensor mechanical angle in [0, 2π), independent of full rotations.
    return motor_.sensor ? wrap2pi(motor_.sensor->getMechanicalAngle()) : 0.0f;
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
bool AntiCogging::holdAndSample(float target, const Config& cfg, float& out, uint32_t deadline) {
    // Settle
    uint32_t t0 = millis();
    while (millis() - t0 < cfg.settle_ms) {
        motor_.loopFOC();
        motor_.move(target);
        if (millis() > deadline) return false;
    }
    // Average the torque command required to hold position
    double acc = 0.0;
    for (uint16_t n = 0; n < cfg.samples; ++n) {
        motor_.loopFOC();
        motor_.move(target);
        acc += motor_.current_sp;      // torque setpoint (A or V depending on torque_controller)
        if (millis() > deadline) return false;
    }
    out = (float)(acc / cfg.samples);
    return true;
}

bool AntiCogging::calibrate(const Config& cfg, Stream* log) {
    if (!motor_.sensor) { if (log) log->println("ACOG: no sensor"); return false; }

    // Save controller state
    MotionControlType prev_mode = motor_.controller;
    bool prev_en = enabled_;
    enabled_ = false;
    motor_.controller = MotionControlType::angle;

    const uint32_t deadline = millis() + cfg.timeout_ms;
    const float step = TWO_PI_F / TABLE_SIZE;

    // Base shaft angle: align bin 0 with sensor mechanical zero
    motor_.loopFOC();
    float mech0  = readMechAngle();
    float shaft0 = motor_.shaft_angle - mech0;   // shaft angle corresponding to mech angle 0
    // Direction-adjusted: shaft_angle already includes sensor_direction, mech angle may not.
    float dirSign = (motor_.sensor_direction == Direction::CCW) ? -1.0f : 1.0f;

    static float fwd[TABLE_SIZE];
    static float rev[TABLE_SIZE];

    if (log) log->printf("ACOG: forward sweep (%d pts)\n", TABLE_SIZE);
    // Pre-roll to approach bin 0 from the forward direction
    if (!holdAndSample(shaft0 - step * 4 * dirSign, cfg, fwd[0], deadline)) goto fail;

    for (int i = 0; i < TABLE_SIZE; ++i) {
        float tgt = shaft0 + dirSign * i * step;
        if (!holdAndSample(tgt, cfg, fwd[i], deadline)) goto fail;
        if (log && (i % 64 == 0)) log->printf("  %3d/%d  tq=%.4f\n", i, TABLE_SIZE, fwd[i]);
    }

    if (cfg.bidirectional) {
        if (log) log->println("ACOG: reverse sweep");
        float dummy;
        if (!holdAndSample(shaft0 + dirSign * (TABLE_SIZE + 4) * step, cfg, dummy, deadline)) goto fail;
        for (int i = TABLE_SIZE - 1; i >= 0; --i) {
            float tgt = shaft0 + dirSign * i * step;
            if (!holdAndSample(tgt, cfg, rev[i], deadline)) goto fail;
        }
        for (int i = 0; i < TABLE_SIZE; ++i) raw_[i] = 0.5f * (fwd[i] + rev[i]);
    } else {
        for (int i = 0; i < TABLE_SIZE; ++i) raw_[i] = fwd[i];
    }

    // Return to start, restore mode
    motor_.controller = prev_mode;
    enabled_ = prev_en;

    fourierFit(cfg.harmonics);

    if (cfg.max_abs_comp > 0.0f) {
        for (int i = 0; i < TABLE_SIZE; ++i)
            table_[i] = constrain(table_[i], -cfg.max_abs_comp, cfg.max_abs_comp);
    }

    valid_ = true;
    if (log) log->printf("ACOG: done, p-p = %.4f\n", peakToPeak());
    return true;

fail:
    motor_.controller = prev_mode;
    enabled_ = prev_en;
    if (log) log->println("ACOG: calibration timeout/failure");
    return false;
}

// ---------------------------------------------------------------------------
// Sinusoidal (truncated Fourier) fit. DC is removed: gravity/friction offset
// is not cogging and should be handled by the main controller.
// ---------------------------------------------------------------------------
void AntiCogging::fourierFit(uint8_t harmonics) {
    const int H = constrain((int)harmonics, 1, TABLE_SIZE / 2);
    static float a[TABLE_SIZE / 2 + 1];
    static float b[TABLE_SIZE / 2 + 1];

    for (int k = 1; k <= H; ++k) {
        double sa = 0.0, sb = 0.0;
        for (int n = 0; n < TABLE_SIZE; ++n) {
            float th = TWO_PI_F * k * n / TABLE_SIZE;
            sa += raw_[n] * cosf(th);
            sb += raw_[n] * sinf(th);
        }
        float scale = (k == TABLE_SIZE / 2) ? 1.0f : 2.0f;
        a[k] = scale * (float)(sa / TABLE_SIZE);
        b[k] = scale * (float)(sb / TABLE_SIZE);
    }
    for (int n = 0; n < TABLE_SIZE; ++n) {
        float v = 0.0f;
        for (int k = 1; k <= H; ++k) {
            float th = TWO_PI_F * k * n / TABLE_SIZE;
            v += a[k] * cosf(th) + b[k] * sinf(th);
        }
        table_[n] = v;
    }
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------
float IRAM_ATTR AntiCogging::lookup(float mech_angle) const {
    float pos = wrap2pi(mech_angle) * (TABLE_SIZE / TWO_PI_F);
    int   i0  = (int)pos;
    float fr  = pos - (float)i0;
    i0 &= (TABLE_SIZE - 1);                     // TABLE_SIZE is a power of two
    int   i1  = (i0 + 1) & (TABLE_SIZE - 1);
    return table_[i0] + fr * (table_[i1] - table_[i0]);
}

float IRAM_ATTR AntiCogging::feedforward() {
    if (!enabled_ || !valid_) return 0.0f;
    float ff = gain_ * lookup(readMechAngle());

    float v = fabsf(motor_.shaft_velocity);
    if (v >= fade_end_) return 0.0f;
    if (v > fade_start_ && fade_end_ > fade_start_)
        ff *= 1.0f - (v - fade_start_) / (fade_end_ - fade_start_);
    return ff;
}

void AntiCogging::move(float torque_target) {
    // Intended for MotionControlType::torque. For velocity/angle control,
    // run your outer loop externally and pass its output here.
    motor_.move(torque_target + feedforward());
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------
uint32_t AntiCogging::crc32(const uint8_t* d, size_t len) const {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        c ^= d[i];
        for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320u & (-(int32_t)(c & 1)));
    }
    return ~c;
}

bool AntiCogging::save(const char* ns) {
    if (!valid_) return false;
    Preferences p;
    if (!p.begin(ns, false)) return false;
    size_t w = p.putBytes("tbl", table_, sizeof(table_));
    p.putUInt("crc", crc32((const uint8_t*)table_, sizeof(table_)));
    p.putUShort("n", TABLE_SIZE);
    p.end();
    return w == sizeof(table_);
}

bool AntiCogging::load(const char* ns) {
    Preferences p;
    if (!p.begin(ns, true)) return false;
    bool ok = (p.getUShort("n", 0) == TABLE_SIZE) &&
              (p.getBytesLength("tbl") == sizeof(table_)) &&
              (p.getBytes("tbl", table_, sizeof(table_)) == sizeof(table_));
    uint32_t crc = p.getUInt("crc", 0);
    p.end();
    valid_ = ok && (crc == crc