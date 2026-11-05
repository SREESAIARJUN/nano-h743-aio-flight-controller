# SimpleFOC Anti-Cogging Module for ESP32-S3

I don't know what "Cycle 6" refers to, since it isn't a SimpleFOC or Espressif term I recognize. If it's part of your project's spec, check the assumptions below against it. Otherwise, this is a complete, self-contained implementation.

## Design Summary

| Aspect | Approach |
|---|---|
| Table | 512 bins over one **mechanical** revolution, indexed by `sensor->getMechanicalAngle()` |
| Calibration | Closed-loop position hold at 512 points, swept **forward and backward**. Averaging the two directions cancels Coulomb friction and hysteresis. |
| Sinusoidal model | Discrete Fourier fit of the raw table, keeping harmonics up to `H` (optionally only the top-K by magnitude), then reconstruction. This removes noise and optionally DC and the 1st harmonic (gravity or eccentric load). |
| Injection | Added to `motor.current_sp` after `motor.move()`. It is consumed by the next `loopFOC()` in voltage, DC-current and FOC-current torque modes. Linear interpolation, with fade-out at high speed. |
| Persistence | ESP32 NVS (`Preferences`), with magic, version, motor-signature and CRC32 validation |
| ESP32-S3 | Hot path in `IRAM_ATTR`, float-only math (the S3 has a single-precision FPU), atomic enable flag for dual-core use |

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <atomic>

class AntiCogging {
public:
    static constexpr uint16_t N = 512;           // table resolution
    static constexpr uint16_t MAX_HARMONICS = 128;

    struct Config {
        // --- calibration ---
        float    settle_time_ms      = 40.0f;   // wait after each step
        uint16_t samples_per_point   = 32;      // averaged current_sp samples
        float    sample_interval_us  = 200.0f;
        float    max_settle_vel      = 0.5f;    // rad/s, must be below to sample
        float    max_wait_ms         = 400.0f;  // abort-wait per point
        float    angle_P             = 0.0f;    // 0 = keep user's P_angle
        uint16_t sweep_overshoot     = 8;       // extra points to unwind backlash
        // --- sinusoidal fit ---
        uint16_t harmonics           = 64;      // keep harmonics 1..H
        uint16_t keep_top_k          = 0;       // 0 = keep all within H
        bool     remove_dc           = true;    // DC = static load / offset
        bool     remove_first_harm   = false;   // true if gravity/eccentric load
        // --- runtime injection ---
        float    gain                = 1.0f;    // overall FF scale
        float    max_ff              = 0.0f;    // 0 = no clamp, else |ff| limit
        float    fade_start_vel      = 20.0f;   // rad/s: full FF below
        float    fade_end_vel        = 60.0f;   // rad/s: zero FF above
    };

    enum class Status : uint8_t {
        OK = 0, NOT_INITIALIZED, SENSOR_MISSING, MOTOR_NOT_READY,
        SETTLE_TIMEOUT, ABORTED, NVS_ERROR, NVS_INVALID
    };

    explicit AntiCogging(BLDCMotor& motor);

    void   setConfig(const Config& c) { cfg_ = c; }
    Config& config() { return cfg_; }

    // Blocking calibration (~40-90 s). Motor must be initFOC()'d.
    // progress callback: (phase 0=fwd 1=bwd 2=fit, index, total)
    using ProgressCb = void (*)(uint8_t phase, uint16_t i, uint16_t total);
    Status calibrate(ProgressCb cb = nullptr);
    void   requestAbort() { abort_.store(true); }

    // Runtime: call after motor.move() each loop iteration.
    void IRAM_ATTR inject();
    float IRAM_ATTR feedforward(float mech_angle, float velocity) const;

    void enable(bool en) { enabled_.store(en && valid_); }
    bool enabled() const { return enabled_.load(); }
    bool valid()   const { return valid_; }

    // Persistence
    Status save(const char* ns = "anticog");
    Status load(const char* ns = "anticog");
    Status erase(const char* ns = "anticog");

    // Diagnostics
    const float* table()    const { return table_; }
    const float* rawTable() const { return raw_; }
    float peakToPeak() const;
    float rmsResidual() const;               // raw vs fitted
    void  dump(Stream& s) const;             // CSV: idx,angle,raw,fit

private:
    struct __attribute__((packed)) BlobHeader {
        uint32_t magic;
        uint16_t version;
        uint16_t n;
        int8_t   sensor_dir;
        uint8_t  pole_pairs;
        uint16_t reserved;
        float    zero_electric_angle;
        uint32_t crc;
    };
    static constexpr uint32_t MAGIC   = 0x41434F47; // 'ACOG'
    static constexpr uint16_t VERSION = 2;

    Status sweep(bool forward, float* sum, uint16_t* cnt,
                 ProgressCb cb, uint8_t phase);
    Status holdAndSample(float target, float& meas_angle, float& meas_cmd);
    void   runFor(float ms);
    void   fillEmptyBins(float* t, const uint16_t* cnt);
    void   fitSinusoidal();
    static uint32_t crc32(const uint8_t* d, size_t len, uint32_t crc = 0);
    static inline uint16_t angleToBin(float a);

    BLDCMotor& m_;
    Config cfg_;
    float  table_[N];      // fitted, used at runtime
    float  raw_[N];        // raw calibration data
    bool   valid_ = false;
    std::atomic<bool> enabled_{false};
    std::atomic<bool> abort_{false};
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <Preferences.h>
#include <math.h>

static constexpr float TWO_PI_F  = 6.28318530718f;
static constexpr float BIN_SCALE = AntiCogging::N / TWO_PI_F;

AntiCogging::AntiCogging(BLDCMotor& motor) : m_(motor) {
    memset(table_, 0, sizeof(table_));
    memset(raw_,   0, sizeof(raw_));
}

inline uint16_t AntiCogging::angleToBin(float a) {
    a = fmodf(a, TWO_PI_F);
    if (a < 0) a += TWO_PI_F;
    uint16_t b = (uint16_t)(a * BIN_SCALE + 0.5f);
    return b % N;
}

// ======================= RUNTIME =======================

float IRAM_ATTR AntiCogging::feedforward(float mech_angle, float vel) const {
    // linear interpolation in table
    float a = mech_angle;
    if (a < 0.0f || a >= TWO_PI_F) {
        a = fmodf(a, TWO_PI_F);
        if (a < 0.0f) a += TWO_PI_F;
    }
    float  f  = a * BIN_SCALE;
    uint16_t i0 = (uint16_t)f;
    float  fr = f - (float)i0;
    i0 %= N;
    uint16_t i1 = (i0 + 1) % N;
    float ff = table_[i0] + fr * (table_[i1] - table_[i0]);

    // speed fade: at high speed inertia filters cogging, and
    // sensor/loop latency makes the phase of the FF wrong
    float av = fabsf(vel);
    if (av > cfg_.fade_start_vel) {
        if (av >= cfg_.fade_end_vel) return 0.0f;
        ff *= 1.0f - (av - cfg_.fade_start_vel) /
                     (cfg_.fade_end_vel - cfg_.fade_start_vel);
    }
    ff *= cfg_.gain;
    if (cfg_.max_ff > 0.0f) ff = _constrain(ff, -cfg_.max_ff, cfg_.max_ff);
    return ff;
}

void IRAM_ATTR AntiCogging::inject() {
    if (!enabled_.load(std::memory_order_relaxed) || !m_.sensor) return;
    if (m_.controller == MotionControlType::velocity_openloop ||
        m_.controller == MotionControlType::angle_openloop) return;

    float ff = feedforward(m_.sensor->getMechanicalAngle(), m_.shaft_velocity);
    m_.current_sp += ff;
    // respect the motor's limits
    float lim = (m_.torque_controller == TorqueControlType::voltage &&
                 !_isset(m_.phase_resistance))
                ? m_.voltage_limit : m_.current_limit;
    m_.current_sp = _constrain(m_.current_sp, -lim, lim);
}

// ======================= CALIBRATION =======================

void AntiCogging::runFor(float ms) {
    uint32_t t0 = micros();
    uint32_t dur = (uint32_t)(ms * 1000.0f);
    while ((micros() - t0) < dur) {
        m_.loopFOC();
        m_.move();
    }
}

AntiCogging::Status AntiCogging::holdAndSample(float target,
                                               float& meas_angle,
                                               float& meas_cmd) {
    m_.target = target;
    runFor(cfg_.settle_time_ms);

    // wait for velocity to die down
    uint32_t t0 = millis();
    while (fabsf(m_.shaft_velocity) > cfg_.max_settle_vel) {
        m_.loopFOC(); m_.move();
        if (millis() - t0 > cfg_.max_wait_ms) break;   // sample anyway
        if (abort_.load()) return Status::ABORTED;
    }

    // average commanded torque (= holding torque) and actual angle.
    // Angles are averaged as unit vectors to handle wrap at 0/2π.
    double cmd_sum = 0, s_sum = 0, c_sum = 0;
    for (uint16_t k = 0; k < cfg_.samples_per_point; ++k) {
        uint32_t ts = micros();
        while ((micros() - ts) < (uint32_t)cfg_.sample_interval_us) {
            m_.loopFOC(); m_.move();
        }
        float a = m_.sensor->getMechanicalAngle();
        s_sum += sinf(a); c_sum += cosf(a);
        cmd_sum += m_.current_sp;
    }
    meas_angle = atan2f((float)s_sum, (float)c_sum);
    meas_cmd   = (float)(cmd_sum / cfg_.samples_per_point);
    return abort_.load() ? Status::ABORTED : Status::OK;
}

AntiCogging::Status AntiCogging::sweep(bool forward, float* sum,
                                       uint16_t* cnt, ProgressCb cb,
                                       uint8_t phase) {
    const float step = TWO_PI_F / N;
    const float dir  = forward ? 1.0f : -1.0f;
    const float start = m_.shaft_angle;

    // pre-travel to take up backlash/hysteresis in the sweep direction
    for (uint16_t k = 1; k <= cfg_.sweep_overshoot; ++k) {
        m_.target = start + dir * step * k;
        runFor(cfg_.settle_time_ms * 0.5f);
    }
    const float base = m_.target;

    for (uint16_t i = 0; i < N; ++i) {
        float a, c;
        Status s = holdAndSample(base + dir * step * i, a, c);
        if (s != Status::OK) return s;
        uint16_t b = angleToBin(a);
        sum[b] += c;
        cnt[b]++;
        if (cb && (i & 0x0F) == 0) cb(phase, i, N);
    }
    return Status::OK;
}

void AntiCogging::fillEmptyBins(float* t, const uint16_t* cnt) {
    // circular linear interpolation across bins with no samples
    int first = -1;
    for (int i = 0; i < N; ++i) if (cnt[i]) { first = i; break; }
    if (first < 0) { memset(t, 0, sizeof(float) * N); return