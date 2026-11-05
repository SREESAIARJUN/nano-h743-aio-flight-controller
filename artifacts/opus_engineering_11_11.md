I don't know what "Cycle 11" refers to in your project, so this is a self-contained implementation built on the standard SimpleFOC 2.3.x API. If Cycle 11 implies particular interfaces, naming, or constraints, share them and I'll adapt the code.

## Design overview

| Stage | What happens |
|---|---|
| **Calibration** | The motor runs in closed-loop `angle` mode and steps through 512 mechanical positions, first forward and then in reverse. At each point it waits to settle, then averages the torque command (`current_sp`) the position loop needs to hold that angle. |
| **Friction cancellation** | Averaging the forward and reverse passes cancels Coulomb friction, which flips sign with direction. Cogging depends only on position, so it survives the averaging. |
| **Sinusoidal fit** | A DFT is taken of the 512-point table. It keeps only harmonics 1…N above a noise threshold and drops DC (DC is load or gravity, not cogging). The table is then re-synthesized from those sinusoids, which gives a smooth, noise-free cogging model. |
| **Persistence** | The table is stored in ESP32 NVS (`Preferences`) with a magic number, version, sensor-offset tag and CRC32. |
| **Injection** | The table is linearly interpolated (in IRAM) at the current mechanical angle. The result is added to `motor.current_sp` after every `loopFOC()`, so it works in torque, velocity and angle modes. It fades out above a configurable speed. |

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
    static constexpr uint16_t TABLE_SIZE = 512;          // power of 2 -> cheap wrap
    static constexpr uint32_t STORAGE_MAGIC = 0xC066A512;
    static constexpr uint16_t STORAGE_VERSION = 1;

    struct Config {
        uint16_t settle_ms        = 25;    // wait after each step
        uint16_t samples_per_pt   = 40;    // averaged current_sp samples
        uint16_t sample_interval_us = 250; // spacing between samples
        uint16_t max_harmonic     = 96;    // highest mechanical harmonic kept
        float    noise_floor_ratio = 0.02f;// drop harmonics < ratio * largest
        float    fade_start_rad_s = 20.0f; // full FF below this speed
        float    fade_end_rad_s   = 60.0f; // zero FF above this speed
        float    gain             = 1.0f;  // runtime scaling
        float    max_ff           = 0.0f;  // clamp (0 = use motor.current_limit / voltage_limit)
        // calibration loop tuning (applied temporarily)
        float    cal_P_angle      = 40.0f;
        float    cal_vel_limit    = 4.0f;
    };

    explicit AntiCogging(BLDCMotor& motor, const char* nvs_namespace = "acog");

    // --- Calibration -------------------------------------------------------
    // Blocking. The motor must be initFOC()'d and unloaded.
    // progress_cb(pass 0/1, index 0..511) may be nullptr.
    bool calibrate(void (*progress_cb)(uint8_t, uint16_t) = nullptr);

    // --- Persistence -------------------------------------------------------
    bool save();
    bool load();
    void clear();

    // --- Runtime -------------------------------------------------------------
    // Call right after motor.loopFOC() (and after motor.move()).
    void IRAM_ATTR apply();
    // Pure lookup (no side-effects): FF torque at given mechanical angle.
    float IRAM_ATTR lookup(float mech_angle) const;

    void enable(bool en) { enabled_ = en && valid_; if (!enabled_) last_ff_ = 0; }
    bool isEnabled() const { return enabled_; }
    bool isValid()   const { return valid_; }

    Config& config() { return cfg_; }
    const float* table() const { return table_; }
    float lastFF() const { return last_ff_; }

    void dump(Print& out) const;   // CSV: index,angle,raw_fwd,raw_rev,fitted

private:
    struct StorageHeader {
        uint32_t magic;
        uint16_t version;
        uint16_t size;
        float    zero_electric_angle;  // tag: table invalid if sensor re-aligned
        int32_t  sensor_direction;
        uint32_t crc;
    };

    void  sampleTable(float* dst, bool forward, float base_angle,
                      void (*cb)(uint8_t, uint16_t), uint8_t pass);
    void  serviceMotorFor(uint32_t us);
    void  fitSinusoids(const float* raw, float* out);
    static uint32_t crc32(const uint8_t* data, size_t len);
    float ffLimit() const;

    BLDCMotor&  motor_;
    const char* ns_;
    Config      cfg_;

    float table_[TABLE_SIZE];
    float* raw_fwd_ = nullptr;   // kept only for dump() after calibration
    float* raw_rev_ = nullptr;

    volatile bool valid_   = false;
    volatile bool enabled_ = false;
    float last_ff_ = 0.0f;
    float vel_filt_ = 0.0f;
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>

static constexpr float kTwoPi = 6.28318530718f;
static constexpr float kIdxPerRad = AntiCogging::TABLE_SIZE / kTwoPi;

AntiCogging::AntiCogging(BLDCMotor& motor, const char* ns)
    : motor_(motor), ns_(ns) {
    memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------------------
// Keep FOC + motion loop alive while we wait (calibration is blocking).
// ---------------------------------------------------------------------------
void AntiCogging::serviceMotorFor(uint32_t us) {
    uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < us) {
        motor_.loopFOC();
        motor_.move();
    }
}

// ---------------------------------------------------------------------------
// One sweep over 512 positions, recording the mean holding torque command.
// ---------------------------------------------------------------------------
void AntiCogging::sampleTable(float* dst, bool forward, float base_angle,
                              void (*cb)(uint8_t, uint16_t), uint8_t pass) {
    const float step = kTwoPi / TABLE_SIZE;

    for (uint16_t n = 0; n < TABLE_SIZE; ++n) {
        uint16_t i = forward ? n : (TABLE_SIZE - 1 - n);
        motor_.target = base_angle + i * step;

        serviceMotorFor((uint32_t)cfg_.settle_ms * 1000UL);

        double acc = 0.0;
        for (uint16_t s = 0; s < cfg_.samples_per_pt; ++s) {
            serviceMotorFor(cfg_.sample_interval_us);
            acc += motor_.current_sp;      // torque cmd from position/velocity loop
        }
        // Index by *measured* mechanical angle, not target, to absorb tracking error.
        float mech = motor_.sensor->getMechanicalAngle();
        int idx = (int)lroundf(mech * kIdxPerRad) & (TABLE_SIZE - 1);
        // Use direct index when tracking is tight (normal case); fallback to measured.
        int use = (abs(idx - (int)i) <= 1 || abs(idx - (int)i) >= TABLE_SIZE - 1) ? i : idx;
        dst[use] = (float)(acc / cfg_.samples_per_pt);

        if (cb) cb(pass, n);
        if ((n & 0x1F) == 0) vTaskDelay(1);   // feed the task watchdog
    }
}

// ---------------------------------------------------------------------------
// Sinusoidal (truncated Fourier) fit of the raw table.
// O(N*H) direct DFT: 512 * 96 ~ 50k MACs, trivial on the S3 FPU.
// ---------------------------------------------------------------------------
void AntiCogging::fitSinusoids(const float* raw, float* out) {
    const uint16_t H = min<uint16_t>(cfg_.max_harmonic, TABLE_SIZE / 2 - 1);
    float* a = (float*)calloc(H + 1, sizeof(float));
    float* b = (float*)calloc(H + 1, sizeof(float));
    if (!a || !b) { free(a); free(b); memcpy(out, raw, sizeof(float) * TABLE_SIZE); return; }

    float maxMag = 0.0f;
    for (uint16_t k = 1; k <= H; ++k) {
        double sa = 0, sb = 0;
        for (uint16_t i = 0; i < TABLE_SIZE; ++i) {
            float th = kTwoPi * k * i / TABLE_SIZE;
            sa += raw[i] * cosf(th);
            sb += raw[i] * sinf(th);
        }
        a[k] = (float)(2.0 * sa / TABLE_SIZE);
        b[k] = (float)(2.0 * sb / TABLE_SIZE);
        maxMag = fmaxf(maxMag, sqrtf(a[k] * a[k] + b[k] * b[k]));
    }

    const float thresh = cfg_.noise_floor_ratio * maxMag;
    for (uint16_t k = 1; k <= H; ++k)
        if (sqrtf(a[k] * a[k] + b[k] * b[k]) < thresh) a[k] = b[k] = 0.0f;

    // Re-synthesize (DC intentionally dropped: that's load/gravity, not cogging).
    for (uint16_t i = 0; i < TABLE_SIZE; ++i) {
        float v = 0.0f;
        for (uint16_t k = 1; k <= H; ++k) {
            if (a[k] == 0.0f && b[k] == 0.0f) continue;
            float th = kTwoPi * k * i / TABLE_SIZE;
            v += a[k] * cosf(th) + b[k] * sinf(th);
        }
        out[i] = v;
    }
    free(a); free(b);
}

// ---------------------------------------------------------------------------
bool AntiCogging::calibrate(void (*cb)(uint8_t, uint16_t)) {
    if (!motor_.sensor || motor_.motor_status != FOCMotorStatus::motor_ready) return false;

    enabled_ = false;
    last_ff_ = 0.0f;

    free(raw_fwd_); free(raw_rev_);
    raw_fwd_ = (float*)heap_caps_malloc(sizeof(float) * TABLE_SIZE, MALLOC_CAP_DEFAULT);
    raw_rev_ = (float*)heap_caps_malloc(sizeof(float) * TABLE_SIZE, MALLOC_CAP_DEFAULT);
    if (!raw_fwd_ || !raw_rev_) return false;

    // Save and override motion settings.
    MotionControlType prevCtrl = motor_.controller;
    float prevP   = motor_.P_angle.P;
    float prevVL  = motor_.velocity_limit;
    float prevTgt = motor_.target;

    motor_.controller     = MotionControlType::angle;
    motor_.P_angle.P      = cfg_.cal_P_angle;
    motor_.velocity_limit = cfg_.cal_vel_limit;

    // Align base so table index 0 == mechanical angle 0.
    motor_.loopFOC();
    float shaft = motor_.shaft_angle;
    float mech  = motor_.sensor->getMechanicalAngle();
    float base  = shaft - mech;                    // shaft angle at mech 0 (this revolution)
    // Shaft angle includes sensor_direction; account for it.
    if (motor_.sensor_direction == Direction::CCW) base = shaft + mech;
    // Note: with CCW direction the index runs backward in mech angle;
    // we compensate by mirroring below.

    motor_.target = base;
    serviceMotorFor(500000);                       // move to start and settle

    sampleTable(raw_fwd_, true,  base, cb, 0);
    serviceMotorFor(200000);
    sampleTable(raw_rev_, false, base, cb, 1);

    // Restore settings.
    motor_.controller     = prevCtrl;
    motor_.P_angle.P      = prevP;
    motor_.velocity_limit = prevVL;
    motor_.target         = (prevCtrl == MotionControlType::angle) ? motor_.shaft_angle : prevTgt;

    // Friction cancellation.
    float avg[TABLE_SIZE];
    for (uint16_t i = 0; i < TABLE_SIZE; ++i) avg[i] = 0.5f * (raw_fwd_[i] + raw_rev_[i]);

    if (motor_.sensor_direction == Direction::CCW) {
        // index i was at shaft = base - ... ; mirror into mechanical-angle order
        float tmp[TABLE_SIZE];
        for (uint16_t i = 0; i < TABLE_SIZE; ++i) tmp[i] = avg[(TABLE_SIZE - i) & (TABLE_SIZE - 1)];
        memcpy(avg, tmp, sizeof(avg));
    }

    // The holding command opposes cogging, so it *is* the feedforward (same s