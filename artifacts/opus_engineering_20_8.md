# Anti-Cogging Calibration & Feedforward Module for SimpleFOC (ESP32-S3)

I don't have context for what "Cycle 20" refers to in your project, so this is a self-contained implementation built on SimpleFOC 2.3.x public APIs. If Cycle 20 has specific interface requirements (naming, storage layout, units), tell me and I'll adapt it.

## Design Overview

| Aspect | Approach |
|---|---|
| Table | 512 bins over one mechanical revolution, indexed by the raw sensor mechanical angle |
| Calibration | Position-hold at every bin, in both directions. The averaged control effort cancels hysteresis and stiction. |
| "Sinusoidal" | A DFT of the raw table keeps only significant harmonics, then reconstructs a smooth table. This removes noise and the bias from PID windup. |
| Injection | Applied after `move()` and before `loopFOC()`. It adds to `current_sp` (FOC/DC current modes) or `voltage.q` (voltage mode). |
| Runtime | `IRAM_ATTR` lookup with linear interpolation and velocity-based latency lead. The feedforward fades out at high speed. |
| Persistence | ESP32 NVS (`Preferences`), with a validity check on pole pairs, sensor direction, torque mode and a CRC |

---

## `AntiCogging.h`

```cpp
#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
    static constexpr uint16_t TABLE_SIZE     = 512;
    static constexpr uint16_t MAX_HARMONICS  = TABLE_SIZE / 2;
    static constexpr uint32_t STORAGE_MAGIC  = 0xAC6C0512;
    static constexpr uint16_t STORAGE_VER    = 1;

    struct CalibrationConfig {
        uint16_t settle_ms        = 60;     // time to settle at each bin
        uint16_t sample_ms        = 40;     // averaging window per bin
        uint16_t harmonics_keep   = 24;     // max harmonics retained in fit
        float    harmonic_min_rel = 0.02f;  // drop harmonics < 2% of largest
        bool     bidirectional    = true;   // forward + reverse pass
        float    hold_velocity_limit = 2.0f;// rad/s, limit for moving between bins
        bool     verbose          = true;
    };

    struct RuntimeConfig {
        float gain            = 1.0f;   // 0..1+, global FF scaling
        float fade_start_vel  = 30.0f;  // rad/s: full FF below this
        float fade_end_vel    = 80.0f;  // rad/s: zero FF above this
        float latency_s       = 0.0f;   // phase lead: angle += vel * latency
        float max_ff          = 0.0f;   // clamp |ff| (0 = auto: 1.2 x table peak)
    };

    explicit AntiCogging(BLDCMotor& motor);

    // Calibration: blocking, motor must be initFOC()'d and free to rotate unloaded
    bool calibrate(const CalibrationConfig& cfg = CalibrationConfig());

    // Runtime: call between motor.move() and motor.loopFOC()
    void IRAM_ATTR apply();

    // Pure lookup (useful for torque-mode users wanting target + ff themselves)
    float IRAM_ATTR lookup(float mech_angle) const;

    // Persistence
    bool save(const char* ns = "anticog");
    bool load(const char* ns = "anticog");
    bool erase(const char* ns = "anticog");

    // Control
    void enable(bool en) { enabled_ = en && valid_; }
    bool isEnabled() const { return enabled_; }
    bool isValid()   const { return valid_; }
    RuntimeConfig runtime;

    // Diagnostics
    float lastFF() const { return last_ff_; }
    float tablePeak() const { return peak_; }
    const float* table() const { return table_; }
    void  dumpTable(Stream& s) const;
    void  dumpHarmonics(Stream& s) const;

private:
    struct __attribute__((packed)) Header {
        uint32_t magic;
        uint16_t version;
        uint16_t size;
        int8_t   pole_pairs;
        int8_t   sensor_dir;
        uint8_t  torque_ctrl;
        uint8_t  reserved;
        uint32_t crc;
    };

    BLDCMotor& motor_;
    float table_[TABLE_SIZE];
    float harm_mag_[MAX_HARMONICS + 1];
    bool  valid_   = false;
    bool  enabled_ = false;
    float peak_    = 0.0f;
    volatile float last_ff_ = 0.0f;

    // Calibration helpers
    void  runFor(uint32_t ms, float target, double* acc, uint32_t* n);
    bool  sweep(const CalibrationConfig& cfg, float* out, bool reverse);
    void  harmonicFilter(float* data, const CalibrationConfig& cfg);
    float binAngleToShaft(uint16_t i, float sensor_zero_mech, float shaft_zero) const;
    static uint32_t crc32(const uint8_t* d, size_t n, uint32_t crc = 0xFFFFFFFF);
    float readControlEffort() const;
    void  computePeak();
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>

static constexpr float TWO_PI_F = 6.28318530718f;

AntiCogging::AntiCogging(BLDCMotor& motor) : motor_(motor) {
    memset(table_, 0, sizeof(table_));
    memset(harm_mag_, 0, sizeof(harm_mag_));
}

// ---------------------------------------------------------------------------
// Control effort = the quantity we will later inject.
// In FOC/DC current mode the velocity PID output (current_sp) is in amps.
// In voltage mode it's volts (or amps if phase_resistance is set; current_sp
// still holds the PID output either way, which is what we want).
// ---------------------------------------------------------------------------
float AntiCogging::readControlEffort() const {
    return motor_.current_sp;
}

// Run the FOC loop for `ms`, optionally accumulating control effort
void AntiCogging::runFor(uint32_t ms, float target, double* acc, uint32_t* n) {
    uint32_t t0 = millis();
    uint32_t lastYield = t0;
    while (millis() - t0 < ms) {
        motor_.loopFOC();
        motor_.move(target);
        if (acc) { *acc += readControlEffort(); (*n)++; }
        // keep the task watchdog happy without starving the FOC loop
        if (millis() - lastYield > 20) { vTaskDelay(1); lastYield = millis(); }
    }
}

// Map table bin i (indexed in raw sensor mechanical angle) to a shaft-angle target.
// shaft_angle = sensor_direction * sensor_angle (+ offset), so we compute
// the shaft target relative to the reference captured at calibration start.
float AntiCogging::binAngleToShaft(uint16_t i, float sensor_zero_mech, float shaft_zero) const {
    float bin_mech = (TWO_PI_F * i) / TABLE_SIZE;
    float delta = bin_mech - sensor_zero_mech;           // in sensor frame
    // unwrap to shortest forward motion within [0, 2pi)
    while (delta < 0)        delta += TWO_PI_F;
    while (delta >= TWO_PI_F) delta -= TWO_PI_F;
    return shaft_zero + (int)motor_.sensor_direction * delta;
}

bool AntiCogging::sweep(const CalibrationConfig& cfg, float* out, bool reverse) {
    // Reference: current raw sensor angle and shaft angle
    motor_.sensor->update();
    float sensor_mech = motor_.sensor->getMechanicalAngle();
    float shaft_zero  = motor_.shaft_angle;

    // Start bin = bin at/after current position
    uint16_t start = (uint16_t)ceilf(sensor_mech * TABLE_SIZE / TWO_PI_F) % TABLE_SIZE;
    float sensor_zero = sensor_mech;

    for (uint16_t k = 0; k < TABLE_SIZE; k++) {
        // forward: start, start+1, ... ; reverse: walk back through the same range
        uint16_t step = reverse ? (TABLE_SIZE - 1 - k) : k;
        uint16_t bin  = (start + step) % TABLE_SIZE;

        // continuous shaft target (avoid wrap jump: use step count, not bin)
        float delta = ((float)step / TABLE_SIZE) * TWO_PI_F
                    + (((float)start / TABLE_SIZE) * TWO_PI_F - sensor_zero);
        float target = shaft_zero + (int)motor_.sensor_direction * delta;

        runFor(cfg.settle_ms, target, nullptr, nullptr);

        double acc = 0; uint32_t n = 0;
        runFor(cfg.sample_ms, target, &acc, &n);
        if (n == 0) return false;
        out[bin] = (float)(acc / n);

        // safety: abort if tracking is badly lost (stall / sensor fault)
        if (fabsf(motor_.shaft_angle - target) > (TWO_PI_F / TABLE_SIZE) * 20.0f) {
            if (cfg.verbose) Serial.printf("[AC] tracking error too large at bin %u\n", bin);
            return false;
        }
        if (cfg.verbose && (k % 64 == 0))
            Serial.printf("[AC] %s %3u/%u  bin=%3u  eff=%.4f\n",
                          reverse ? "rev" : "fwd", k, TABLE_SIZE, bin, out[bin]);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Sinusoidal reconstruction: DFT -> keep dominant harmonics -> inverse.
// O(N * H) with precomputed twiddles; runs once, ~fast on the S3 FPU.
// ---------------------------------------------------------------------------
void AntiCogging::harmonicFilter(float* data, const CalibrationConfig& cfg) {
    const uint16_t N = TABLE_SIZE;
    const uint16_t H = MAX_HARMONICS;

    static float cosT[TABLE_SIZE], sinT[TABLE_SIZE];
    for (uint16_t i = 0; i < N; i++) {
        cosT[i] = cosf(TWO_PI_F * i / N);
        sinT[i] = sinf(TWO_PI_F * i / N);
    }

    static float re[MAX_HARMONICS + 1], im[MAX_HARMONICS + 1];
    for (uint16_t k = 0; k <= H; k++) {
        double sr = 0, si = 0;
        for (uint16_t n = 0; n < N; n++) {
            uint32_t idx = ((uint32_t)k * n) % N;
            sr += data[n] * cosT[idx];
            si -= data[n] * sinT[idx];
        }
        re[k] = (float)sr; im[k] = (float)si;
        harm_mag_[k] = sqrtf(re[k] * re[k] + im[k] * im[k]) * ((k == 0 || k == H) ? 1.0f : 2.0f) / N;
    }

    // DC = load / gravity / PID bias -> not cogging; remove it
    re[0] = im[0] = 0; harm_mag_[0] = 0;

    // find largest harmonic
    float maxMag = 0;
    for (uint16_t k = 1; k <= H; k++) maxMag = fmaxf(maxMag, harm_mag_[k]);

    // select top `harmonics_keep` above relative threshold
    static bool keep[MAX_HARMONICS + 1];
    memset(keep, 0, sizeof(keep));
    for (uint16_t c = 0; c < cfg.harmonics_keep; c++) {
        int best = -1; float bm = 0;
        for (uint16_t k = 1; k <= H; k++)
            if (!keep[k] && harm_mag_[k] > bm) { bm = harm_mag_[k]; best = k; }
        if (best < 0 || bm < cfg.harmonic_min_rel * maxMag) break;
        keep[best] = true;
    }

    // inverse DFT with only kept harmonics
    for (uint16_t n = 0; n < N; n++) {
        float v = 0;
        for (uint16_t k = 1; k <= H; k++) {
            if (!keep[k]) continue;
            uint32_t idx = ((uint32_t)k * n) % N;
            float scale = (k == H) ? 1.0f : 2.0f;
            v += scale * (re[k] * cosT[idx] - im[k] * sinT[idx]);
        }
        data[n