# Anti-Cogging Calibration and Feedforward Module for SimpleFOC on ESP32-S3

I don't have the earlier "cycles" of this project, so I've made the following assumptions. Tell me what differs and I'll adjust.

- **Library:** SimpleFOC 2.3.x with `BLDCMotor`. Torque modes supported are `voltage`, `dc_current` and `foc_current`.
- **Table:** 512 bins over one mechanical revolution.
- **"Sinusoidal":** the raw measured table is fitted with a truncated Fourier series. The table is then rebuilt from the selected harmonics, which gives a smooth, noise-free, zero-mean compensation curve.
- **Calibration:** a quasi-static position sweep in `angle` mode.
  - The averaged PID torque command needed to hold each bin is recorded in both directions.
  - The two directions are averaged, which cancels Coulomb friction.
- **Injection:** feedforward is added after `motor.move()`, so the next `loopFOC()` uses it.
  - In voltage mode it is added to `motor.voltage.q`.
  - In current modes it is added to `motor.current_sp`.
- **Storage:** results are persisted in ESP32 NVS (`Preferences`) with a magic number, version and CRC.

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
    static constexpr uint16_t MAX_HARMONICS  = 128;     // must be <= TABLE_SIZE/2
    static constexpr uint32_t STORAGE_MAGIC  = 0xAC0C5120;
    static constexpr uint16_t STORAGE_VER    = 1;

    struct Config {
        uint16_t settle_ms        = 25;     // dwell after each step before sampling
        uint16_t samples_per_bin  = 64;     // averaged current_sp samples per bin
        uint32_t loop_period_us   = 100;    // FOC loop period during calibration (~10 kHz)
        uint8_t  passes           = 1;      // fwd+rev pass pairs
        uint16_t num_harmonics    = 48;     // harmonics kept in sinusoidal fit (1..MAX_HARMONICS)
        float    min_harm_amp     = 0.0f;   // drop harmonics below this amplitude (units of torque cmd)
        float    gain             = 1.0f;   // runtime injection gain (0..~1.2)
        float    fade_start_rad_s = 30.0f;  // start fading compensation above this speed
        float    fade_end_rad_s   = 80.0f;  // fully disabled above this speed (<=start disables fading)
        bool     remove_dc        = true;   // remove mean (gravity/friction offset)
    };

    enum class Status : uint8_t {
        OK = 0,
        NOT_INITIALIZED,
        NO_SENSOR,
        MOTOR_DISABLED,
        ABORTED,
        BIN_COVERAGE_FAIL,
        STORAGE_FAIL,
        STORAGE_INVALID
    };

    explicit AntiCogging(BLDCMotor& motor);

    void   init(const Config& cfg);
    Config& config() { return _cfg; }

    // Blocking calibration. Motor must be initFOC()'ed and enabled.
    // Optional progress callback: (percent 0..100).
    Status calibrate(void (*progress_cb)(uint8_t) = nullptr);
    void   requestAbort() { _abort = true; }

    // Call right after motor.move() in the main loop.
    void IRAM_ATTR apply();

    // Pure lookup (linear interpolation), mechanical angle in rad [0, 2pi).
    float IRAM_ATTR lookup(float mech_angle) const;

    void enable(bool en)  { _enabled = en && _valid; }
    bool isEnabled() const { return _enabled; }
    bool isValid()   const { return _valid; }

    Status save(const char* ns = "anticog");
    Status load(const char* ns = "anticog");
    void   clear();

    // Diagnostics
    const float* table()    const { return _table; }
    const float* rawTable() const { return _raw; }
    float harmonicAmplitude(uint16_t k) const;   // valid after calibrate()
    void  printReport(Print& out) const;
    float lastInjection() const { return _last_ff; }

private:
    BLDCMotor& _motor;
    Config     _cfg;

    float _table[TABLE_SIZE];   // fitted, injected table
    float _raw[TABLE_SIZE];     // raw measured (fwd+rev averaged)
    float _harm_a[MAX_HARMONICS + 1];
    float _harm_b[MAX_HARMONICS + 1];

    TorqueControlType _calib_torque_type = TorqueControlType::voltage;

    volatile bool _abort   = false;
    bool          _enabled = false;
    bool          _valid   = false;
    bool          _inited  = false;
    float         _last_ff = 0.0f;

    // helpers
    static inline float wrap2pi(float a) {
        a = fmodf(a, _2PI);
        return (a < 0.0f) ? a + _2PI : a;
    }
    inline uint16_t angleToBin(float mech) const {
        int b = (int)(wrap2pi(mech) * (TABLE_SIZE / _2PI) + 0.5f);
        return (uint16_t)(b % TABLE_SIZE);
    }
    void   runLoopFor(uint32_t ms);
    bool   sampleBin(float& avg_cmd, float& avg_mech);
    void   fitSinusoidal();
    static uint32_t crc32(const uint8_t* d, size_t n, uint32_t crc = 0xFFFFFFFF);
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& motor) : _motor(motor) {
    clear();
}

void AntiCogging::init(const Config& cfg) {
    _cfg = cfg;
    if (_cfg.num_harmonics < 1)             _cfg.num_harmonics = 1;
    if (_cfg.num_harmonics > MAX_HARMONICS) _cfg.num_harmonics = MAX_HARMONICS;
    if (_cfg.samples_per_bin < 1)           _cfg.samples_per_bin = 1;
    if (_cfg.passes < 1)                    _cfg.passes = 1;
    _inited = true;
}

void AntiCogging::clear() {
    memset(_table, 0, sizeof(_table));
    memset(_raw, 0, sizeof(_raw));
    memset(_harm_a, 0, sizeof(_harm_a));
    memset(_harm_b, 0, sizeof(_harm_b));
    _valid = false;
    _enabled = false;
    _last_ff = 0.0f;
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------
float IRAM_ATTR AntiCogging::lookup(float mech_angle) const {
    float pos = wrap2pi(mech_angle) * (TABLE_SIZE / _2PI);
    uint16_t i0 = (uint16_t)pos;
    float frac  = pos - (float)i0;
    i0 &= (TABLE_SIZE - 1);                         // TABLE_SIZE is power of 2
    uint16_t i1 = (i0 + 1) & (TABLE_SIZE - 1);
    return _table[i0] + frac * (_table[i1] - _table[i0]);
}

void IRAM_ATTR AntiCogging::apply() {
    if (!_enabled || !_motor.enabled || !_motor.sensor) { _last_ff = 0.0f; return; }

    // Speed-dependent fade: at high speed cogging is filtered by inertia,
    // and table phase lag would make compensation counter-productive.
    float w = fabsf(_motor.shaft_velocity);
    float k = _cfg.gain;
    if (_cfg.fade_end_rad_s > _cfg.fade_start_rad_s) {
        if (w >= _cfg.fade_end_rad_s) {
            k = 0.0f;
        } else if (w > _cfg.fade_start_rad_s) {
            k *= 1.0f - (w - _cfg.fade_start_rad_s) /
                        (_cfg.fade_end_rad_s - _cfg.fade_start_rad_s);
        }
    }
    if (k == 0.0f) { _last_ff = 0.0f; return; }

    // Table is indexed by raw sensor mechanical angle (independent of
    // sensor_direction / zero offset), so it stays valid across re-alignments.
    float ff = k * lookup(_motor.sensor->getMechanicalAngle());
    _last_ff = ff;

    switch (_motor.torque_controller) {
        case TorqueControlType::voltage:
            // move() already computed voltage.q; add and clamp
            _motor.voltage.q = _constrain(_motor.voltage.q + ff,
                                          -_motor.voltage_limit, _motor.voltage_limit);
            break;
        case TorqueControlType::dc_current:
        case TorqueControlType::foc_current:
            _motor.current_sp = _constrain(_motor.current_sp + ff,
                                           -_motor.current_limit, _motor.current_limit);
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
void AntiCogging::runLoopFor(uint32_t ms) {
    uint32_t t_end = millis() + ms;
    uint32_t t_next = micros();
    uint32_t t_yield = millis();
    while ((int32_t)(millis() - t_end) < 0 && !_abort) {
        if ((int32_t)(micros() - t_next) >= 0) {
            t_next += _cfg.loop_period_us;
            _motor.loopFOC();
            _motor.move();
        }
        if (millis() - t_yield >= 50) {   // keep task watchdog happy
            t_yield = millis();
            vTaskDelay(1);
        }
    }
}

bool AntiCogging::sampleBin(float& avg_cmd, float& avg_mech) {
    double sum_cmd = 0.0;
    float  sin_acc = 0.0f, cos_acc = 0.0f;   // circular mean of angle
    uint16_t n = 0;
    uint32_t t_next = micros();

    while (n < _cfg.samples_per_bin && !_abort) {
        if ((int32_t)(micros() - t_next) >= 0) {
            t_next += _cfg.loop_period_us;
            _motor.loopFOC();
            _motor.move();

            // The holding command = PID output of the outer loops
            float cmd = (_motor.torque_controller == TorqueControlType::voltage)
                        ? _motor.voltage.q : _motor.current_sp;
            sum_cmd += cmd;

            float m = _motor.sensor->getMechanicalAngle();
            sin_acc += sinf(m);
            cos_acc += cosf(m);
            n++;
        }
    }
    if (n == 0) return false;
    avg_cmd  = (float)(sum_cmd / n);
    avg_mech = wrap2pi(atan2f(sin_acc, cos_acc));
    return true;
}

AntiCogging::Status AntiCogging::calibrate(void (*progress_cb)(uint8_t)) {
    if (!_inited)          return Status::NOT_INITIALIZED;
    if (!_motor.sensor)    return Status::NO_SENSOR;
    if (!_motor.enabled)   return Status::MOTOR_DISABLED;

    _abort = false;
    bool was_enabled = _enabled;
    _enabled = false;                         // never inject while measuring

    // Save controller state
    MotionControlType prev_ctrl = _motor.controller;
    float prev_target = _motor.target;
    _calib_torque_type = _motor.torque_controller;

    // Accumulators per bin, separate per direction
    static float fwd_sum[TABLE_SIZE], rev_sum[TABLE_SIZE];
    static uint16_t fwd_cnt[TABLE_SIZE], rev_cnt[TABLE_