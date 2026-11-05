# Anti-Cogging Calibration & Feedforward Module for SimpleFOC (ESP32-S3)

I don't know what "Cycle 31" refers to. It may be an internal project iteration or a spec I haven't seen. This implementation is built from first principles. If Cycle 31 has specific requirements (table format, NVS keys, units, interfaces), tell me and I'll adapt it.

## Design Overview

| Aspect | Approach |
|---|---|
| **Table** | 512 points over one mechanical revolution, stored as torque command (V or A, matching your torque controller) |
| **Calibration** | Closed-loop angle hold at each point. The average q-axis command needed to hold position is the cogging torque. |
| **Friction rejection** | Forward and reverse sweeps are averaged, which cancels Coulomb friction and hysteresis |
| **"Sinusoidal" smoothing** | DFT of the raw table. Only the DC-free low/cogging harmonics are kept, so the table is a sum of sinusoids, which removes noise. |
| **Runtime** | Fast linear interpolation lookup, gain scaling, and velocity-dependent fade-out |
| **Persistence** | ESP32 NVS (`Preferences`) with CRC32 validation |
| **Hot path** | `IRAM_ATTR` lookup, no allocation, no trig |

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
    static constexpr uint16_t MAX_HARMONICS = 64;   // highest harmonic retained in fit

    struct Config {
        float    settleTimeMs        = 60.0f;   // wait after each step before sampling
        uint16_t samplesPerPoint     = 200;     // FOC-loop samples averaged per point
        float    maxPositionErrorRad = 0.05f;   // abort if tracking error exceeds this
        float    maxCommandAbs       = 0.0f;    // 0 = use motor limit; abort if exceeded
        uint16_t harmonicsToKeep     = 48;      // 1..MAX_HARMONICS; 0 = no smoothing
        bool     removeDC            = true;    // strip mean (gravity/bias), keep ripple only
        float    fadeStartRadS       = 20.0f;   // feedforward starts fading above this speed
        float    fadeEndRadS         = 60.0f;   // feedforward fully off above this speed
        float    gain                = 1.0f;    // runtime compensation scale (0..~1.2)
        const char* nvsNamespace     = "acog";
    };

    enum class Status : uint8_t {
        NotCalibrated, Calibrating, Ready, ErrorTracking,
        ErrorSaturation, ErrorNoSensor, ErrorStorage
    };

    AntiCogging(BLDCMotor& motor, Config cfg = Config());

    // ---- Calibration (blocking; motor must be initFOC()'d and free to rotate) ----
    Status calibrate(Print* log = &Serial);

    // ---- Runtime ----
    float IRAM_ATTR lookup(float mechAngle) const;  // raw table value at angle
    float IRAM_ATTR compensation() const;           // scaled + speed-faded value at current angle
    float IRAM_ATTR compensate(float torqueTarget) const { return torqueTarget + compensation(); }
    void  move(float torqueTarget);                 // wraps motor.move() in torque mode

    void  enable(bool en)        { _enabled = en; }
    bool  enabled() const        { return _enabled; }
    void  setGain(float g)       { _cfg.gain = g; }
    Status status() const        { return _status; }
    const float* table() const   { return _table; }

    // ---- Persistence ----
    bool save();
    bool load();
    bool erase();

    // ---- Diagnostics ----
    void  dumpTable(Print& out) const;   // CSV: index,angle,value
    float peakToPeak() const;

private:
    void  runFOCFor(uint32_t ms);
    float holdAndMeasure(float targetShaftAngle, bool& ok);
    float readTorqueCommand() const;
    void  harmonicSmooth();
    uint32_t crc32(const uint8_t* data, size_t len) const;
    static float wrap2PI(float a);

    BLDCMotor& _motor;
    Config     _cfg;
    Status     _status  = Status::NotCalibrated;
    bool       _enabled = false;

    float _table[TABLE_SIZE];   // compensation torque per mechanical position
    float _fwd[TABLE_SIZE];     // scratch: forward sweep
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>

static constexpr float TWO_PI_F  = 6.28318530718f;
static constexpr float IDX_SCALE = AntiCogging::TABLE_SIZE / TWO_PI_F;

AntiCogging::AntiCogging(BLDCMotor& motor, Config cfg) : _motor(motor), _cfg(cfg) {
    memset(_table, 0, sizeof(_table));
    memset(_fwd,   0, sizeof(_fwd));
}

float AntiCogging::wrap2PI(float a) {
    a = fmodf(a, TWO_PI_F);
    return (a < 0.0f) ? a + TWO_PI_F : a;
}

// ------------------------------------------------------------------
// Torque command actually applied (what the controller "needed")
// ------------------------------------------------------------------
float AntiCogging::readTorqueCommand() const {
    switch (_motor.torque_controller) {
        case TorqueControlType::foc_current:
        case TorqueControlType::dc_current:
            return _motor.current_sp;     // amps
        case TorqueControlType::voltage:
        default:
            return _motor.voltage.q;      // volts
    }
}

void AntiCogging::runFOCFor(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        _motor.loopFOC();
        _motor.move();
    }
}

// ------------------------------------------------------------------
// Hold an angle, wait to settle, average the required torque command
// ------------------------------------------------------------------
float AntiCogging::holdAndMeasure(float target, bool& ok) {
    _motor.target = target;
    runFOCFor((uint32_t)_cfg.settleTimeMs);

    const float limit = (_cfg.maxCommandAbs > 0.0f) ? _cfg.maxCommandAbs
        : ((_motor.torque_controller == TorqueControlType::voltage)
               ? _motor.voltage_limit : _motor.current_limit);

    double acc = 0.0;
    for (uint16_t i = 0; i < _cfg.samplesPerPoint; i++) {
        _motor.loopFOC();
        _motor.move();
        float cmd = readTorqueCommand();
        if (fabsf(_motor.shaft_angle - target) > _cfg.maxPositionErrorRad) {
            _status = Status::ErrorTracking; ok = false; return 0.0f;
        }
        if (fabsf(cmd) >= 0.98f * limit) {
            _status = Status::ErrorSaturation; ok = false; return 0.0f;
        }
        acc += cmd;
    }
    ok = true;
    return (float)(acc / _cfg.samplesPerPoint);
}

// ------------------------------------------------------------------
// Calibration
// ------------------------------------------------------------------
AntiCogging::Status AntiCogging::calibrate(Print* log) {
    if (!_motor.sensor) { _status = Status::ErrorNoSensor; return _status; }

    _status  = Status::Calibrating;
    _enabled = false;

    // Save controller state to restore later
    MotionControlType prevCtrl   = _motor.controller;
    float             prevTarget = _motor.target;

    _motor.controller = MotionControlType::angle;

    // Anchor sweep to absolute mechanical zero: table index 0 == mech angle 0
    _motor.loopFOC(); _motor.move();
    const float mech0 = _motor.sensor->getMechanicalAngle();
    const float base  = _motor.shaft_angle - mech0;    // shaft_angle at mech angle 0
    const float sign  = (_motor.sensor_direction == Direction::CCW) ? -1.0f : 1.0f;
    const float step  = TWO_PI_F / TABLE_SIZE;

    // Move gently to start position
    _motor.target = _motor.shaft_angle;
    for (int i = 0; i <= 50; i++) {
        _motor.target = _motor.shaft_angle + (base - _motor.shaft_angle) * (i / 50.0f);
        runFOCFor(10);
    }

    bool ok = true;
    if (log) log->println(F("[ACOG] forward sweep..."));

    // ---- Forward sweep (one extra pre-roll step to establish direction) ----
    holdAndMeasure(base - step * sign, ok);
    for (uint16_t i = 0; i < TABLE_SIZE && ok; i++) {
        _fwd[i] = holdAndMeasure(base + sign * step * i, ok);
        if (log && (i % 64) == 0) log->printf("  %3u/%u  %.4f\n", i, TABLE_SIZE, _fwd[i]);
    }

    // ---- Reverse sweep ----
    if (ok && log) log->println(F("[ACOG] reverse sweep..."));
    if (ok) holdAndMeasure(base + sign * step * TABLE_SIZE, ok);   // overshoot one step
    for (int i = TABLE_SIZE - 1; i >= 0 && ok; i--) {
        float rev = holdAndMeasure(base + sign * step * i, ok);
        // Average cancels Coulomb friction (opposite sign per direction)
        _table[i] = 0.5f * (_fwd[i] + rev);
        if (log && (i % 64) == 0) log->printf("  %3d/%u  %.4f\n", i, TABLE_SIZE, _table[i]);
    }

    // Restore control mode, hold current position
    _motor.target     = (prevCtrl == MotionControlType::angle) ? _motor.shaft_angle : prevTarget;
    _motor.controller = prevCtrl;

    if (!ok) {
        memset(_table, 0, sizeof(_table));
        if (log) log->printf("[ACOG] FAILED (status %u)\n", (unsigned)_status);
        return _status;
    }

    // Measured value is the torque the controller had to apply to *cancel* cogging,
    // which is exactly the feedforward to inject. Sign in sensor direction:
    if (sign < 0) for (auto& v : _table) v = -v;

    harmonicSmooth();

    _status = Status::Ready;
    if (log) log->printf("[ACOG] done. ripple p-p = %.4f\n", peakToPeak());
    return _status;
}

// ------------------------------------------------------------------
// Sinusoidal (Fourier) smoothing: keep harmonics 1..N, optionally DC
// ------------------------------------------------------------------
void AntiCogging::harmonicSmooth() {
    const uint16_t N = min<uint16_t>(_cfg.harmonicsToKeep, MAX_HARMONICS);
    if (N == 0) {
        if (_cfg.removeDC) {
            float m = 0; for (auto v : _table) m += v; m /= TABLE_SIZE;
            for (auto& v : _table) v -= m;
        }
        return;
    }

    static float a[MAX_HARMONICS + 1], b[MAX_HARMONICS + 1];
    const float w = TWO_PI_F / TABLE_SIZE;

    for (uint16_t k = 0; k <= N; k++) {
        double sa = 0, sb = 0;
        for (uint16_t n = 0; n < TABLE_SIZE; n++) {
            float ph = w * k * n;
            sa += _table[n] * cosf(ph);
            sb += _table[n] * sinf(ph);
        }
        a[k] = (float)(sa * 2.0 / TABLE_SIZE);
        b[k] = (float)(sb * 2.0 / TABLE_SIZE);
    }
    a[0] *= 0.5f;   // DC term

    for (uint16_t n = 0; n < TABLE_SIZE; n++) {
        float v = _cfg.removeDC ? 0.0f : a[0];
        for (uint16_t k = 1; k <= N; k++) {
            float ph = w * k * n;
            v += a[k] * cosf(ph) + b[k] * sinf(ph);
        }
        _fwd[n] = v;   // reuse scratch
    }
    memcpy(_table, _fwd, sizeof(_table));
}

// ------------------------------------------------------------------
// Runtime (hot path)
// ------------------------------------------------------------------
float IRAM_ATTR AntiCogging::lookup(float mechAngle) const {
    float x = wrap2PI(mechAngle) * IDX_SCALE;
    uint16_t i0 = (uint16_t)x;
    float frac  = x - i0;
    i0 &= (TABLE_SIZE - 1);
    uint16_t i1 = (i0 + 1) & (TABLE_SIZE - 1);
    return _table[i0] + frac * (_table[i1] - _table[i0]);
}

float IRAM_ATTR AntiCogging::compensation() const {
    if (!_enabled || _status != Status::Ready || !_motor.sensor) return 0.0f;

    float spd  = fabsf(_motor.shaft_velocity);
    float fade = 1.0f;
    if (spd >= _cfg.fadeEndRadS) return 0.0f;
    if (spd > _cfg.fadeStartRadS)
        fade = 1.0f - (spd - _cfg.fadeStartRadS) / (_cfg.fadeEndRadS - _cfg.fadeStartRadS);

    float ang = _motor.sensor->getMechanicalAngle();   // cached by loopFOC, 0..2π
    if (_motor.sensor_direction == Direction::CCW) ang = TWO_PI_F - ang;
    return _cfg.gain * fade * lookup(ang);
}

void AntiCogging::move(float torqueTarget) {
    // Valid in MotionControlType::torque. For velocity/ang