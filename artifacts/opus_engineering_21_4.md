I don't have context for what "Cycle 21" refers to in your project, such as a prior spec, ticket, or design iteration. Below is a complete, self-contained implementation based on standard anti-cogging practice for SimpleFOC on the ESP32-S3. If Cycle 21 has specific requirements (table format, NVS keys, harmonic count, API names), share them and I'll adapt the code.

## Design overview

**Calibration**
1. The motor runs in `angle` control with a velocity PID that includes an integral term. At standstill, that integrator settles to exactly the torque needed to cancel cogging at that position.
2. The motor steps through 512 equally spaced mechanical positions, first forward and then backward. At each bin it waits to settle, then averages the torque command (`current_sp`).
3. Averaging the forward and backward sweeps cancels Coulomb friction, which has opposite sign in each direction, and leaves the position-dependent cogging torque.
4. The DC offset is removed. The table is then fitted to a truncated Fourier series of N harmonics (the "sinusoidal" model) and reconstructed. This rejects noise and non-periodic artifacts.
5. The table is stored in NVS through `Preferences`, with a magic number, version, and CRC.

**Injection**
- An `IRAM_ATTR` lookup with linear interpolation, indexed by mechanical shaft angle.
- The value is added after `motor.move()` and before the next `motor.loopFOC()`.
- It works in `foc_current`, `dc_current`, and `voltage` torque modes. In voltage mode, the value is converted using phase resistance when it is known.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

#ifndef ANTICOG_TABLE_SIZE
#define ANTICOG_TABLE_SIZE 512
#endif

static_assert((ANTICOG_TABLE_SIZE & (ANTICOG_TABLE_SIZE - 1)) == 0,
              "ANTICOG_TABLE_SIZE must be a power of two");

class AntiCogging {
public:
  struct Config {
    uint16_t settle_ms        = 60;     // wait after each step before sampling
    uint16_t samples_per_bin  = 40;     // torque samples averaged per bin
    uint16_t sample_period_us = 250;    // spacing between samples
    uint8_t  harmonics        = 48;     // Fourier harmonics kept in the fit (0 = raw table)
    float    max_comp         = 1.0f;   // clamp on injected compensation [A or V]
    float    velocity_cutoff  = 200.0f; // rad/s above which compensation fades out
    float    velocity_fade    = 50.0f;  // fade band width [rad/s]
    float    cal_vel_P        = 0.0f;   // 0 = keep the user's existing PID gains
    float    cal_vel_I        = 0.0f;
    float    cal_angle_P      = 0.0f;
    const char* nvs_namespace = "anticog";
  };

  enum class Status : uint8_t {
    NotCalibrated = 0,
    Calibrating,
    Ready,
    Error
  };

  explicit AntiCogging(BLDCMotor& motor);

  void setConfig(const Config& cfg) { _cfg = cfg; }
  const Config& config() const { return _cfg; }

  // Blocking calibration. Motor must be initFOC()'d and enabled.
  // Typical duration: 2 * 512 * (settle + samples) ~ 1-2 min with defaults.
  bool calibrate(Print* log = &Serial);

  // Persistence (NVS via Preferences)
  bool save();
  bool load();
  bool erase();

  // Runtime: call once per control loop, after motor.move().
  void IRAM_ATTR inject();

  // Raw lookup (for diagnostics / custom loops)
  float IRAM_ATTR compensation(float mech_angle) const;

  void enable(bool en) { _enabled = en && _status == Status::Ready; }
  bool enabled() const { return _enabled; }
  void setGain(float g) { _gain = g; }  // 0..1 typical; tune live
  Status status() const { return _status; }

  const float* table() const { return _table; }
  void dumpTable(Print& out) const;

private:
  bool  settleAndSample(float target, float& out_avg);
  void  removeDC(float* t);
  void  fourierFit(float* t, uint8_t harmonics);
  uint32_t crc32(const uint8_t* data, size_t len) const;

  BLDCMotor& _motor;
  Config     _cfg;
  Status     _status  = Status::NotCalibrated;
  bool       _enabled = false;
  float      _gain    = 1.0f;

  alignas(4) float _table[ANTICOG_TABLE_SIZE];

  static constexpr uint32_t MAGIC   = 0xAC0C5121;
  static constexpr uint16_t VERSION = 1;
  static constexpr float    BIN_SCALE = ANTICOG_TABLE_SIZE / _2PI;
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>

AntiCogging::AntiCogging(BLDCMotor& motor) : _motor(motor) {
  memset(_table, 0, sizeof(_table));
}

// ---------------------------------------------------------------------------
// Runtime lookup: linear interpolation on mechanical angle, IRAM-resident.
// ---------------------------------------------------------------------------
float IRAM_ATTR AntiCogging::compensation(float mech_angle) const {
  // Normalize to [0, 2PI)
  float a = fmodf(mech_angle, _2PI);
  if (a < 0.0f) a += _2PI;

  float pos  = a * BIN_SCALE;
  int   i0   = (int)pos;
  float frac = pos - (float)i0;
  i0 &= (ANTICOG_TABLE_SIZE - 1);
  int   i1   = (i0 + 1) & (ANTICOG_TABLE_SIZE - 1);

  return _table[i0] + frac * (_table[i1] - _table[i0]);
}

void IRAM_ATTR AntiCogging::inject() {
  if (!_enabled || !_motor.enabled) return;

  // Fade compensation out at high speed. Cogging is filtered by inertia
  // there, and the table phase lag would add noise instead of removing it.
  float v = fabsf(_motor.shaft_velocity);
  float w = 1.0f;
  if (v > _cfg.velocity_cutoff) {
    w = 1.0f - (v - _cfg.velocity_cutoff) / _cfg.velocity_fade;
    if (w <= 0.0f) return;
  }

  // Use the sensor's mechanical angle (not shaft_angle, which has
  // sensor_direction / offset applied) so the table stays position-locked.
  float mech = _motor.sensor ? _motor.sensor->getMechanicalAngle()
                             : _motor.shaft_angle;
  if (_motor.sensor_direction == Direction::CCW) mech = _2PI - mech;

  float c = _gain * w * compensation(mech);
  c = _constrain(c, -_cfg.max_comp, _cfg.max_comp);

  switch (_motor.torque_controller) {
    case TorqueControlType::foc_current:
    case TorqueControlType::dc_current:
      // loopFOC() consumes current_sp directly
      _motor.current_sp += c;
      break;

    case TorqueControlType::voltage:
    default:
      // move() already converted current_sp -> voltage.q, so add here.
      // If the table was recorded in "current" units with known R, convert.
      if (_isset(_motor.phase_resistance)) {
        _motor.current_sp += c;
        _motor.voltage.q  += c * _motor.phase_resistance;
      } else {
        _motor.voltage.q  += c;  // table is already in volts
      }
      _motor.voltage.q = _constrain(_motor.voltage.q,
                                    -_motor.voltage_limit, _motor.voltage_limit);
      break;
  }
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
bool AntiCogging::settleAndSample(float target, float& out_avg) {
  _motor.target = target;

  // Run the control loop while settling
  uint32_t t0 = millis();
  while (millis() - t0 < _cfg.settle_ms) {
    _motor.loopFOC();
    _motor.move();
    yield();
  }

  // Sample the steady-state torque command. Using current_sp (the velocity
  // PID output) rather than measured current keeps the reading noise-free
  // and in the same units as injection.
  double acc = 0.0;
  for (uint16_t s = 0; s < _cfg.samples_per_bin; ++s) {
    uint32_t ts = micros();
    while (micros() - ts < _cfg.sample_period_us) {
      _motor.loopFOC();
      _motor.move();
    }
    acc += _motor.current_sp;
  }
  out_avg = (float)(acc / _cfg.samples_per_bin);

  // Sanity check: position must actually have been reached
  float err = fabsf(_motor.shaft_angle - target);
  return err < (_2PI / ANTICOG_TABLE_SIZE) * 4.0f;
}

bool AntiCogging::calibrate(Print* log) {
  if (!_motor.enabled || _motor.motor_status != FOCMotorStatus::motor_ready) {
    if (log) log->println(F("[AC] motor not ready (initFOC + enable first)"));
    _status = Status::Error;
    return false;
  }

  _status  = Status::Calibrating;
  _enabled = false;

  // Save user state
  MotionControlType prev_ctrl = _motor.controller;
  float prevVP = _motor.PID_velocity.P, prevVI = _motor.PID_velocity.I;
  float prevAP = _motor.P_angle.P;
  float prevTarget = _motor.target;

  _motor.controller = MotionControlType::angle;
  if (_cfg.cal_vel_P   > 0) _motor.PID_velocity.P = _cfg.cal_vel_P;
  if (_cfg.cal_vel_I   > 0) _motor.PID_velocity.I = _cfg.cal_vel_I;
  if (_cfg.cal_angle_P > 0) _motor.P_angle.P      = _cfg.cal_angle_P;

  if (_motor.PID_velocity.I <= 0.0f && log)
    log->println(F("[AC] WARNING: velocity I gain is 0; results will be poor"));

  static float fwd[ANTICOG_TABLE_SIZE];
  static float bwd[ANTICOG_TABLE_SIZE];

  // Map bin index -> shaft_angle target, anchored at the current mechanical
  // zero so table index == sensor mechanical angle.
  float mech_now = _motor.sensor->getMechanicalAngle();
  if (_motor.sensor_direction == Direction::CCW) mech_now = _2PI - mech_now;
  float base = _motor.shaft_angle - mech_now;  // shaft_angle at mech = 0
  const float step = _2PI / ANTICOG_TABLE_SIZE;

  // Pre-roll: approach bin 0 from below so the forward sweep is consistent
  settleAndSample(base - 8 * step, fwd[0]);

  uint16_t failures = 0;
  if (log) log->println(F("[AC] forward sweep..."));
  for (int i = 0; i < ANTICOG_TABLE_SIZE; ++i) {
    if (!settleAndSample(base + i * step, fwd[i])) failures++;
    if (log && (i % 64) == 0) log->printf("[AC]  fwd %3d/%d\n", i, ANTICOG_TABLE_SIZE);
  }

  // Overshoot, then come back for the reverse sweep
  float dummy;
  settleAndSample(base + (ANTICOG_TABLE_SIZE + 8) * step, dummy);

  if (log) log->println(F("[AC] backward sweep..."));
  for (int i = ANTICOG_TABLE_SIZE - 1; i >= 0; --i) {
    if (!settleAndSample(base + i * step, bwd[i])) failures++;
    if (log && (i % 64) == 0) log->printf("[AC]  bwd %3d/%d\n", i, ANTICOG_TABLE_SIZE);
  }

  // Restore user state
  _motor.controller      = prev_ctrl;
  _motor.PID_velocity.P  = prevVP;
  _motor.PID_velocity.I  = prevVI;
  _motor.P_angle.P       = prevAP;
  _motor.target          = (prev_ctrl == MotionControlType::angle)
                           ? _motor.shaft_angle : prevTarget;

  if (failures > ANTICOG_TABLE_SIZE / 20) {  // >5% of bins failed to settle
    if (log) log->printf("[AC] FAILED: %u bins did not settle\n", failures);
    _status = Status::Error;
    return false;
  }

  // Average directions -> cancels friction
  for (int i = 0; i < ANTICOG_TABLE_SIZE; ++i)
    _table[i] = 0.5f * (fwd[i] + bwd[i]);

  removeDC(_table);
  if (_cfg.harmonics > 0) fourierFit(_table, _cfg.harmonics);

  float pk = 0;
  for (int i = 0; i < ANTICOG_TABLE_SIZE; ++i) pk = fmaxf(pk, fabsf(_table[i]));
  if (log) log->printf("[AC] done. peak cogging comp = %.4f, %u unsettled bins\n",
                       pk, failures);

  _status = Status::Ready;
  return true;
}

void AntiCogging::removeDC(float* t) {
  double m = 0;
  for (int i = 0; i < ANTICOG_TABLE_SIZE; ++i) m += t[i];
  m /= ANTICOG_TABLE_SIZE;
  for (int i = 0; i < ANTICOG_TABLE_SIZE; ++i) t[i] -= (float)m;
}

// Truncated real DFT fit: keep harmonics 1..H and reconstruct.
// O(N*H). With N = 512 and H = 48 this is ~25k MACs, trivial at calibration time.
void AntiCog