# Anti-Cogging Calibration & Feedforward Module for SimpleFOC (ESP32-S3)

I don't know what "Cycle 23" refers to. It may be a revision or sprint in your project, and I have no context for it. The code below is a complete, self-contained implementation based on these assumptions:

- **SimpleFOC 2.3.x**, using a `BLDCMotor` with any `Sensor`. It works with `foc_current`, `dc_current` or `voltage` torque control.
- **512-bin table** over one mechanical revolution.
- **Sinusoidal (harmonic) fit**: the raw measurement is decomposed by DFT. Only significant harmonics are kept, which removes noise. The 512-point table is then rebuilt from that fit.
- **Bidirectional calibration** (forward and reverse sweeps averaged) to cancel Coulomb friction.
- **Persistence** in ESP32 NVS via `Preferences`.
- **Injection** happens after `motor.move()` by adding to `current_sp` or `voltage.q`. Interpolation is linear, with a velocity-dependent fade-out.

If your "Cycle 23" spec differs (fixed-point table, ISR injection, different harmonic count, etc.), tell me and I'll adapt it.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
  static constexpr uint16_t TABLE_SIZE    = 512;
  static constexpr uint16_t MAX_HARMONICS = 64;   // fit uses harmonics 1..MAX
  static constexpr uint32_t STORAGE_MAGIC = 0xAC0C5120;
  static constexpr uint16_t STORAGE_VER   = 1;

  struct Config {
    uint16_t settle_ms        = 60;    // wait after each position step
    uint16_t sample_count     = 40;    // samples averaged per bin
    uint16_t sample_period_us = 500;   // spacing between samples
    float    max_angle_error  = 0.05f; // rad; bin rejected if exceeded
    uint16_t harmonics        = 48;    // number of harmonics kept in fit
    float    harmonic_thresh  = 0.02f; // keep harmonic if amp > thresh * max amp
    float    fade_start_rad_s = 20.0f; // full compensation below this speed
    float    fade_end_rad_s   = 60.0f; // zero compensation above this speed
    float    gain             = 1.0f;  // runtime scaling
    float    clamp            = 0.0f;  // 0 = auto (current/voltage limit)
  };

  enum class Status : uint8_t {
    Idle, Calibrating, Ready, Failed_NotInit, Failed_Tracking, Failed_Aborted
  };

  explicit AntiCogging(BLDCMotor& motor);

  // --- calibration ---
  Status calibrate(Print* log = &Serial);   // blocking
  void   abort() { abort_req_ = true; }

  // --- runtime ---
  // Call every loop AFTER motor.move(), BEFORE the next motor.loopFOC().
  void   apply();
  float  compensationAt(float mech_angle) const;  // interpolated value
  void   enable(bool en) { enabled_ = en && (status_ == Status::Ready); }
  bool   isEnabled() const { return enabled_; }

  // --- persistence ---
  bool   save(const char* ns = "anticog");
  bool   load(const char* ns = "anticog");
  bool   erase(const char* ns = "anticog");

  // --- diagnostics ---
  void   dumpTable(Print& out) const;
  float  lastAppliedValue() const { return last_applied_; }
  Status status() const { return status_; }

  Config config;

private:
  struct SavedState {
    MotionControlType   controller;
    TorqueControlType   torque;
    float pP, pI, pD, pRamp, pLimit;
    float vP, vI, vD, vRamp, vLimit, vTf;
    float aP, aLimit;
    float target;
  };

  void   saveState(SavedState& s);
  void   restoreState(const SavedState& s);
  bool   sweep(int dir, float* out, Print* log);
  void   runFor(uint32_t ms, float target);
  float  measureBin(float target, bool& ok);
  void   harmonicFit(const float* raw, float* out);
  float  torqueSetpoint() const;
  uint32_t checksum() const;
  inline float wrap2pi(float a) const {
    a = fmodf(a, _2PI);
    return a < 0 ? a + _2PI : a;
  }

  BLDCMotor& motor_;
  float table_[TABLE_SIZE];   // compensation in A (current modes) or V (voltage mode)
  float zero_angle_ = 0.0f;   // mechanical angle at bin 0
  TorqueControlType table_mode_ = TorqueControlType::voltage;

  volatile bool abort_req_ = false;
  bool   enabled_ = false;
  Status status_  = Status::Idle;
  float  last_applied_ = 0.0f;
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& motor) : motor_(motor) {
  memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
float AntiCogging::torqueSetpoint() const {
  // The output of the position/velocity loops is the torque demand.
  // In voltage mode, move() writes voltage.q; in current modes it writes current_sp.
  return (motor_.torque_controller == TorqueControlType::voltage)
           ? motor_.voltage.q
           : motor_.current_sp;
}

void AntiCogging::saveState(SavedState& s) {
  s.controller = motor_.controller;
  s.torque     = motor_.torque_controller;
  s.pP = motor_.P_angle.P; s.pI = motor_.P_angle.I; s.pD = motor_.P_angle.D;
  s.pRamp = motor_.P_angle.output_ramp; s.pLimit = motor_.P_angle.limit;
  s.vP = motor_.PID_velocity.P; s.vI = motor_.PID_velocity.I; s.vD = motor_.PID_velocity.D;
  s.vRamp = motor_.PID_velocity.output_ramp; s.vLimit = motor_.PID_velocity.limit;
  s.vTf = motor_.LPF_velocity.Tf;
  s.aP = motor_.P_angle.P; s.aLimit = motor_.velocity_limit;
  s.target = motor_.target;
}

void AntiCogging::restoreState(const SavedState& s) {
  motor_.controller        = s.controller;
  motor_.torque_controller = s.torque;
  motor_.P_angle.P = s.pP; motor_.P_angle.I = s.pI; motor_.P_angle.D = s.pD;
  motor_.P_angle.output_ramp = s.pRamp; motor_.P_angle.limit = s.pLimit;
  motor_.PID_velocity.P = s.vP; motor_.PID_velocity.I = s.vI; motor_.PID_velocity.D = s.vD;
  motor_.PID_velocity.output_ramp = s.vRamp; motor_.PID_velocity.limit = s.vLimit;
  motor_.LPF_velocity.Tf = s.vTf;
  motor_.velocity_limit = s.aLimit;
  motor_.PID_velocity.reset();
  motor_.P_angle.reset();
  motor_.target = s.target;
}

void AntiCogging::runFor(uint32_t ms, float target) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    motor_.loopFOC();
    motor_.move(target);
  }
}

float AntiCogging::measureBin(float target, bool& ok) {
  runFor(config.settle_ms, target);

  double acc = 0.0;
  float  max_err = 0.0f;
  for (uint16_t i = 0; i < config.sample_count; ++i) {
    uint32_t t0 = micros();
    while (micros() - t0 < config.sample_period_us) {
      motor_.loopFOC();
      motor_.move(target);
    }
    acc += torqueSetpoint();
    float err = fabsf(target - motor_.shaft_angle);
    if (err > max_err) max_err = err;
  }
  ok = (max_err <= config.max_angle_error);
  return (float)(acc / config.sample_count);
}

// ---------------------------------------------------------------------------
// One full revolution sweep. dir = +1 / -1
// out[] indexed by bin (always ascending angle order).
// ---------------------------------------------------------------------------
bool AntiCogging::sweep(int dir, float* out, Print* log) {
  const float step  = _2PI / TABLE_SIZE;
  const float start = motor_.shaft_angle;   // continuous (unwrapped) angle

  // Approach the start point from the sweep direction (takes up backlash/friction).
  runFor(300, start - dir * 4 * step);
  runFor(300, start);

  uint16_t failures = 0;
  for (uint16_t k = 0; k < TABLE_SIZE; ++k) {
    if (abort_req_) return false;

    float target = start + dir * k * step;
    bool ok;
    float v = measureBin(target, ok);

    // Map to bin index from the absolute mechanical angle
    float mech = wrap2pi(motor_.sensor->getMechanicalAngle() - zero_angle_);
    uint16_t bin = (uint16_t)lroundf(mech / step) % TABLE_SIZE;
    out[bin] = v;

    if (!ok && ++failures > TABLE_SIZE / 20) {   // >5% bad bins
      if (log) log->printf("[AC] tracking error too large at bin %u\n", bin);
      return false;
    }
    if (log && (k % 64 == 0))
      log->printf("[AC] dir %+d  %3u/%u  val=%.4f\n", dir, k, TABLE_SIZE, v);
  }

  // Return to start-ish so the next sweep has room
  runFor(200, start + dir * TABLE_SIZE * step);
  return true;
}

// ---------------------------------------------------------------------------
// Sinusoidal fit: DFT -> keep significant harmonics -> reconstruct.
// The DC term is dropped (it is friction/load, not cogging).
// ---------------------------------------------------------------------------
void AntiCogging::harmonicFit(const float* raw, float* out) {
  const uint16_t H = min<uint16_t>(config.harmonics,
                                   min<uint16_t>(MAX_HARMONICS, TABLE_SIZE / 2 - 1));
  static float a[MAX_HARMONICS + 1], b[MAX_HARMONICS + 1];
  float max_amp = 0.0f;

  for (uint16_t h = 1; h <= H; ++h) {
    double sa = 0, sb = 0;
    for (uint16_t n = 0; n < TABLE_SIZE; ++n) {
      float ph = _2PI * h * n / TABLE_SIZE;
      sa += raw[n] * cosf(ph);
      sb += raw[n] * sinf(ph);
    }
    a[h] = (float)(2.0 * sa / TABLE_SIZE);
    b[h] = (float)(2.0 * sb / TABLE_SIZE);
    float amp = sqrtf(a[h] * a[h] + b[h] * b[h]);
    if (amp > max_amp) max_amp = amp;
  }

  const float thr = config.harmonic_thresh * max_amp;
  for (uint16_t h = 1; h <= H; ++h)
    if (sqrtf(a[h] * a[h] + b[h] * b[h]) < thr) a[h] = b[h] = 0.0f;

  for (uint16_t n = 0; n < TABLE_SIZE; ++n) {
    float s = 0.0f;
    for (uint16_t h = 1; h <= H; ++h) {
      if (a[h] == 0.0f && b[h] == 0.0f) continue;
      float ph = _2PI * h * n / TABLE_SIZE;
      s += a[h] * cosf(ph) + b[h] * sinf(ph);
    }
    out[n] = s;
  }
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
AntiCogging::Status AntiCogging::calibrate(Print* log) {
  if (!motor_.sensor || motor_.motor_status != FOCMotorStatus::motor_ready) {
    status_ = Status::Failed_NotInit;
    return status_;
  }

  status_    = Status::Calibrating;
  abort_req_ = false;
  enabled_   = false;

  SavedState saved;
  saveState(saved);

  // Stiff, slow position loop for quasi-static measurement
  motor_.controller        = MotionControlType::angle;
  motor_.P_angle.P         = 20.0f;
  motor_.velocity_limit    = 2.0f;
  motor_.PID_velocity.P    = max(saved.vP, 0.2f);
  motor_.PID_velocity.I    = max(saved.vI, 10.0f);   // integrator is what learns the cogging
  motor_.P