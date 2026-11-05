# Anti-Cogging Calibration and Feedforward Module for SimpleFOC on ESP32-S3

I don't have any context for "Cycle 22", such as earlier iterations, a spec, or an existing codebase. This is a self-contained implementation built on these assumptions:

- **SimpleFOC version:** v2.3.x API (`BLDCMotor`, `PIDController`, `_normalizeAngle`, `motor.shaft_angle`).
- **Control mode:** the motor runs in `MotionControlType::torque`, with either voltage or current torque mode. The module wraps the outer loops (velocity and angle) so it can add the feedforward at the torque setpoint. SimpleFOC has no native torque feedforward hook.
- **Table resolution:** 512 bins over one mechanical revolution. Cogging is periodic in mechanical angle.
- **What "sinusoidal" means here:** after the raw table is measured, it is projected onto a truncated Fourier series. Only significant harmonics are kept, which removes noise and friction artifacts. The injected signal is then a smooth sum of sinusoids, with linear interpolation between bins.
- **Persistence:** the table is stored in NVS through `Preferences`. It is checked against the sensor/zero-angle metadata so a stale table is never applied.

If your earlier cycles defined different interfaces, share them and I'll adapt this.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
  static constexpr uint16_t N_BINS        = 512;
  static constexpr uint16_t MAX_HARMONICS = 128;   // < N_BINS/2
  static constexpr uint32_t TABLE_VERSION = 0xAC220001;

  struct CalibConfig {
    float    holdP          = 8.0f;    // position-hold PID (output = torque units: V or A)
    float    holdI          = 60.0f;
    float    holdD          = 0.02f;
    float    holdLimit      = 3.0f;    // max torque cmd during calibration
    uint16_t substeps       = 8;       // micro-steps between bins (smooth motion)
    uint16_t settleMs       = 25;      // settle time per bin before sampling
    uint16_t sampleCount    = 200;     // FOC iterations averaged per bin
    uint16_t keepHarmonics  = 48;      // highest harmonic order kept
    float    harmonicThresh = 0.02f;   // drop harmonics < thresh * max amplitude
    bool     bidirectional  = true;    // average fwd/rev to cancel friction
  };

  explicit AntiCogging(BLDCMotor& motor);

  // ---- calibration ----
  // Blocking. Motor must be initialized (initFOC done) and in torque mode.
  bool calibrate(const CalibConfig& cfg = CalibConfig(), Print* log = &Serial);

  // ---- persistence ----
  bool save(const char* nvsNamespace = "acog");
  bool load(const char* nvsNamespace = "acog");
  void clear();

  // ---- runtime ----
  void  enable(bool en)              { enabled_ = en && valid_; }
  bool  isEnabled() const            { return enabled_; }
  bool  isValid()   const            { return valid_; }
  void  setGain(float g)             { gain_ = g; }
  void  setVelocityFade(float vStart, float vEnd) { fadeStart_ = vStart; fadeEnd_ = vEnd; }
  void  setLatencyCompensation(float seconds)     { latency_ = seconds; }

  // Feedforward torque at the current shaft state.
  float IRAM_ATTR feedforward(float mechAngle, float velocity) const;

  // Torque mode: call every loop instead of motor.move(target).
  void moveTorque(float torqueCmd);
  // Velocity mode emulation (uses motor.PID_velocity + LPF_velocity).
  void moveVelocity(float velTarget);
  // Angle mode emulation (uses motor.P_angle cascaded to velocity).
  void moveAngle(float angleTarget);

  // Diagnostics
  void  dumpTable(Print& out) const;
  float tableValue(uint16_t i) const { return table_[i % N_BINS]; }
  float peakToPeak() const;

private:
  BLDCMotor& m_;
  float table_[N_BINS];
  bool  valid_      = false;
  bool  enabled_    = false;
  float gain_       = 1.0f;
  float fadeStart_  = 20.0f;   // rad/s: start reducing FF
  float fadeEnd_    = 60.0f;   // rad/s: FF fully off
  float latency_    = 0.0f;    // s: angle lead = vel * latency
  float zeroElecRef_ = NOT_SET;
  int   dirRef_      = 0;

  static float wrap2Pi(float a) { return _normalizeAngle(a); }
  bool  sweep(const CalibConfig& cfg, int dir, float startAngle,
              float* out, Print* log);
  void  fourierFilter(uint16_t keepHarm, float thresh, Print* log);
  void  runFOCFor(uint32_t ms, PIDController& pid, float target);
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& motor) : m_(motor) {
  clear();
}

void AntiCogging::clear() {
  memset(table_, 0, sizeof(table_));
  valid_ = false;
  enabled_ = false;
}

// ============================================================================
//  Calibration
// ============================================================================

void AntiCogging::runFOCFor(uint32_t ms, PIDController& pid, float target) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    m_.loopFOC();
    m_.move(pid(target - m_.shaft_angle));
  }
}

bool AntiCogging::sweep(const CalibConfig& cfg, int dir, float startAngle,
                        float* out, Print* log) {
  PIDController pid(cfg.holdP, cfg.holdI, cfg.holdD, 0, cfg.holdLimit);
  const float binW = _2PI / N_BINS;

  // Lock onto start position.
  runFOCFor(300, pid, startAngle);

  // Visit N_BINS + 1 points so the bin containing the start is
  // re-measured after the lock-in transient has decayed.
  for (uint16_t k = 0; k <= N_BINS; ++k) {
    float base = startAngle + dir * k * binW;

    // Micro-step into the bin to avoid exciting the PID.
    for (uint16_t s = 1; s <= cfg.substeps; ++s) {
      float tgt = base - dir * binW + dir * binW * s / cfg.substeps;
      uint32_t t0 = micros();
      while (micros() - t0 < 1000) {
        m_.loopFOC();
        m_.move(pid(tgt - m_.shaft_angle));
      }
    }
    runFOCFor(cfg.settleMs, pid, base);

    // Average the holding torque (PID output) over the window.
    double acc = 0.0, angAcc = 0.0;
    for (uint16_t n = 0; n < cfg.sampleCount; ++n) {
      m_.loopFOC();
      float u = pid(base - m_.shaft_angle);
      m_.move(u);
      acc    += u;
      angAcc += m_.shaft_angle;
    }
    float holdTorque = acc / cfg.sampleCount;
    float meanAng    = angAcc / cfg.sampleCount;

    if (fabsf(holdTorque) >= 0.98f * cfg.holdLimit) {
      if (log) log->printf("[ACOG] saturated at bin %u (%.3f)\n", k, holdTorque);
      m_.move(0);
      return false;
    }

    // Bin from the *measured* angle, not the setpoint.
    uint16_t idx = (uint16_t)(wrap2Pi(meanAng) / binW) % N_BINS;
    // Holding torque opposes cogging torque, so FF = holding torque.
    out[idx] = holdTorque;

    if (log && (k % 64 == 0))
      log->printf("[ACOG] dir %+d  %3u/%u  T=%.4f\n", dir, k, N_BINS, holdTorque);
    yield();  // keep the task watchdog happy
  }
  return true;
}

bool AntiCogging::calibrate(const CalibConfig& cfg, Print* log) {
  if (m_.controller != MotionControlType::torque) {
    if (log) log->println("[ACOG] motor must be in torque control mode");
    return false;
  }
  if (!m_.sensor || m_.motor_status != FOCMotorStatus::motor_ready) {
    if (log) log->println("[ACOG] motor not initialized (initFOC)");
    return false;
  }

  bool wasEnabled = enabled_;
  enabled_ = false;

  static float fwd[N_BINS];
  static float rev[N_BINS];
  // NaN marks bins that never got a sample.
  for (uint16_t i = 0; i < N_BINS; ++i) fwd[i] = rev[i] = NAN;

  m_.loopFOC(); m_.move(0);
  float start = m_.shaft_angle;

  if (log) log->println("[ACOG] forward sweep...");
  if (!sweep(cfg, +1, start, fwd, log)) { enabled_ = wasEnabled; return false; }

  if (cfg.bidirectional) {
    if (log) log->println("[ACOG] reverse sweep...");
    m_.loopFOC();
    if (!sweep(cfg, -1, m_.shaft_angle, rev, log)) { enabled_ = wasEnabled; return false; }
  }
  m_.move(0);

  // Merge and fill gaps.
  for (uint16_t i = 0; i < N_BINS; ++i) {
    bool f = !isnan(fwd[i]), r = !isnan(rev[i]);
    table_[i] = (f && r) ? 0.5f * (fwd[i] + rev[i]) : f ? fwd[i] : r ? rev[i] : NAN;
  }
  uint16_t holes = 0;
  for (uint16_t i = 0; i < N_BINS; ++i) {
    if (!isnan(table_[i])) continue;
    ++holes;
    int a = i, b = i;
    for (uint16_t s = 1; s < N_BINS && isnan(table_[(i - s + N_BINS) % N_BINS]); ++s) a = i - s - 1;
    for (uint16_t s = 1; s < N_BINS && isnan(table_[(i + s) % N_BINS]); ++s) b = i + s + 1;
    float va = table_[(a % N_BINS + N_BINS) % N_BINS];
    float vb = table_[b % N_BINS];
    if (isnan(va)) va = 0; if (isnan(vb)) vb = 0;
    float t = (float)(i - a) / (float)(b - a);
    table_[i] = va + (vb - va) * t;
  }
  if (log && holes) log->printf("[ACOG] interpolated %u empty bins\n", holes);

  // Remove the DC term (friction/load bias). It is not cogging.
  double mean = 0; for (float v : table_) mean += v; mean /= N_BINS;
  for (float& v : table_) v -= mean;

  fourierFilter(cfg.keepHarmonics, cfg.harmonicThresh, log);

  zeroElecRef_ = m_.zero_electric_angle;
  dirRef_      = (int)m_.sensor_direction;
  valid_       = true;
  enabled_     = wasEnabled;

  if (log) log->printf("[ACOG] done. p-p cogging = %.4f\n", peakToPeak());
  return true;
}

// Project the table onto sin/cos harmonics 1..keepHarm and drop weak ones.
void AntiCogging::fourierFilter(uint16_t keepHarm, float thresh, Print* log) {
  if (keepHarm > MAX_HARMONICS) keepHarm = MAX_HARMONICS;
  static float A[MAX_HARMONICS + 1], B[MAX_HARMONICS + 1], Mag[MAX_HARMONICS + 1];

  float maxMag = 0;
  for (uint16_t h = 1; h <= keepHarm; ++h) {
    double a = 0, b = 0;
    for (uint16_t i = 0; i < N_BINS; ++i) {
      float ph = _2PI * h * i / N_BINS;
      a += table_[i] * cosf(ph);
      b += table_[i] * sinf(ph);
    }
    A[h] = 2.0 * a / N_BINS;
    B[h] = 2.0 * b / N_BINS;
    Mag[h] = sqrtf(A[h] * A[h] + B[h] * B[h]);
    if (Mag[h] > maxMag) maxMag = Mag[h];
  }

  uint16_t kept = 0;
  for (uint16_t i = 0; i < N_BINS; ++i) table_[i] = 0;
  for (uint16_t h = 1; h <= keepHarm; ++h