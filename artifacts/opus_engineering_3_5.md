# Anti-Cogging Calibration & Feedforward Module for SimpleFOC (ESP32-S3)

I don't know the specifics of your "Cycle 3" spec, so this implementation makes the following assumptions. Adjust any of them if your spec differs.

- **512 bins** spanning one *mechanical* revolution, indexed by the raw sensor mechanical angle.
- **Calibration** holds the rotor at each bin using SimpleFOC's own angle loop, which needs a nonzero velocity-loop I term. It records the steady-state torque command (`current_sp`). Forward and reverse sweeps are averaged so Coulomb friction cancels.
- **"Sinusoidal"** means the raw 512-point table is projected onto a configurable set of Fourier harmonics (DFT). The table is then rebuilt from those harmonics, which rejects noise and non-periodic content.
- **Runtime injection** linearly interpolates the table and adds the result to `current_sp` (current modes) or `voltage.q` (voltage mode) after `motor.move()`.
- **Persistence** uses ESP32 NVS (`Preferences`), with a version, CRC and sensor-identity check.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
  static constexpr uint16_t N_POINTS      = 512;
  static constexpr uint16_t MAX_HARMONICS = 64;   // harmonics kept for the fit
  static constexpr uint32_t STORE_VERSION = 3;    // "Cycle 3" layout

  struct Config {
    float    settle_time_ms       = 40.0f;  // dwell before sampling each bin
    uint16_t samples_per_point    = 64;     // averaged torque samples per bin
    uint16_t sample_interval_us   = 100;    // spacing between samples
    uint16_t num_harmonics        = 24;     // fitted harmonics (<= MAX_HARMONICS)
    bool     keep_dc              = false;  // DC term is usually friction/bias -> drop
    float    max_comp             = 1.0f;   // clamp on injected value [A or V]
    float    gain                 = 1.0f;   // runtime scaling of compensation
    float    cal_velocity_limit   = 2.0f;   // rad/s during calibration moves
    float    max_abs_sample       = 0.0f;   // 0 = disabled; abort if |torque| exceeds
    bool     verbose              = true;
  };

  enum class Status : uint8_t { OK = 0, NOT_INIT, ABORTED, SATURATED, NVS_ERROR, BAD_DATA };

  explicit AntiCogging(BLDCMotor& motor, Print* log = &Serial);

  Config& config() { return cfg_; }

  // Blocking calibration. Motor must be initialised, sensor aligned, unloaded.
  Status calibrate();

  // Call every loop AFTER motor.move(). Adds compensation torque.
  void IRAM_ATTR apply();

  // Compensation value at the current rotor position (no side effects).
  float IRAM_ATTR compensationAt(float mech_angle) const;

  void enable(bool en)       { enabled_ = en && valid_; }
  bool isEnabled()   const   { return enabled_; }
  bool isValid()     const   { return valid_; }
  void setGain(float g)      { cfg_.gain = g; }

  Status save(const char* ns = "acog");
  Status load(const char* ns = "acog");
  void   clear();

  void dumpTable(Print& out) const;      // CSV: index,raw_fwd,raw_rev,fitted
  void dumpHarmonics(Print& out) const;

  const float* table() const { return table_; }

private:
  struct __attribute__((packed)) Blob {
    uint32_t version;
    uint32_t n_points;
    float    sensor_offset;       // identity check: zero_electric_angle
    int8_t   sensor_direction;
    uint16_t num_harmonics;
    float    a[MAX_HARMONICS + 1];
    float    b[MAX_HARMONICS + 1];
    float    table[N_POINTS];
    uint32_t crc;
  };

  BLDCMotor& m_;
  Print*     log_;
  Config     cfg_;

  float table_[N_POINTS];
  float raw_fwd_[N_POINTS];
  float raw_rev_[N_POINTS];
  float a_[MAX_HARMONICS + 1];
  float b_[MAX_HARMONICS + 1];

  volatile bool enabled_ = false;
  bool          valid_   = false;

  static constexpr float BIN_WIDTH = _2PI / N_POINTS;

  bool  moveAndHold(float target_shaft_angle, uint32_t hold_ms);
  bool  sampleTorque(float& out_mean);
  void  serviceFOC();
  float readTorqueCommand() const;
  void  fitHarmonics(const float* raw);
  void  rebuildFromHarmonics();
  static uint32_t crc32(const uint8_t* data, size_t len);
  void  logf(const char* fmt, ...) const;
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <stdarg.h>
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& motor, Print* log) : m_(motor), log_(log) {
  clear();
}

void AntiCogging::clear() {
  memset(table_,   0, sizeof(table_));
  memset(raw_fwd_, 0, sizeof(raw_fwd_));
  memset(raw_rev_, 0, sizeof(raw_rev_));
  memset(a_, 0, sizeof(a_));
  memset(b_, 0, sizeof(b_));
  valid_ = false;
  enabled_ = false;
}

void AntiCogging::logf(const char* fmt, ...) const {
  if (!log_ || !cfg_.verbose) return;
  char buf[160];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  log_->print(buf);
}

// ---------------------------------------------------------------------------
// Low-level helpers
// ---------------------------------------------------------------------------
void AntiCogging::serviceFOC() {
  m_.loopFOC();
  m_.move();
}

// Torque command produced by the position/velocity loop = what the controller
// needs to hold the rotor still = cogging + friction (+ any external load).
float AntiCogging::readTorqueCommand() const {
  return m_.current_sp;
}

bool AntiCogging::moveAndHold(float target, uint32_t hold_ms) {
  m_.target = target;
  uint32_t t0 = millis();
  while (millis() - t0 < hold_ms) {
    serviceFOC();
    if (log_ && Serial.available() && Serial.peek() == 'x') { // user abort
      Serial.read();
      return false;
    }
  }
  return true;
}

bool AntiCogging::sampleTorque(float& out_mean) {
  double acc = 0.0;
  for (uint16_t i = 0; i < cfg_.samples_per_point; ++i) {
    uint32_t t0 = micros();
    serviceFOC();
    float v = readTorqueCommand();
    if (cfg_.max_abs_sample > 0.0f && fabsf(v) > cfg_.max_abs_sample) return false;
    acc += v;
    while (micros() - t0 < cfg_.sample_interval_us) serviceFOC();
  }
  out_mean = (float)(acc / cfg_.samples_per_point);
  return true;
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
AntiCogging::Status AntiCogging::calibrate() {
  if (!m_.sensor || m_.motor_status != FOCMotorStatus::motor_ready) {
    logf("ACOG: motor/sensor not ready\n");
    return Status::NOT_INIT;
  }
  if (cfg_.num_harmonics > MAX_HARMONICS) cfg_.num_harmonics = MAX_HARMONICS;

  enabled_ = false;  // never calibrate with compensation active

  // Save and override controller settings.
  MotionControlType prev_ctrl = m_.controller;
  float prev_vlim = m_.velocity_limit;
  m_.controller     = MotionControlType::angle;
  m_.velocity_limit = cfg_.cal_velocity_limit;

  if (m_.PID_velocity.I <= 0.0f)
    logf("ACOG WARNING: velocity PID I=0 -> steady-state torque will be wrong\n");

  // Reference: map bin k to raw mechanical angle k*BIN_WIDTH.
  // shaft_angle = dir * (raw_mech + full_rotations*2PI) - sensor_offset
  // so to reach raw mech angle th, target shaft = start + dir*(th - raw_start)
  // with continuous unwrapping.
  serviceFOC();
  const float dir        = (float)m_.sensor_direction;
  const float raw_start  = m_.sensor->getMechanicalAngle();
  const float shaft_start= m_.shaft_angle;
  const uint16_t k0      = (uint16_t)(raw_start / BIN_WIDTH) % N_POINTS;

  auto shaftForStep = [&](int32_t step) {
    // step counts bins forward from k0 (may exceed N_POINTS, may be negative)
    float raw_target = (float)(k0 + step) * BIN_WIDTH;
    return shaft_start + dir * (raw_target - raw_start);
  };

  logf("ACOG: calibration start (%u pts, 'x' aborts)\n", N_POINTS);
  const uint32_t settle = (uint32_t)cfg_.settle_time_ms;

  // Pre-roll to remove backlash/settling effects before the forward sweep.
  if (!moveAndHold(shaftForStep(-4), 300) || !moveAndHold(shaftForStep(0), 300))
    goto aborted;

  // ---- Forward sweep ----
  for (int32_t s = 0; s < (int32_t)N_POINTS; ++s) {
    if (!moveAndHold(shaftForStep(s), settle)) goto aborted;
    float v;
    if (!sampleTorque(v)) goto saturated;
    raw_fwd_[(k0 + s) % N_POINTS] = v;
    if ((s & 63) == 0) logf("ACOG fwd %3ld/%u  tq=%.4f\n", (long)s, N_POINTS, v);
  }

  // Overshoot then come back so the reverse sweep approaches from above.
  if (!moveAndHold(shaftForStep(N_POINTS + 4), 300)) goto aborted;

  // ---- Reverse sweep ----
  for (int32_t s = N_POINTS; s > 0; --s) {
    if (!moveAndHold(shaftForStep(s), settle)) goto aborted;
    float v;
    if (!sampleTorque(v)) goto saturated;
    raw_rev_[(k0 + s) % N_POINTS] = v;
    if ((s & 63) == 0) logf("ACOG rev %3ld/%u  tq=%.4f\n", (long)s, N_POINTS, v);
  }

  // Restore controller.
  m_.controller     = prev_ctrl;
  m_.velocity_limit = prev_vlim;

  {
    // Average fwd/rev: Coulomb friction flips sign and cancels; cogging stays.
    static float avg[N_POINTS];
    for (uint16_t i = 0; i < N_POINTS; ++i)
      avg[i] = 0.5f * (raw_fwd_[i] + raw_rev_[i]);

    fitHarmonics(avg);
    rebuildFromHarmonics();
  }

  valid_ = true;
  logf("ACOG: calibration done\n");
  return Status::OK;

aborted:
  m_.controller = prev_ctrl; m_.velocity_limit = prev_vlim;
  logf("ACOG: aborted\n");
  return Status::ABORTED;

saturated:
  m_.controller = prev_ctrl; m_.velocity_limit = prev_vlim;
  logf("ACOG: torque sample exceeded max_abs_sample\n");
  return Status::SATURATED;
}

// ---------------------------------------------------------------------------
// Sinusoidal (Fourier) fit over one mechanical revolution
//   f(th) = a0 + sum_{h=1..H} a_h cos(h th) + b_h sin(h th)
// ---------------------------------------------------------------------------
void AntiCogging::fitHarmonics(const float* raw) {
  const uint16_t H = cfg_.num_harmonics;
  memset(a_, 0, sizeof(a_));
  memset(b_, 0, sizeof(b_));

  double dc = 0.0;
  for (uint16_t i = 0; i < N_POINTS; ++i) dc += raw[i];
  a_[0] = (float)(dc / N_POINTS);

  for (uint16_t h = 1; h <= H; ++h) {
    double sc = 0.0, ss = 0.0;
    for (uint16_t i = 0; i < N_POINTS; ++i) {
      double th = (double)h * i * (2.0 * M_PI / N_POINTS);
      sc += raw[i] * cos(th);
      ss += raw[i] * sin(th);
    }
    a_[h] = (float)(2.0 * sc / N_POINTS);
    b_[h