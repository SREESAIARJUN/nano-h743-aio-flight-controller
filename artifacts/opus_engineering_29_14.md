# Anti-Cogging Calibration and Feedforward Module for SimpleFOC (ESP32-S3)

I don't have context for "Cycle 29," so I treated it as a version label and added it as a revision constant. If it refers to specific project requirements, share them and I'll adapt the code.

## How it works

**Calibration**
1. The motor is placed in stiff angle control and stepped through 512 positions per mechanical revolution. It does a forward pass and then a backward pass.
2. At each position, it records the torque command needed to hold that position. Samples are filed by the angle the sensor actually reads, so encoder lag and PID error don't shift the table.
3. The forward and backward passes are averaged. This cancels the direction-dependent part of Coulomb friction.
4. The table is fitted with a truncated Fourier series of up to H harmonics, and the DC term is optionally removed. This is the "sinusoidal" model. It discards noise and keeps the periodic cogging profile.
5. The result is stored in NVS with a CRC check.

**Injection**
- It runs after `motor.move()`, once per loop.
- It reads the 512-entry table with circular linear interpolation.
- It applies a velocity-proportional phase lead to compensate for loop latency.
- It fades the feedforward out at high speed.
- It adds the result to `current_sp`, or to `voltage.q` when using voltage-mode torque control.

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
  static constexpr uint16_t MAX_HARMONICS = 128;
  static constexpr uint32_t STORAGE_MAGIC = 0xAC0C0029; // rev "29"
  static constexpr uint16_t STORAGE_VER   = 29;

  struct Config {
    // Calibration
    float    cal_P_angle      = 40.0f;  // stiff angle loop during calibration
    float    cal_vel_limit    = 2.0f;   // rad/s
    uint32_t settle_us        = 120000; // settle time per bin
    uint32_t sample_us        = 80000;  // sampling window per bin
    bool     bidirectional    = true;   // fwd + bwd pass (friction cancel)

    // Sinusoidal (Fourier) fit
    uint16_t harmonics        = 48;     // 0 = raw table, no fit
    bool     remove_dc        = true;   // drop gravity/friction bias

    // Injection
    float    gain             = 1.0f;   // overall feedforward scale
    float    phase_lead_s     = 0.0003f;// latency compensation [s]
    float    fade_start_rads  = 30.0f;  // full FF below this |vel|
    float    fade_end_rads    = 80.0f;  // zero FF above this |vel|

    // NVS
    const char* nvs_namespace = "anticog";
  };

  using ProgressCb = void (*)(uint8_t pass, uint16_t bin, uint16_t total);

  explicit AntiCogging(FOCMotor& motor);

  Config& config() { return cfg_; }

  // Blocking calibration. Motor must be initialised (initFOC done).
  bool calibrate(ProgressCb cb = nullptr);

  // Call every loop AFTER motor.move()
  void apply();

  // Pure lookup (mechanical sensor angle, rad) -> torque command units
  float lookup(float mech_angle) const;

  void enable(bool en) { enabled_ = en && valid_; }
  bool enabled() const { return enabled_; }
  bool valid()   const { return valid_; }
  float lastFeedforward() const { return last_ff_; }

  bool save();
  bool load();
  bool erase();

  void printTable(Print& out) const;
  void printStats(Print& out) const;

  const float* table() const { return table_; }

private:
  FOCMotor& m_;
  Config cfg_;
  float table_[TABLE_SIZE];
  volatile bool enabled_ = false;
  bool  valid_   = false;
  float last_ff_ = 0.0f;
  float stat_pk2pk_ = 0, stat_rms_ = 0, stat_fit_resid_ = 0;

  // calibration scratch (heap-allocated only during calibration)
  struct Acc { float sum; uint32_t n; };

  bool runPass(Acc* acc, int dir, uint8_t pass_id, ProgressCb cb);
  void runFOCFor(uint32_t us, float target, Acc* acc);
  static void fillGaps(float* y, const bool* have);
  void fourierFit(float* y);
  void computeStats();
  static uint32_t fnv1a(const void* data, size_t len);

  static inline uint16_t binOf(float a) {
    a = _normalizeAngle(a);
    int b = (int)(a * (TABLE_SIZE / _2PI) + 0.5f);
    return (uint16_t)(b & (TABLE_SIZE - 1));
  }
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"

AntiCogging::AntiCogging(FOCMotor& motor) : m_(motor) {
  memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------
float AntiCogging::lookup(float mech_angle) const {
  float a = _normalizeAngle(mech_angle) * (TABLE_SIZE / _2PI);
  int   i0 = (int)a;
  float f  = a - (float)i0;
  i0 &= (TABLE_SIZE - 1);
  int   i1 = (i0 + 1) & (TABLE_SIZE - 1);
  return table_[i0] + f * (table_[i1] - table_[i0]);
}

void AntiCogging::apply() {
  if (!enabled_ || !m_.sensor) { last_ff_ = 0; return; }

  const float vel  = m_.shaft_velocity;
  const float avel = fabsf(vel);

  // velocity fade
  float scale;
  if (avel <= cfg_.fade_start_rads) scale = 1.0f;
  else if (avel >= cfg_.fade_end_rads) { last_ff_ = 0; return; }
  else scale = 1.0f - (avel - cfg_.fade_start_rads) /
                      (cfg_.fade_end_rads - cfg_.fade_start_rads);

  // Table indexed by raw sensor angle; shaft velocity is in motor frame.
  float ang = m_.sensor->getMechanicalAngle()
            + (float)((int)m_.sensor_direction) * vel * cfg_.phase_lead_s;

  float ff = cfg_.gain * scale * lookup(ang);
  last_ff_ = ff;

  if (m_.torque_controller == TorqueControlType::voltage) {
    // move() has already converted current_sp -> voltage.q
    float dv = _isset(m_.phase_resistance) ? ff * m_.phase_resistance : ff;
    m_.current_sp += ff;
    m_.voltage.q = _constrain(m_.voltage.q + dv,
                              -m_.voltage_limit, m_.voltage_limit);
  } else {
    // dc_current / foc_current: consumed by next loopFOC()
    m_.current_sp = _constrain(m_.current_sp + ff,
                               -m_.current_limit, m_.current_limit);
  }
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
void AntiCogging::runFOCFor(uint32_t us, float target, Acc* acc) {
  const uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < us) {
    m_.loopFOC();
    m_.move(target);
    if (acc) {
      uint16_t b = binOf(m