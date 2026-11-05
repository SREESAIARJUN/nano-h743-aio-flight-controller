# Anti-Cogging Calibration & Feedforward Module for SimpleFOC (ESP32-S3)

I don't know what "Cycle 5" refers to in your project, so I built this against **SimpleFOC 2.3.x `BLDCMotor`** with these assumptions. Adjust them if your spec differs:

- **Table:** 512 bins over one mechanical revolution, indexed by the sensor's raw mechanical angle. This requires an absolute sensor such as an AS5600, AS5047 or MT6701.
- **"Sinusoidal":** the raw measured table is projected onto a truncated Fourier series of mechanical harmonics. This removes noise, and the smoothed table is what gets stored and injected.
- **Calibration method:** the motor holds position at each bin under a stiff angle loop. The steady-state torque command needed to hold there equals the cogging torque. Forward and reverse sweeps are averaged to cancel Coulomb friction.
- **Injection:** after `motor.move()`, the module adds feedforward to `current_sp`, and also to `voltage.q` in voltage-torque mode. The table is stored in the same units as `current_sp` (A or V, depending on `torque_controller`).
- **Storage:** NVS via `Preferences`, with a magic number, version, sensor fingerprint, and CRC32.

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
  static constexpr uint32_t NVS_MAGIC     = 0xAC0C5121;
  static constexpr uint16_t NVS_VERSION   = 1;

  struct Config {
    // Calibration
    float    settle_ms        = 60.0f;   // wait after moving to a bin
    uint16_t sample_count     = 64;      // samples averaged per bin
    float    sample_interval_us = 250.0f;// spacing between samples
    float    cal_P_angle      = 30.0f;   // stiff hold gains used during cal
    float    cal_P_vel        = 0.3f;
    float    cal_I_vel        = 15.0f;
    float    cal_vel_limit    = 4.0f;    // rad/s while stepping between bins
    uint8_t  harmonics        = 48;      // Fourier terms kept (<= TABLE_SIZE/2)
    bool     bidirectional    = true;    // fwd+rev sweep to cancel friction
    // Injection
    float    gain             = 1.0f;    // global FF scale
    float    fade_start_rad_s = 30.0f;   // FF fully on below this speed
    float    fade_end_rad_s   = 80.0f;   // FF fully off above this speed
    float    ff_limit         = 2.0f;    // clamp (A or V)
  };

  enum class State : uint8_t { Idle, Calibrating, Ready, Error };

  AntiCogging(BLDCMotor& motor, Sensor& sensor);

  void   setConfig(const Config& c) { cfg_ = c; }
  Config& config() { return cfg_; }

  // Blocking calibration (motor must be initFOC()'d). Returns true on success.
  // progressCb(percent) is optional.
  bool calibrate(void (*progressCb)(uint8_t) = nullptr);

  // Call every loop AFTER motor.loopFOC() ... motor.move(target).
  void IRAM_ATTR apply();

  // Lookup FF for an arbitrary mechanical angle (rad).
  float IRAM_ATTR lookup(float mech_angle) const;

  bool save(const char* ns = "anticog");
  bool load(const char* ns = "anticog");
  void clear();

  void enable(bool en) { enabled_ = en && state_ == State::Ready; }
  bool enabled() const { return enabled_; }
  State state() const { return state_; }
  float lastFF() const { return last_ff_; }

  void dumpTable(Stream& s) const;

private:
  BLDCMotor& motor_;
  Sensor&    sensor_;
  Config     cfg_;
  State      state_   = State::Idle;
  bool       enabled_ = false;
  float      last_ff_ = 0.0f;

  float table_[TABLE_SIZE];          // smoothed, injected table
  float raw_fwd_[TABLE_SIZE];        // scratch for calibration
  float raw_rev_[TABLE_SIZE];

  static constexpr float BIN_RAD = _2PI / TABLE_SIZE;
  static constexpr float RAD_BIN = TABLE_SIZE / _2PI;

  bool  sweep(float* out, bool forward, float base_angle,
              void (*cb)(uint8_t), uint8_t pct0, uint8_t pct1);
  void  holdAt(float target_shaft_angle, uint32_t ms);
  float measureBin(float target_shaft_angle);
  void  fourierSmooth(const float* in, float* out, uint8_t harmonics);
  static uint32_t crc32(const uint8_t* d, size_t n);
  uint32_t sensorFingerprint() const;
  static inline float wrap2pi(float a) {
    a = fmodf(a, _2PI);
    return a < 0 ? a + _2PI : a;
  }
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"

AntiCogging::AntiCogging(BLDCMotor& motor, Sensor& sensor)
  : motor_(motor), sensor_(sensor) {
  memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------- runtime
float IRAM_ATTR AntiCogging::lookup(float mech_angle) const {
  float pos = wrap2pi(mech_angle) * RAD_BIN;
  int   i0  = (int)pos;
  float fr  = pos - (float)i0;
  i0 &= (TABLE_SIZE - 1);                  // TABLE_SIZE is power of 2
  int   i1  = (i0 + 1) & (TABLE_SIZE - 1);
  return table_[i0] + fr * (table_[i1] - table_[i0]);
}

void IRAM_ATTR AntiCogging::apply() {
  if (!enabled_) { last_ff_ = 0.0f; return; }

  // Speed-dependent fade: cogging is filtered by inertia at speed,
  // and sensor latency makes FF phase-wrong there anyway.
  float w = fabsf(motor_.shaft_velocity);
  float fade;
  if (w <= cfg_.fade_start_rad_s)    fade = 1.0f;
  else if (w >= cfg_.fade_end_rad_s) fade = 0.0f;
  else fade = 1.0f - (w - cfg_.fade_start_rad_s) /
                     (cfg_.fade_end_rad_s - cfg_.fade_start_rad_s);
  if (fade <= 0.0f) { last_ff_ = 0.0f; return; }

  // Raw mechanical angle (independent of motor zero offset / direction).
  float ff = cfg_.gain * fade * lookup(sensor_.getMechanicalAngle());
  ff = _constrain(ff, -cfg_.ff_limit, cfg_.ff_limit);
  last_ff_ = ff;

  motor_.current_sp += ff;
  if (motor_.torque_controller == TorqueControlType::voltage) {
    // In voltage mode move() already wrote voltage.q from current_sp.
    float r = _isset(motor_.phase_resistance) ? motor_.phase_resistance : 1.0f;
    motor_.voltage.q = _constrain(motor_.voltage.q + ff * (_isset(motor_.phase_resistance) ? r : 1.0f),
                                  -motor_.voltage_limit, motor_.voltage_limit);
  }
}

// ------------------------------------------------------------ calibration
void AntiCogging::holdAt(float target, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    motor_.loopFOC();
    motor_.move(target);
  }
}

float AntiCogging::measureBin(float target) {
  holdAt(target, (uint32_t)cfg_.settle_ms);
  double acc = 0.0;
  for (uint16_t s = 0; s < cfg_.sample_count; ++s) {
    uint32_t t0 = micros();
    do { motor_.loopFOC(); motor_.move(target); }
    while ((float)(micros() - t0) < cfg_.sample_interval_us);
    acc += motor_.current_sp;   // effort required to hold == -cogging
  }
  return (float)(acc / cfg_.sample_count);
}

bool AntiCogging::sweep(float* out, bool forward, float base,
                        void (*cb)(uint8_t), uint8_t p0, uint8_t p1) {
  // Bins are defined in raw mechanical angle; map to shaft_angle frame.
  int dir = (motor_.sensor_direction == Direction::CCW) ? -1 : 1;
  for (uint16_t k = 0; k < TABLE_SIZE; ++k) {
    uint16_t bin = forward ? k : (TABLE_SIZE - 1 - k);
    float mech_target = bin * BIN_RAD;
    float delta = wrap2pi(mech_target - base);   // 0..2π ahead of start
    float shaft_target = motor_.shaft_angle_sp_start_ + dir * delta;
    out[bin] = measureBin(shaft_target);

    // sanity: did we actually reach the bin?
    float err = fabsf(wrap2pi(sensor_.getMechanicalAngle() - mech_target + _PI) - _PI);
    if (err > 4.0f * BIN_RAD) {
      Serial.printf("[AC] bin %u position error %.4f rad\n", bin, err);
      return false;
    }
    if (cb && (k & 15) == 0) cb(p0 + (uint32_t)(p1 - p0) * k / TABLE_SIZE);
  }
  return true;
}
```

**Header fix.** `sweep()` above references a reference point. Add this member to the class in `AntiCogging.h`:

```cpp
  float shaft_angle_sp_start_ = 0.0f;   // shaft_angle at mech angle 'base'
```

Make it accessible from `sweep` (it is a member of `AntiCogging`), then use `shaft_angle_sp_start_` instead of `motor_.shaft_angle_sp_start_`:

```cpp
    float shaft_target = shaft_angle_sp_start_ + dir * delta;
```

Continue `AntiCogging.cpp`:

```cpp
bool AntiCogging::calibrate(void (*cb)(uint8_t)) {
  if (motor_.motor_status != FOCMotorStatus::motor_ready) {
    state_ = State::Error; return false;
  }
  state_ = State::Calibrating;
  bool was_enabled = enabled_;
  enabled_ = false;

  // Save controller state
  auto  saved_ctrl  = motor_.controller;
  float sP_ang = motor_.P_angle.P;
  float sP_vel = motor_.PID_velocity.P, sI_vel = motor_.PID_velocity.I;
  float sD_vel = motor_.PID_velocity.D, sVlim = motor_.velocity_limit;

  motor_.controller     = MotionControlType::angle;
  motor_.P_angle.P      = cfg_.cal_P_angle;
  motor_.PID_velocity.P = cfg_.cal_P_vel;
  motor_.PID_velocity.I = cfg_.cal_I_vel;
  motor_.PID_velocity.D = 0;
  motor_.velocity_limit = cfg_.cal_vel_limit;
  motor_.PID_velocity.reset();
  motor_.P_angle.reset();

  // Reference: map raw mech angle 0 to a shaft_angle setpoint.
  motor_.loopFOC();
  sensor_.update();
  float base = wrap2pi(sensor_.getMechanicalAngle());
  shaft_angle_sp_start_ = motor_.shaft_angle;

  bool ok = sweep(raw_fwd_, true, base, cb, 0, cfg_.bidirectional ? 50 : 100);
  if (ok && cfg_.bidirectional) {
    // go one full turn ahead so reverse sweep approaches from the other side
    ok = sweep(raw_rev_, false, base, cb, 50, 100);
  }

  // restore controller
  motor_.controller     = saved_ctrl;
  motor_.P_angle.P      = sP_ang;
  motor_.PID_velocity.P = sP_vel; motor_.PID_velocity.I = sI_vel;
  motor_.PID_velocity.D = sD_vel; motor_.velocity_limit = sVlim;
  motor_.PID_velocity.reset(); motor_.P_angle.reset();

  if (!ok) { state_ = State::Error; enabled_ = was_enabled; return false; }

  if (cfg_.bidirectional)
    for (uint16_t i = 0; i < TABLE_SIZE; ++i)
      raw_fwd_[i] = 0.5f * (raw_fwd_[i] + raw_rev_[i]);   // cancels friction

  // remove DC (load torque / gravity bias), then sinusoidal fit
  double mean = 0;
  for (auto v : raw_fwd_) mean += v;
  mean /= TABLE_SIZE;
  for (auto& v : raw_fwd_) v -= (float)mean;

  fourierSmooth(raw_fwd_, table_, cfg_.harmonics);

  state_ = State::Ready;
  enabled_ = true;
  if (cb) cb(100);
  return true;
}

// Project onto harmonics 1..H of the mechanical revolution.
void AntiCogging::fourierSmooth(const float* in, float* out, uint8_t H) {
  H = min<uint8_t>(H, TABLE_SIZE / 2 