# Anti-Cogging Calibration and Feedforward Module for SimpleFOC (ESP32-S3)

I don't have your project's "Cycle 1" spec, so this implementation is built on stated assumptions (listed at the end). Adjust as needed.

## How It Works

**Calibration (non-blocking state machine)**
1. The motor is put in angle mode and stepped through 512 positions per mechanical revolution, first forward, then in reverse.
2. At each step it waits to settle, then averages the holding effort (`motor.current_sp`).
3. Samples are binned by the **measured** raw mechanical angle, not the commanded target. This makes the result robust to position error.
4. Forward and reverse passes are averaged so Coulomb friction cancels. The DC component is then removed.
5. The table is projected onto a truncated Fourier series. This is the "sinusoidal" model. Only harmonics up to `max_harmonic` and above `harmonic_threshold` are kept, which suppresses noise.
6. The result is stored in NVS with a magic number, version, mode tag and checksum.

**Runtime**
- An `IRAM_ATTR` linearly interpolated lookup on `sensor->getMechanicalAngle()`.
- The compensation fades out with speed.
- It is injected after `motor.move()` into `current_sp`, or into `voltage.q` for voltage mode.

---

## `AntiCogging.h`

```cpp
#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
  static constexpr uint16_t N        = 512;            // table size (power of 2)
  static constexpr uint16_t N_MASK   = N - 1;
  static constexpr uint32_t MAGIC    = 0x41434F47;     // 'ACOG'
  static constexpr uint16_t VERSION  = 1;

  struct Config {
    // --- calibration ---
    uint32_t settle_us          = 40000;   // wait after each step before sampling
    uint32_t sample_us          = 30000;   // averaging window per step
    uint32_t start_settle_us    = 1000000; // settle at start position
    uint8_t  passes             = 1;       // fwd+rev revolutions to accumulate
    float    cal_velocity_limit = 2.0f;    // rad/s limit during calibration
    // --- sinusoidal (Fourier) model ---
    bool     fourier_fit        = true;
    uint16_t max_harmonic       = 96;      // highest mechanical harmonic kept (<= N/2)
    float    harmonic_threshold = 0.002f;  // drop harmonics below this amplitude
    // --- runtime ---
    float    gain               = 1.0f;    // feedforward gain
    float    fade_start_vel     = 20.0f;   // rad/s: full compensation below
    float    fade_end_vel       = 60.0f;   // rad/s: zero compensation above
  };

  enum class State : uint8_t {
    Idle, MovingToStart, Forward, Reverse, Processing, Done, Error
  };

  explicit AntiCogging(BLDCMotor& motor);
  ~AntiCogging();

  void setConfig(const Config& c) { cfg_ = c; }
  Config& config() { return cfg_; }

  // ---------- calibration (non-blocking) ----------
  bool startCalibration();          // motor must be initFOC()'d and enabled
  void abortCalibration();
  // Call every loop iteration INSTEAD of motor.loopFOC()/motor.move() while
  // calibrating. Returns true while calibration is in progress.
  bool updateCalibration();
  State state() const { return state_; }
  float progress() const;           // 0..1

  // ---------- runtime ----------
  // Call right AFTER motor.move() each loop.
  void IRAM_ATTR inject();
  float IRAM_ATTR compensation(float mech_angle, float velocity) const;

  void enable(bool en) { enabled_ = en; }
  bool enabled() const { return enabled_; }
  bool valid() const { return valid_; }

  // ---------- persistence ----------
  bool save(const char* ns = "acog");
  bool load(const char* ns = "acog");
  bool erase(const char* ns = "acog");

  // ---------- diagnostics ----------
  const float* table() const { return table_; }
  float peakToPeak() const;
  void  printTable(Print& out) const;

private:
  struct CalBuffers {
    float    sum_f[N];
    float    sum_r[N];
    uint16_t cnt_f[N];
    uint16_t cnt_r[N];
  };

  enum class Phase : uint8_t { Settling, Sampling };

  BLDCMotor& motor_;
  Config     cfg_;

  float table_[N];
  bool  valid_   = false;
  bool  enabled_ = false;

  // calibration state
  State       state_  = State::Idle;
  Phase       phase_  = Phase::Settling;
  CalBuffers* buf_    = nullptr;
  uint32_t    t_mark_ = 0;
  uint32_t    step_   = 0;
  uint32_t    total_steps_ = 0;
  uint8_t     pass_   = 0;
  float       start_angle_ = 0.0f;
  uint8_t     torque_mode_tag_ = 0;

  // saved motor configuration
  MotionControlType saved_controller_;
  float saved_target_   = 0.0f;
  float saved_vel_limit_= 0.0f;

  inline uint16_t binOf(float mech_angle) const;
  void   finishCalibration();
  void   fillEmptyBins(float* data, const uint16_t* cnt);
  void   fourierFit();
  void   restoreMotor();
  uint32_t checksum() const;
  uint8_t  modeTag() const;
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
}

AntiCogging::~AntiCogging() {
  if (buf_) { free(buf_); buf_ = nullptr; }
}

// ====================================================================
// Helpers
// ====================================================================
inline uint16_t AntiCogging::binOf(float a) const {
  a = fmodf(a, TWO_PI_F);
  if (a < 0) a += TWO_PI_F;
  int i = (int)(a * (N / TWO_PI_F) + 0.5f);
  return (uint16_t)(i & N_MASK);
}

uint8_t AntiCogging::modeTag() const {
  // Table units equal current_sp units, which depend on torque mode
  // and on whether phase_resistance is set (voltage mode -> amps vs volts).
  uint8_t t = (uint8_t)motor_.torque_controller;
  if (motor_.torque_controller == TorqueControlType::voltage &&
      _isset(motor_.phase_resistance)) t |= 0x80;
  return t;
}

float AntiCogging::progress() const {
  if (state_ == State::Done) return 1.0f;
  if (total_steps_ == 0) return 0.0f;
  uint32_t per_pass = 2 * N;
  uint32_t done = pass_ * per_pass +
                  (state_ == State::Reverse ? N : 0) + step_;
  return fminf(1.0f, (float)done / (float)total_steps_);
}

// ====================================================================
// Calibration
// ====================================================================
bool AntiCogging::startCalibration() {
  if (!motor_.sensor || motor_.motor_status != FOCMotorStatus::motor_ready) {
    state_ = State::Error;
    return false;
  }
  if (!buf_) {
    buf_ = (CalBuffers*)heap_caps_malloc(sizeof(CalBuffers),
                                         MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!buf_) buf_ = (CalBuffers*)malloc(sizeof(CalBuffers));
    if (!buf_) { state_ = State::Error; return false; }
  }
  memset(buf_, 0, sizeof(CalBuffers));

  enabled_ = false;  // never inject while measuring

  saved_controller_ = motor_.controller;
  saved_target_     = motor_.target;
  saved_vel_limit_  = motor_.velocity_limit;

  motor_.controller     = MotionControlType::angle;
  motor_.velocity_limit = cfg_.cal_velocity_limit;

  start_angle_ = motor_.shaft_angle;
  motor_.target = start_angle_;

  torque_mode_tag_ = modeTag();
  pass_  = 0;
  step_  = 0;
  total_steps_ = (uint32_t)cfg_.passes * 2 * N;
  phase_ = Phase::Settling;
  t_mark_ = micros();
  state_ = State::MovingToStart;
  return true;
}

void AntiCogging::abortCalibration() {
  if (state_ == State::Idle || state_ == State::Done) return;
  restoreMotor();
  if (buf_) { free(buf_); buf_ = nullptr; }
  state_ = State::Idle;
}

void AntiCogging::restoreMotor() {
  motor_.controller     = saved_controller_;
  motor_.velocity_limit = saved_vel_limit_;
  motor_.target = (saved_controller_ == MotionControlType::angle)
                      ? motor_.shaft_angle : saved_target_;
}

bool AntiCogging::updateCalibration() {
  if (state_ == State::Idle || state_ == State::Done || state_ == State::Error)
    return false;

  motor_.loopFOC();
  mot