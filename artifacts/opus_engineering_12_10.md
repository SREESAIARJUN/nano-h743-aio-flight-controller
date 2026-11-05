I don't have context for what "Cycle 12" refers to in your project, so I've written this as a self-contained, drop-in module against the public SimpleFOC 2.3.x API (`BLDCMotor`, `Sensor`). You may need to adjust names or conventions to match your codebase.

## Design summary

| Aspect | Approach |
|---|---|
| **Measurement** | The motor is held in angle mode at each of 512 positions per mechanical revolution. After settling, the averaged torque command (`current_sp`) needed to hold position is recorded as the cogging torque. |
| **Friction rejection** | A forward sweep and a backward sweep are averaged, which cancels direction-dependent stiction and Coulomb friction. The DC mean is then removed, so load and gravity offsets are excluded. |
| **"Sinusoidal" model** | The 512-point table is decomposed with a DFT. Only harmonics 1…H above a noise floor are kept, and the table is rebuilt from them. Cogging is periodic, so this removes measurement noise without adding phase lag. |
| **Injection** | Linear interpolation runs in IRAM with no heap use. The feedforward is added after `move()`, into `current_sp` (current modes) or `voltage.q` (voltage mode). It is safe with `motion_downsample`, because it tracks the base value and does not accumulate. |
| **Speed fade** | Compensation fades out at high speed, where rotor inertia already filters cogging and table error would only add noise. |
| **Persistence** | The table is stored in ESP32 NVS (`Preferences`) together with a sensor/motor signature and CRC. It is rejected if the zero electrical angle, direction, or pole pairs change. |

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

class AntiCogging {
public:
  static constexpr uint16_t N = 512;            // table points per mech. revolution
  static constexpr uint8_t  MAX_HARMONICS = 64; // upper bound for harmonic fit

  struct Config {
    float    settle_time_s      = 0.15f;  // wait after reaching each point
    uint16_t samples_per_point  = 200;    // averaging samples per point
    float    pos_tolerance_rad  = 0.004f; // must be inside this before sampling
    float    max_point_time_s   = 1.5f;   // timeout per point
    float    move_velocity      = 2.0f;   // rad/s velocity limit during sweep
    uint8_t  harmonics          = 24;     // harmonics kept in sinusoidal fit (0 = raw table)
    float    harmonic_floor     = 0.02f;  // drop harmonics below this fraction of the largest
    float    fade_start_rad_s   = 30.0f;  // full compensation below this speed
    float    fade_end_rad_s     = 80.0f;  // zero compensation above this speed
    float    max_ff             = 0.0f;   // clamp on |ff| (0 = use motor.current_limit)
  };

  enum class Status : uint8_t { Uncalibrated, Calibrating, Ready, Failed };

  explicit AntiCogging(BLDCMotor& motor);

  Config cfg;

  // Blocking calibration. Motor must be initFOC()'d, unloaded, and free to rotate.
  // Uses SimpleFOC's angle loop; restores the previous controller settings afterwards.
  bool calibrate(Print* log = &Serial);

  // Call right after motor.move() each loop iteration.
  void IRAM_ATTR apply();

  // Feedforward (torque-command units) at a mechanical angle [rad].
  float IRAM_ATTR lookup(float mech_angle) const;

  bool save(const char* ns = "anticog");
  bool load(const char* ns = "anticog");
  void clear();

  void  enable(bool en) { enabled_ = en && status_ == Status::Ready; }
  bool  enabled() const { return enabled_; }
  void  setGain(float g) { gain_ = g; }
  float gain() const     { return gain_; }
  Status status() const  { return status_; }
  float lastFF() const   { return last_ff_; }
  const float* table() const { return table_; }
  void dump(Print& out) const;

private:
  struct Signature {
    uint32_t magic;
    uint16_t points;
    uint16_t pole_pairs;
    float    zero_electric_angle;
    int8_t   direction;
    uint8_t  _pad[3];
    uint32_t crc;
  };
  static constexpr uint32_t MAGIC = 0xAC0C5121;

  bool  sweep(float* out, bool forward, Print* log);
  bool  holdAndMeasure(float target, float& result);
  void  runFOCFor(float seconds);
  void  harmonicFilter(float* data);
  Signature makeSignature() const;
  static uint32_t crc32(const uint8_t* d, size_t n, uint32_t crc = 0xFFFFFFFF);

  BLDCMotor& motor_;
  float      table_[N];
  float      gain_     = 1.0f;
  bool       enabled_  = false;
  Status     status_   = Status::Uncalibrated;

  // injection bookkeeping (prevents accumulation when move() is downsampled)
  float last_ff_          = 0.0f;
  float last_written_cur_ = NAN;
  float last_written_vq_  = NAN;
  float base_cur_         = 0.0f;
  float base_vq_          = 0.0f;
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <Preferences.h>
#include <math.h>

static constexpr float TWO_PI_F = 6.28318530718f;

AntiCogging::AntiCogging(BLDCMotor& motor) : motor_(motor) {
  memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------------------
// Lookup / injection (hot path)
// ---------------------------------------------------------------------------
float IRAM_ATTR AntiCogging::lookup(float a) const {
  a = fmodf(a, TWO_PI_F);
  if (a < 0) a += TWO_PI_F;
  float idx = a * (N / TWO_PI_F);
  int   i0  = (int)idx;
  float fr  = idx - (float)i0;
  i0 &= (N - 1);                       // N is a power of two
  int i1 = (i0 + 1) & (N - 1);
  return table_[i0] + fr * (table_[i1] - table_[i0]);
}

void IRAM_ATTR AntiCogging::apply() {
  if (!enabled_ || !motor_.enabled) { last_ff_ = 0; return; }

  // speed-dependent fade
  float v = fabsf(motor_.shaft_velocity);
  float fade = 1.0f;
  if (v >= cfg.fade_end_rad_s) fade = 0.0f;
  else if (v > cfg.fade_start_rad_s)
    fade = 1.0f - (v - cfg.fade_start_rad_s) / (cfg.fade_end_rad_s - cfg.fade_start_rad_s);

  // shaft_angle already includes sensor_direction; the table was built in the same frame
  float ff = (fade > 0.0f) ? gain_ * fade * lookup(motor_.shaft_angle) : 0.0f;
  float lim = cfg.max_ff > 0 ? cfg.max_ff : motor_.current_limit;
  ff = _constrain(ff, -lim, lim);
  last_ff_ = ff;

  // If move() ran since our last write, the value differs -> new base.
  // Otherwise (downsampled move) reuse the stored base so ff never accumulates.
  if (motor_.current_sp != last_written_cur_) base_cur_ = motor_.current_sp;
  motor_.current_sp = base_cur_ + ff;
  last_written_cur_ = motor_.current_sp;

  // In voltage mode move() already converted current_sp -> voltage.q,
  // so the feedforward must be applied there too.
  if (motor_.torque_controller == TorqueControlType::voltage) {
    if (motor_.voltage.q != last_written_vq_) base_vq_ = motor_.voltage.q;
    float ffv = _isset(motor_.phase_resistance) ? ff * motor_.phase_resistance : ff;
    motor_.voltage.q = _constrain(base_vq_ + ffv, -motor_.voltage_limit, motor_.voltage_limit);
    last_written_vq_ = motor_.voltage.q;
  }
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
void AntiCogging::runFOCFor(float seconds) {
  uint32_t t0 = micros(), n = 0;
  while ((micros() - t0) < (uint32_t)(seconds * 1e6f)) {
    motor_.loopFOC();
    motor_.move();
    if ((++n & 0x3FF) == 0) yield();   // keep task WDT happy
  }
}

bool AntiCogging::holdAndMeasure(float target, float& result) {
  motor_.target = target;

  // wait until position reached (or timeout)
  uint32_t t0 = micros(), n = 0;
  while (fabsf(motor_.shaft_angle - target) > cfg.pos_tolerance_rad) {
    motor_.loopFOC();
    motor_.move();
    if ((++n & 0x3FF) == 0) yield();
    if ((micros() - t0) > (uint32_t)(cfg.max_point_time_s * 1e6f)) break; // keep going, measure anyway
  }

  runFOCFor(cfg.settle_time_s);     // let the integrator converge

  double acc = 0;
  for (uint16_t s = 0; s < cfg.samples_per_point; ++s) {
    motor_.loopFOC();
    motor_.move();
    acc += motor_.current_sp;        // torque command needed to hold = -cogging torque
    delayMicroseconds(100);
  }
  result = (float)(acc / cfg.samples_per_point);
  return isfinite(result);
}

bool AntiCogging::sweep(float* out, bool forward, Print* log) {
  const float step = TWO_PI_F / N;
  const float base = motor_.shaft_angle - fmodf(motor_.shaft_angle, TWO_PI_F); // start of current rev
  // approach first point from the correct side to keep stiction consistent
  float pre = base + (forward ? -2 * step : TWO_PI_F + 2 * step);
  motor_.target = pre;
  runFOCFor(0.5f);

  for (uint16_t k = 0; k < N; ++k) {
    uint16_t i = forward ? k : (N - 1 - k);
    float t = base + i * step;
    if (!holdAndMeasure(t, out[i])) return false;
    if (log && (k % 64 == 0))
      log->printf("ACOG: %s %3u/%u  tq=%.4f\n", forward ? "fwd" : "bwd", k, N, out[i]);
  }
  return true;
}

bool AntiCogging::calibrate(Print* log) {
  if (motor_.motor_status != FOCMotorStatus::motor_ready || !motor_.sensor) {
    if (log) log->println("ACOG: motor not initialised");
    status_ = Status::Failed;
    return false;
  }

  bool was_enabled = enabled_;
  enabled_ = false;
  status_  = Status::Calibrating;

  // save controller state
  MotionControlType prev_ctrl = motor_.controller;
  float prev_vlim   = motor_.velocity_limit;
  float prev_target = motor_.target;
  unsigned int prev_ds = motor_.motion_downsample;

  motor_.controller        = MotionControlType::angle;
  motor_.velocity_limit    = cfg.move_velocity;
  motor_.motion_downsample = 0;

  // the static buffer avoids 4 kB on the loop task stack
  static float fwd[N], bwd[N];
  if (log) log->println("ACOG: forward sweep");
  bool ok = sweep(fwd, true, log);
  if (ok) { if (log) log->println("ACOG: backward sweep"); ok = sweep(bwd, false, log); }

  // restore controller state
  motor_.controller        = prev_ctrl;
  motor_.velocity_limit    = prev_vlim;
  motor_.motion_downsample = prev_ds;
  motor_.target            = (prev_ctrl == MotionControlType::angle) ? motor_.shaft_angle : prev_target;

  if (!ok) {
    status_ = Status::Failed;
    if (log) log->println("ACOG: calibration failed");
    return false;
  }

  // bidirectional average -> remove DC (load/friction offset)
  double mean = 0;
  for (uint16_t i = 0; i < N; ++i)