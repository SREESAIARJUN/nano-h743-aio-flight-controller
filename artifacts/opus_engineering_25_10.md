I don't have context for what "Cycle 25" refers to in your project, such as a spec revision, sprint, or internal document. I've written a self-contained implementation from first principles, so adjust names and parameters to match your spec.

## Design summary

| Aspect | Choice |
|---|---|
| Table | 512 bins over one **mechanical** revolution, indexed by `sensor->getMechanicalAngle()`. Sensor-frame indexing stays valid across reboots. |
| Measurement | Position-hold at each bin. The module averages the holding effort (`current.q` if current sensing exists, else `voltage.q`). |
| Friction rejection | A forward sweep and a reverse sweep are averaged. Coulomb friction cancels and cogging remains. |
| "Sinusoidal" | The raw table is fit with a truncated Fourier series (harmonics 1..N). This removes noise and keeps only periodic content. |
| Persistence | ESP32 NVS (`Preferences`) stores the table with a header (magic, version, direction, pole pairs, CRC32). |
| Runtime | IRAM-resident, linearly interpolated lookup with latency phase-advance, speed-dependent fade-out and a gain setting. |
| Injection | Feedforward is added to the torque command (`TorqueControlType` units). A helper runs the velocity PID externally so FF works in velocity mode too. |

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
  static constexpr uint32_t MAGIC      = 0xAC0C512A;
  static constexpr uint16_t VERSION    = 2;

  struct Config {
    uint16_t settle_ms        = 25;    // settling time per bin before sampling
    uint16_t samples_per_bin  = 64;    // averaged samples per bin
    uint8_t  harmonics        = 48;    // Fourier harmonics kept (<= TABLE_SIZE/2)
    uint8_t  passes           = 1;     // forward+reverse sweep pairs
    float    hold_P           = 0.0f;  // 0 = keep the user's angle-PID gains
    float    max_comp         = 0.0f;  // clamp on |ff|; 0 = auto (2x measured peak)
    float    fade_start_rad_s = 30.0f; // FF begins fading above this speed
    float    fade_end_rad_s   = 80.0f; // FF is zero above this speed
    float    latency_s        = 60e-6f;// phase advance (sensor+loop latency)
  };

  explicit AntiCogging(BLDCMotor& motor, Sensor& sensor);

  // Blocking calibration. Motor must be initFOC()'d and enabled.
  // Returns false on failure; the motor's previous mode is restored.
  bool calibrate(const Config& cfg = Config(), Print* log = &Serial);

  bool save(const char* ns = "acog");
  bool load(const char* ns = "acog");
  void clear();

  // Feedforward torque (same units as motor.target in torque mode).
  float IRAM_ATTR compensation();

  // Convenience: call instead of motor.move() in torque mode.
  void IRAM_ATTR moveTorque(float base_torque);

  // Velocity control with FF injected after the PID (motor must be in torque mode).
  void IRAM_ATTR moveVelocity(float target_vel);

  void  setEnabled(bool e) { enabled_ = e && valid_; }
  bool  isValid()  const   { return valid_; }
  void  setGain(float g)   { gain_ = g; }
  float peak()     const   { return peak_; }
  const float* table() const { return table_; }
  void  dump(Print& out) const;

private:
  BLDCMotor& motor_;
  Sensor&    sensor_;
  Config     cfg_;

  float table_[TABLE_SIZE];
  bool  valid_   = false;
  bool  enabled_ = false;
  float gain_    = 1.0f;
  float peak_    = 0.0f;
  float clamp_   = 0.0f;
  int8_t   stored_dir_ = 0;
  uint8_t  stored_pp_  = 0;

  struct __attribute__((packed)) Header {
    uint32_t magic; uint16_t version; uint16_t size;
    int8_t dir; uint8_t pole_pairs; uint8_t harmonics; uint8_t _pad;
    float peak; float clamp; uint32_t crc;
  };

  void   runFOCFor(uint32_t ms);
  float  measureEffort(uint16_t n);
  void   fillGaps(float* sum, uint16_t* cnt);
  void   fourierFit(uint8_t harmonics);
  static uint32_t crc32(const uint8_t* d, size_t n, uint32_t c = 0xFFFFFFFF);
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>
#include <string.h>

static constexpr float BIN_W = _2PI / AntiCogging::TABLE_SIZE;

AntiCogging::AntiCogging(BLDCMotor& m, Sensor& s) : motor_(m), sensor_(s) {
  memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------- helpers
void AntiCogging::runFOCFor(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    motor_.loopFOC();
    motor_.move();
  }
  yield();  // let other tasks/WDT run between bins
}

float AntiCogging::measureEffort(uint16_t n) {
  // Holding effort = q-axis command needed to resist cogging at this angle.
  const bool use_current = (motor_.current_sense != nullptr) &&
                           (motor_.torque_controller != TorqueControlType::voltage);
  double acc = 0;
  for (uint16_t i = 0; i < n; i++) {
    motor_.loopFOC();
    motor_.move();
    acc += use_current ? motor_.current.q : motor_.voltage.q;
  }
  return (float)(acc / n);
}

void AntiCogging::fillGaps(float* sum, uint16_t* cnt) {
  // Average filled bins; linearly interpolate (circularly) any empty bins.
  int first = -1;
  for (int i = 0; i < TABLE_SIZE; i++) if (cnt[i]) { first = i; break; }
  if (first < 0) return;
  for (int i = 0; i < TABLE_SIZE; i++) if (cnt[i]) sum[i] /= cnt[i];

  int prev = first;
  for (int k = 1; k <= TABLE_SIZE; k++) {
    int i = (first + k) % TABLE_SIZE;
    if (!cnt[i]) continue;
    int gap = (i - prev + TABLE_SIZE) % TABLE_SIZE;
    if (gap == 0) gap = TABLE_SIZE;
    for (int g = 1; g < gap; g++) {
      float t = (float)g / gap;
      sum[(prev + g) % TABLE_SIZE] = sum[prev] * (1 - t) + sum[i] * t;
    }
    prev = i;
  }
}

void AntiCogging::fourierFit(uint8_t H) {
  // Keep harmonics 1..H of the measured profile (DC removed = friction/load bias).
  if (H > TABLE_SIZE / 2) H = TABLE_SIZE / 2;
  static float a[TABLE_SIZE / 2 + 1], b[TABLE_SIZE / 2 + 1];
  for (int k = 1; k <= H; k++) {
    double ak = 0, bk = 0;
    for (int n = 0; n < TABLE_SIZE; n++) {
      float ph = BIN_W * k * n;
      ak += table_[n] * cosf(ph);
      bk += table_[n] * sinf(ph);
    }
    a[k] = 2.0f * ak / TABLE_SIZE;
    b[k] = 2.0f * bk / TABLE_SIZE;
  }
  peak_ = 0;
  for (int n = 0; n < TABLE_SIZE; n++) {
    float v = 0;
    for (int k = 1; k <= H; k++) {
      float ph = BIN_W * k * n;
      v += a[k] * cosf(ph) + b[k] * sinf(ph);
    }
    table_[n] = v;
    peak_ = fmaxf(peak_, fabsf(v));
  }
}

// ---------------------------------------------------------------- calibration
bool AntiCogging::calibrate(const Config& cfg, Print* log) {
  cfg_ = cfg;
  if (motor_.motor_status != FOCMotorStatus::motor_ready) {
    if (log) log->println(F("ACOG: motor not ready (run initFOC first)"));
    return false;
  }

  // Save and override the control mode.
  auto prev_ctrl   = motor_.controller;
  float prev_tgt   = motor_.target;
  float prev_P     = motor_.P_angle.P;
  bool  prev_en    = enabled_;
  enabled_ = false;

  motor_.controller = MotionControlType::angle;
  if (cfg_.hold_P > 0) motor_.P_angle.P = cfg_.hold_P;

  static float    sum[TABLE_SIZE];
  static uint16_t cnt[TABLE_SIZE];
  memset(sum, 0, sizeof(sum));
  memset(cnt, 0, sizeof(cnt));

  // shaft_angle is in the motor frame (direction applied); step in that frame.
  motor_.loopFOC(); motor_.move();
  const float start = motor_.shaft_angle;
  const float step  = _2PI / TABLE_SIZE;

  if (log) log->printf("ACOG: calibrating %u bins x %u passes...\n",
                       TABLE_SIZE, cfg_.passes);
  uint32_t t_start = millis();

  for (uint8_t p = 0; p < cfg_.passes; p++) {
    for (int dir = 0; dir < 2; dir++) {           // 0 = forward, 1 = reverse
      for (int i = 0; i <= TABLE_SIZE; i++) {
        int idx = dir ? (TABLE_SIZE - i) : i;
        motor_.target = start + idx * step;
        runFOCFor(cfg_.settle_ms);
        float effort = measureEffort(cfg_.samples_per_bin);

        float mech = sensor_.getMechanicalAngle();   // sensor frame, 0..2pi
        int bin = (int)(mech / BIN_W + 0.5f) % TABLE_SIZE;
        sum[bin] += effort;
        cnt[bin]++;
      }
      if (log) log->printf("ACOG: pass %u %s done (%lus)\n", p,
                           dir ? "rev" : "fwd", (millis() - t_start) / 1000);
    }
  }

  // Effort is measured in the motor frame; convert to the sensor-angle-indexed table.
  // The table holds the torque the controller must ADD = the measured holding effort.
  fillGaps(sum, cnt);
  memcpy(table_, sum, sizeof(table_));

  int filled = 0;
  for (int i = 0; i < TABLE_SIZE; i++) filled += cnt[i] ? 1 : 0;

  fourierFit(cfg_.harmonics);
  clamp_      = cfg_.max_comp > 0 ? cfg_.max_comp : 2.0f * peak_;
  stored_dir_ = (int8_t)motor_.sensor_direction;
  stored_pp_  = (uint8_t)motor_.pole_pairs;

  // Restore.
  motor_.controller = prev_ctrl;
  motor_.P_angle.P  = prev_P;
  motor_.target     = (prev_ctrl == MotionControlType::angle) ? motor_.shaft_angle
                                                              : prev_tgt;

  valid_ = filled > TABLE_SIZE / 4;
  enabled_ = valid_ ? true : prev_en;
  if (log) log->printf("ACOG: %s, coverage %d/%d bins, peak %.