I don't have context for what "Cycle 14" refers to in your project, such as a prior spec, ticket, or design document. Below is a complete, self-contained implementation based on standard SimpleFOC (v2.3.x) APIs and ESP32-S3 Arduino facilities. If Cycle 14 has specific requirements (different table resolution, storage format, harmonic set, or API), share them and I'll adapt it.

## Design summary

- **Table:** 512 bins over one mechanical revolution. Each bin stores the torque command (in `current_sp` units: amps in current modes, volts in voltage mode) needed to cancel cogging at that angle.
- **Calibration:** The motor is held in angle mode at each bin, first forward and then backward. Averaging the two sweeps cancels Coulomb friction and hysteresis. The averaged holding effort is the cogging profile.
- **Sinusoidal fit:** The raw profile is DFT-decomposed. Only the DC term is removed and the strongest *K* harmonics are kept. The table is then rebuilt as a smooth sum of sinusoids, which removes sensor and measurement noise.
- **Injection:** After `motor.move()`, the class adds the interpolated feedforward to `motor.current_sp`. `loopFOC()` consumes `current_sp` in every torque mode, so this works for voltage, DC-current and FOC-current modes. The feedforward fades out above a configurable speed, where cogging is filtered by inertia anyway.
- **Persistence:** The table is stored in NVS through `Preferences`, with a header, version, sensor-offset check and CRC32.
- **Real-time safety:** The lookup is `IRAM_ATTR`, branch-light, and uses linear interpolation with no allocation.

---

### `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

class AntiCogging {
public:
  static constexpr uint16_t TABLE_SIZE  = 512;
  static constexpr uint16_t MAX_HARMONICS = 64;
  static constexpr uint32_t STORAGE_MAGIC   = 0xAC0C5120;
  static constexpr uint16_t STORAGE_VERSION = 1;

  struct Config {
    uint16_t settle_ms        = 40;    // wait after each step before sampling
    uint16_t samples_per_bin  = 64;    // FOC-loop samples averaged per bin
    uint16_t harmonics_kept   = 24;    // strongest harmonics retained in fit
    float    max_compensation = 1.0f;  // clamp on |ff| (A or V)
    float    fade_start_vel   = 20.0f; // rad/s: full ff below this
    float    fade_end_vel     = 60.0f; // rad/s: zero ff above this
    float    gain             = 1.0f;  // global ff scale (tuning)
    bool     use_sinusoidal_fit = true;
  };

  explicit AntiCogging(BLDCMotor& motor, const char* nvs_namespace = "acog");

  // ---- calibration ----
  // Blocking. Motor must be initFOC()'d and angle PID reasonably stiff.
  // Returns true on success. Progress printed to `log` if non-null.
  bool calibrate(Print* log = &Serial);

  // ---- runtime ----
  // Call AFTER motor.move(), BEFORE next motor.loopFOC().
  void IRAM_ATTR apply();
  // Pure lookup, mech angle in rad (any range).
  float IRAM_ATTR lookup(float mech_angle) const;

  void enable(bool e) { enabled_ = e && valid_; }
  bool isEnabled() const { return enabled_; }
  bool isValid()  const { return valid_; }

  // ---- persistence ----
  bool save();
  bool load();
  void clear();

  // ---- diagnostics ----
  void  dumpTable(Print& out) const;
  float peakToPeak() const;
  const float* table() const { return table_; }

  Config config;

private:
  void   sinusoidalFit(const float* raw, float* out, uint16_t keep) const;
  float  sampleHoldingEffort(float target_angle, uint16_t settle_ms, uint16_t n);
  void   runFor(uint32_t ms);
  uint32_t crc32(const uint8_t* data, size_t len) const;

  BLDCMotor&  motor_;
  const char* ns_;
  float       table_[TABLE_SIZE];
  bool        valid_   = false;
  bool        enabled_ = false;
  float       last_ff_ = 0.0f;

  static constexpr float BIN_SCALE = TABLE_SIZE / _2PI;
};
```

### `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <Preferences.h>
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& motor, const char* nvs_namespace)
  : motor_(motor), ns_(nvs_namespace) {
  memset(table_, 0, sizeof(table_));
}

// ============================================================
//  Runtime
// ============================================================
float IRAM_ATTR AntiCogging::lookup(float mech_angle) const {
  // wrap into [0, 2π)
  float a = fmodf(mech_angle, _2PI);
  if (a < 0) a += _2PI;

  float idx  = a * BIN_SCALE;
  uint16_t i0 = (uint16_t)idx;
  float frac = idx - (float)i0;
  i0 &= (TABLE_SIZE - 1);                 // TABLE_SIZE is power of 2
  uint16_t i1 = (i0 + 1) & (TABLE_SIZE - 1);

  return table_[i0] + frac * (table_[i1] - table_[i0]);
}

void IRAM_ATTR AntiCogging::apply() {
  if (!enabled_) { last_ff_ = 0; return; }

  // velocity-based fade
  float v = fabsf(motor_.shaft_velocity);
  float w;
  if (v <= config.fade_start_vel)      w = 1.0f;
  else if (v >= config.fade_end_vel)   w = 0.0f;
  else w = 1.0f - (v - config.fade_start_vel) /
                  (config.fade_end_vel - config.fade_start_vel);
  if (w == 0.0f) { last_ff_ = 0; return; }

  // use raw sensor mechanical angle (direction/offset independent of shaft_angle sign)
  float ang = motor_.sensor->getMechanicalAngle();
  float ff  = config.gain * w * lookup(ang);
  ff = _constrain(ff, -config.max_compensation, config.max_compensation);

  motor_.current_sp += ff;
  last_ff_ = ff;
}

// ============================================================
//  Calibration
// ============================================================
void AntiCogging::runFor(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    motor_.loopFOC();
    motor_.move();
  }
}

float AntiCogging::sampleHoldingEffort(float target, uint16_t settle_ms, uint16_t n) {
  motor_.target = target;
  runFor(settle_ms);

  double acc = 0;
  for (uint16_t k = 0; k < n; ++k) {
    motor_.loopFOC();
    motor_.move();
    acc += motor_.current_sp;   // controller output = torque needed to hold
  }
  return (float)(acc / n);
}

bool AntiCogging::calibrate(Print* log) {
  if (!motor_.sensor || motor_.motor_status != FOCMotorStatus::motor_ready) {
    if (log) log->println(F("ACOG: motor not ready (initFOC first)"));
    return false;
  }

  // Save and override motor state
  bool was_enabled = enabled_;
  enabled_ = false;
  MotionControlType prev_ctrl = motor_.controller;
  float prev_target = motor_.target;
  float prev_vlimit = motor_.velocity_limit;

  motor_.controller = MotionControlType::angle;
  motor_.velocity_limit = 2.0f;           // slow, gentle moves

  float* fwd = (float*)heap_caps_malloc(TABLE_SIZE * sizeof(float), MALLOC_CAP_8BIT);
  float* bwd = (float*)heap_caps_malloc(TABLE_SIZE * sizeof(float), MALLOC_CAP_8BIT);
  if (!fwd || !bwd) {
    if (log) log->println(F("ACOG: alloc failed"));
    free(fwd); free(bwd);
    return false;
  }

  // Map table bin -> shaft angle. Table is indexed by sensor mechanical angle,
  // so anchor bin 0 to the current sensor reading.
  motor_.loopFOC(); motor_.move();
  float mech0  = motor_.sensor->getMechanicalAngle();
  float shaft0 = motor_.shaft_angle;
  int   dir    = (motor_.sensor_direction == Direction::CCW) ? -1 : 1;
  const float step = _2PI / TABLE_SIZE;

  // shaft target that places the sensor at bin i
  auto targetFor = [&](uint16_t i) {
    float d = i * step - mech0;
    d = fmodf(d, _2PI); if (d < 0) d += _2PI;
    return shaft0 + dir * d;
  };

  if (log) log->println(F("ACOG: forward sweep"));
  // pre-roll to remove backlash
  sampleHoldingEffort(targetFor(0) - dir * 4 * step, config.settle_ms * 4, 1);
  for (uint16_t i = 0; i < TABLE_SIZE; ++i) {
    fwd[i] = sampleHoldingEffort(targetFor(i), config.settle_ms, config.samples_per_bin);
    if (log && (i % 64) == 0) log->printf("  %u/%u\n", i, TABLE_SIZE);
  }

  if (log) log->println(F("ACOG: backward sweep"));
  sampleHoldingEffort(targetFor(TABLE_SIZE - 1) + dir * 4 * step + dir * 0.0f,
                      config.settle_ms * 4, 1);
  for (int i = TABLE_SIZE - 1; i >= 0; --i) {
    bwd[i] = sampleHoldingEffort(targetFor(i), config.settle_ms, config.samples_per_bin);
    if (log && (i % 64) == 0) log->printf("  %d\n", i);
  }

  // Average sweeps (cancels friction) and remove DC (gravity/bias)
  double mean = 0;
  for (uint16_t i = 0; i < TABLE_SIZE; ++i) {
    fwd[i] = 0.5f * (fwd[i] + bwd[i]);
    mean += fwd[i];
  }
  mean /= TABLE_SIZE;
  for (uint16_t i = 0; i < TABLE_SIZE; ++i) fwd[i] -= (float)mean;

  // Feedforward must SUPPLY the holding torque the controller was producing,
  // so the table holds the measured effort directly.
  if (config.use_sinusoidal_fit)
    sinusoidalFit(fwd, table_, config.harmonics_kept);
  else
    memcpy(table_, fwd, sizeof(table_));

  free(fwd); free(bwd);

  // Restore motor state
  motor_.controller     = prev_ctrl;
  motor_.velocity_limit = prev_vlimit;
  motor_.target         = prev_target;

  valid_   = true;
  enabled_ = true;
  (void)was_enabled;

  if (log) log->printf("ACOG: done, p-p = %.4f\n", peakToPeak());
  return true;
}

// ============================================================
//  Sinusoidal (Fourier) fit: keep strongest K harmonics
// ============================================================
void AntiCogging::sinusoidalFit(const float* raw, float* out, uint16_t keep) const {
  const uint16_t H = TABLE_SIZE / 2;     // Nyquist
  static float re[TABLE_SIZE / 2 + 1];
  static float im[TABLE_SIZE / 2 + 1];
  static float mag[TABLE_SIZE / 2 + 1];

  // DFT (O(N*H), one-time; precomputed twiddle via recurrence)
  for (uint16_t k = 1; k <= H; ++k) {
    double sr = 0, si = 0;
    const double w = 2.0 * M_PI * k / TABLE_SIZE;
    for (uint16_t n = 0; n < TABLE_SIZE; ++n) {
      sr += raw[n] * cos(w * n);
      si -= raw[n] * sin(w * n);
    }
    re[k] = (float)sr; im[k] = (float)si;
    mag[k] = sqrtf(re[k] * re[k] + im[k] * im[k]);
  }

  // select top-K harmonics by magnitude
  if (keep > MAX_HARMONICS) keep = MAX_HARMONICS;
  if (keep > H) keep = H;
  bool sel[TABLE_SIZE / 2 + 1] = {false};
  for (uint16_t j = 0; j < keep; ++j) {
    uint16_t best = 0; float bm = -1;
    for (uint16_t k = 1; k <= H; ++k)
      if (!sel[k] && mag[k] > bm) { bm = mag[k]; best = k; }
    if (best) sel[best] = true;
  }

  // inverse reconstruction
  for (uint16_t n = 0; n < TABLE_SIZE; ++n) {
    double v = 0;
    for (uint16_t k = 1; k <= H; ++k) {
      if (!sel[k]) continue;
      const double w = 2.0 * M_PI * k * n / TABLE_SIZE;
      double scale = (k == H) ? 1.0 : 2.0;   // Nyquist bin not doubled
      v += scale * (re[k] * cos(w) - im[k] * sin(w));
    }
    out[n] = (float)(v / TABLE_SIZE);
  }
}

// ============================================================
//  Persistence (NVS)
// ============================================================
struct __attribute__((packed