I don't have context for what "Cycle 16" refers to, such as a prior spec, project log, or numbering scheme, so I can't match it exactly. Below is a complete, self-contained implementation for SimpleFOC 2.3.x on ESP32-S3. If Cycle 16 has specific requirements (table units, harmonic count, storage format, API names), tell me and I'll adapt it.

## Design summary

- **512 bins** over one mechanical revolution, indexed by the raw sensor mechanical angle. This is independent of the zero offset and direction.
- **Calibration** steps the motor through every bin under the angle loop, in both forward and reverse sweeps. At each bin it measures the settled holding torque command (`current_sp`) and averages the two directions to cancel Coulomb friction.
- **Sinusoidal fit:** the raw table is DFT-analysed. Only the strongest N harmonics are kept (optionally restricted to multiples of the cogging fundamental). The table is then rebuilt from those sinusoids, which removes noise and gives a smooth feedforward.
- **Injection** adds the interpolated value to `current_sp` after `move()` and before `loopFOC()`. This works in `voltage`, `dc_current` and `foc_current` torque modes.
- **Persistence** stores the table in NVS via `Preferences`, with a CRC check.
- The injection path is `IRAM_ATTR`, branch-light, and free of trig calls.

---

### `AntiCogging.h`
```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

#ifndef ANTICOG_BINS
#define ANTICOG_BINS 512
#endif

class AntiCogging {
public:
  static constexpr uint16_t BINS = ANTICOG_BINS;
  static_assert((BINS & (BINS - 1)) == 0, "BINS must be a power of 2");

  struct Config {
    uint16_t settle_ms        = 30;    // wait after each step before sampling
    uint16_t samples_per_bin  = 64;    // loopFOC iterations averaged per bin
    uint8_t  harmonics_kept   = 12;    // sinusoidal components retained
    uint16_t cogging_period   = 0;     // e.g. LCM(slots, poles); 0 = any harmonic
    float    max_ff           = 1.0f;  // clamp on injected value (current_sp units)
    float    gain             = 1.0f;  // runtime scaling of feedforward
    float    max_vel_error    = 0.5f;  // rad/s; reject samples while still moving
  };

  explicit AntiCogging(BLDCMotor& motor);

  Config cfg;

  // Blocking calibration. Motor must be initFOC()'d and able to hold position.
  // Returns false on abort/failure.
  bool calibrate(Print* log = &Serial);

  // Call between motor.move() and motor.loopFOC().
  void IRAM_ATTR apply();

  float IRAM_ATTR lookup(float mech_angle) const;

  bool save(const char* ns = "anticog");
  bool load(const char* ns = "anticog");
  void clear();

  void enable(bool en) { enabled_ = en && valid_; }
  bool isEnabled() const { return enabled_; }
  bool isValid()   const { return valid_; }
  const float* table() const { return table_; }
  float lastInjection() const { return last_ff_; }
  void dump(Print& p) const;

private:
  BLDCMotor& motor_;
  float table_[BINS];
  bool  valid_   = false;
  bool  enabled_ = false;
  volatile float last_ff_ = 0.0f;

  bool sweep(int dir, float* acc, uint16_t* cnt, Print* log);
  void runFor(uint32_t ms);
  void fitSinusoids(const float* raw, Print* log);
  static uint32_t crc32(const uint8_t* d, size_t n);
  static inline uint16_t binOf(float a) {
    float x = a * (BINS / _2PI);
    int i = (int)x;
    return (uint16_t)(i & (BINS - 1));
  }
};
```

### `AntiCogging.cpp`
```cpp
#include "AntiCogging.h"
#include <Preferences.h>
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& m) : motor_(m) { clear(); }

void AntiCogging::clear() {
  memset(table_, 0, sizeof(table_));
  valid_ = enabled_ = false;
}

// ---------------------------------------------------------------- runtime
float IRAM_ATTR AntiCogging::lookup(float a) const {
  // wrap to [0, 2π)
  a = fmodf(a, _2PI);
  if (a < 0) a += _2PI;
  float x = a * (BINS / _2PI);
  int   i = (int)x;
  float f = x - (float)i;
  i &= (BINS - 1);
  int j = (i + 1) & (BINS - 1);
  return table_[i] + f * (table_[j] - table_[i]);
}

void IRAM_ATTR AntiCogging::apply() {
  if (!enabled_ || !motor_.sensor) { last_ff_ = 0; return; }
  float ff = cfg.gain * lookup(motor_.sensor->getMechanicalAngle());
  ff = _constrain(ff, -cfg.max_ff, cfg.max_ff);
  last_ff_ = ff;

  motor_.current_sp += ff;

  // In voltage mode, move() has already turned current_sp into voltage.q.
  if (motor_.torque_controller == TorqueControlType::voltage) {
    float dv = _isset(motor_.phase_resistance) ? ff * motor_.phase_resistance : ff;
    motor_.voltage.q = _constrain(motor_.voltage.q + dv,
                                  -motor_.voltage_limit, motor_.voltage_limit);
  }
  // dc_current / foc_current: loopFOC() consumes current_sp directly.
}

// ---------------------------------------------------------------- calibration
void AntiCogging::runFor(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    motor_.loopFOC();
    motor_.move();
  }
}

bool AntiCogging::sweep(int dir, float* acc, uint16_t* cnt, Print* log) {
  const float step  = _2PI / BINS;
  const float start = motor_.shaft_angle;

  for (uint16_t k = 0; k <= BINS; ++k) {          // +1 to close the loop
    motor_.target = start + dir * step * k;
    runFor(cfg.settle_ms);

    float sum = 0; uint16_t n = 0;
    for (uint16_t s = 0; s < cfg.samples_per_bin; ++s) {
      motor_.loopFOC();
      motor_.move();
      if (fabsf(motor_.shaft_velocity) < cfg.max_vel_error) {
        sum += motor_.current_sp;
        ++n;
      }
    }
    if (n) {
      uint16_t b = binOf(motor_.sensor->getMechanicalAngle());
      acc[b] += sum / n;
      cnt[b]++;
    }
    if ((k & 31) == 0) {
      vTaskDelay(1);                               // feed task watchdog
      if (log) log->printf("  dir %+d  %3u/%u\n", dir, k, BINS);
    }
    if (Serial.available() && Serial.read() == 'x') {
      if (log) log->println("AntiCog: aborted");
      return false;
    }
  }
  return true;
}

bool AntiCogging::calibrate(Print* log) {
  if (!motor_.sensor || motor_.motor_status != FOCMotorStatus::motor_ready) {
    if (log) log->println("AntiCog: motor not ready");
    return false;
  }

  bool was_enabled = enabled_;
  enabled_ = false;
  MotionControlType prev_mode = motor_.controller;
  float prev_target = motor_.target;
  motor_.controller = MotionControlType::angle;
  motor_.target = motor_.shaft_angle;
  runFor(500);

  float*    fwd  = (float*)   calloc(BINS, sizeof(float));
  float*    rev  = (float*)   calloc(BINS, sizeof(float));
  uint16_t* nf   = (uint16_t*)calloc(BINS, sizeof(uint16_t));
  uint16_t* nr   = (uint16_t*)calloc(BINS, sizeof(uint16_t));
  float*    raw  = (float*)   calloc(BINS, sizeof(float));
  bool ok = fwd && rev && nf && nr && raw;

  if (ok) {
    if (log) log->println("AntiCog: forward sweep");
    ok = sweep(+1, fwd, nf, log);
  }
  if (ok) {
    if (log) log->println("AntiCog: reverse sweep");
    ok = sweep(-1, rev, nr, log);
  }

  if (ok) {
    // Average directions (cancels friction), track holes.
    uint16_t holes = 0;
    for (uint16_t i = 0; i < BINS; ++i) {
      bool hf = nf[i], hr = nr[i];
      if (hf && hr)  raw[i] = 0.5f * (fwd[i] / nf[i] + rev[i] / nr[i]);
      else if (hf)   raw[i] = fwd[i] / nf[i];
      else if (hr)   raw[i] = rev[i] / nr[i];
      else         { raw[i] = NAN; ++holes; }
    }
    if (holes > BINS / 8) {
      if (log) log->printf("AntiCog: too many empty bins (%u)\n", holes);
      ok = false;
    } else {
      // Fill holes by circular linear interpolation.
      for (uint16_t i = 0; i < BINS; ++i) {
        if (!isnan(raw[i])) continue;
        int a = i, b = i;
        do a = (a - 1) & (BINS - 1); while (isnan(raw[a]));
        do b = (b + 1) & (BINS - 1); while (isnan(raw[b]));
        int da = (i - a) & (BINS - 1), db = (b - i) & (BINS - 1);
        raw[i] = raw[a] + (raw[b] - raw[a]) * (float)da / (float)(da + db);
      }
      // Remove DC (gravity/friction offset is not cogging).
      float mean = 0;
      for (uint16_t i = 0; i < BINS; ++i) mean += raw[i];
      mean /= BINS;
      for (uint16_t i = 0; i < BINS; ++i) raw[i] -= mean;

      fitSinusoids(raw, log);
      valid_ = true;
    }
  }

  free(fwd); free(rev); free(nf); free(nr); free(raw);

  motor_.controller = prev_mode;
  motor_.target = (prev_mode == MotionControlType::angle) ? motor_.shaft_angle
                                                          : prev_target;
  enabled_ = ok ? true : was_enabled && valid_;
  if (log) log->println(ok ? "AntiCog: calibration done" : "AntiCog: failed");
  return ok;
}

// Keep the strongest harmonics and rebuild the table from sinusoids.
void AntiCogging::fitSinusoids(const float* raw, Print* log) {
  const uint16_t H = BINS / 2;
  float* re  = (float*)calloc(H + 1, sizeof(float));
  float* im  = (float*)calloc(H + 1, sizeof(float));
  float* mag = (float*)calloc(H + 1, sizeof(float));

  // Plain DFT (one-time, 512 x 257 MACs, roughly 30 ms on S3).
  for (uint16_t k = 1; k <= H; ++k) {
    if (cfg.cogging_period && (k % cfg.cogging_period)) continue;
    float sr = 0, si = 0;
    for (uint16_t n = 0; n < BINS; ++n) {
      float ph = _2PI * k * n / BINS;
      sr += raw[n] * cosf(ph);
      si -= raw[n] * sinf(ph);
    }
    float scale = (k == H) ? 1.0f / BINS : 2.0f / BINS;
    re[k] = sr * scale; im[k] = si * scale;
    mag[k] = sqrtf(re[k] * re[k] + im[k] * im[k]);
  }

  // Select top-N magnitudes.
  uint8_t N = cfg.harmonics_kept;
  uint16_t keep[64]; if (N > 64) N = 64;
  uint8_t kept = 0;
  for (uint8_t m = 0; m < N; ++m) {
    uint16_t best = 0; float bm = 0;
    for (uint16_t k = 1; k <= H; ++k) if (mag[k] > bm) { bm = mag[k]; best = k; }
    if (!best || bm <= 0) break;
    keep[kept++] = best;
    if (log) log->printf("  H%-3u  amp %.4f  phase %.3f\n",
                         best, bm, atan2f(im[best], re[best]));
    mag[best] = 0;
  }

  // Reconstruct.
  for (uint16_t n = 0; n < BINS; ++n) {
    float v = 0;
    for (uint8_t m = 0; m < kept; ++m) {
      uint16_t k = keep[m];
      float ph = _2PI * k * n / BINS;
      v += re[k] * cosf(ph) - im[k] * sinf(ph);
    }
    table_[n] = v;
  }
  free(re); free(im); free(mag);
}

// ---------------------------------------------------------------- storage
uint32_t AntiCogging::crc32(const uint8_t* d, size_t n) {
  uint32_t c