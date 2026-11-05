I don't have context for what "Cycle 27" refers to. It may be an internal project milestone or a previous conversation I can't see. Below is a complete, self-contained implementation based on standard SimpleFOC 2.3.x APIs. If Cycle 27 has specific requirements (units, storage format, harmonic set, etc.), share them and I'll adapt it.

## Design overview

1. **Calibration (blocking):** The motor is held in closed-loop angle mode at 512 evenly spaced mechanical positions. At each point, the torque command the controller needs to hold position (`motor.current_sp`) is averaged.
2. **Bidirectional sweep:** A forward and a reverse sweep are averaged. This cancels Coulomb friction and loop lag, which flip sign with direction, while keeping cogging, which is position-dependent.
3. **Sinusoidal (Fourier) fit:** A DFT of the 512 samples is taken. The DC term (load/gravity bias) is dropped, and the table is rebuilt from the first *K* harmonics, optionally removing harmonics below a magnitude threshold. This removes sampling noise and gives a smooth, periodic table.
4. **Feedforward:** At runtime, the table is read with linear interpolation on the mechanical angle. The result is added to the torque command (voltage or current, matching the motor's torque mode).
5. **Persistence:** The table is stored in ESP32 NVS (`Preferences`) with a header and CRC32. It is only valid with an absolute sensor, or with an incremental sensor whose index or zero is identical every boot.

---

### `CoggingCompensator.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class CoggingCompensator {
public:
  static constexpr uint16_t TABLE_SIZE   = 512;
  static constexpr uint32_t NVS_MAGIC    = 0xC0661A27;
  static constexpr uint16_t NVS_VERSION  = 1;

  struct Config {
    uint16_t settle_ms        = 60;     // wait after each step before sampling
    uint16_t samples_per_pt   = 40;     // averaged samples per position
    uint16_t sample_period_us = 250;    // spacing between samples
    uint16_t max_harmonic     = 64;     // highest harmonic kept in fit (<= 255)
    float    harmonic_min_amp = 0.0f;   // drop harmonics below this amplitude (V or A)
    float    max_ff           = 1.0f;   // clamp on feedforward magnitude (V or A)
    float    gain             = 1.0f;   // runtime scaling of compensation
    float    cal_velocity_lim = 2.0f;   // rad/s limit while stepping
    float    max_hold_effort  = 3.0f;   // abort if hold torque exceeds this
    const char* nvs_namespace = "cogcomp";
  };

  explicit CoggingCompensator(BLDCMotor& motor);

  void   setConfig(const Config& c) { cfg_ = c; }
  Config& config() { return cfg_; }

  // Blocking calibration. Motor must be initialised (init + initFOC) and
  // tuned for stiff angle control. Returns true on success.
  bool calibrate(Print* log = &Serial);

  // Runtime feedforward (V or A, matching motor.torque_controller).
  float IRAM_ATTR feedforward(float mech_angle) const;
  float IRAM_ATTR feedforward() const;      // reads motor.sensor directly

  // Convenience: torque-mode move with compensation injected.
  void move(float torque_target);

  void enable(bool en)   { enabled_ = en && valid_; }
  bool isEnabled() const { return enabled_; }
  bool isValid()   const { return valid_; }

  bool save();
  bool load();
  void clear();

  void dumpTable(Print& out) const;
  const float* table() const { return table_; }

private:
  bool  sweep(int dir, float* out, Print* log);
  float holdAndMeasure(float target_angle, bool& ok);
  void  fourierFit(const float* raw, float* fitted, Print* log);
  void  runLoopFor(uint32_t us);
  static uint32_t crc32(const uint8_t* d, size_t n);

  BLDCMotor& motor_;
  Config     cfg_;
  float      table_[TABLE_SIZE];
  float      start_angle_ = 0.0f;   // shaft_angle at start (multi-turn offset)
  volatile bool valid_   = false;
  volatile bool enabled_ = false;
};
```

### `CoggingCompensator.cpp`

```cpp
#include "CoggingCompensator.h"
#include <math.h>

static constexpr float TWO_PI_F = 6.28318530718f;

struct NvsHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t crc;
};

CoggingCompensator::CoggingCompensator(BLDCMotor& motor) : motor_(motor) {
  memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------
float IRAM_ATTR CoggingCompensator::feedforward(float a) const {
  if (!enabled_) return 0.0f;
  // wrap to [0, 2π)
  a = fmodf(a, TWO_PI_F);
  if (a < 0) a += TWO_PI_F;

  const float pos  = a * (TABLE_SIZE / TWO_PI_F);
  uint32_t    i0   = (uint32_t)pos;
  const float frac = pos - (float)i0;
  i0 &= (TABLE_SIZE - 1);                       // TABLE_SIZE is power of 2
  const uint32_t i1 = (i0 + 1) & (TABLE_SIZE - 1);

  float ff = (table_[i0] + frac * (table_[i1] - table_[i0])) * cfg_.gain;
  return _constrain(ff, -cfg_.max_ff, cfg_.max_ff);
}

float IRAM_ATTR CoggingCompensator::feedforward() const {
  if (!motor_.sensor) return 0.0f;
  return feedforward(motor_.sensor->getMechanicalAngle());
}

void CoggingCompensator::move(float torque_target) {
  // Intended for MotionControlType::torque. For velocity/angle modes, add
  // feedforward() to the PID output in a custom loop instead (see notes).
  motor_.move(torque_target + feedforward());
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------
void CoggingCompensator::runLoopFor(uint32_t us) {
  const uint32_t t0 = micros();
  uint32_t lastYield = millis();
  while ((uint32_t)(micros() - t0) < us) {
    motor_.loopFOC();
    motor_.move();
    if (millis() - lastYield > 200) {     // keep task watchdog happy
      vTaskDelay(1);
      lastYield = millis();
    }
  }
}

float CoggingCompensator::holdAndMeasure(float target, bool& ok) {
  motor_.target = target;
  runLoopFor((uint32_t)cfg_.settle_ms * 1000UL);

  double acc = 0.0;
  for (uint16_t s = 0; s < cfg_.samples_per_pt; ++s) {
    runLoopFor(cfg_.sample_period_us);
    const float effort = motor_.current_sp;   // torque cmd (V or A)
    if (fabsf(effort) > cfg_.max_hold_effort) { ok = false; return 0.0f; }
    acc += effort;
  }
  ok = true;
  return (float)(acc / cfg_.samples_per_pt);
}

bool CoggingCompensator::sweep(int dir, float* out, Print* log) {
  const float step = TWO_PI_F / TABLE_SIZE;
  // Pre-roll a few steps so backlash/friction direction is established.
  for (int k = 0; k < 8; ++k) {
    motor_.target = start_angle_ + dir * (k - 8) * step + (dir < 0 ? TWO_PI_F : 0);
    runLoopFor(cfg_.settle_ms * 1000UL);
  }

  for (uint16_t n = 0; n < TABLE_SIZE; ++n) {
    const uint16_t idx = (dir > 0) ? n : (TABLE_SIZE - 1 - n);
    const float target = start_angle_ + idx * step;
    bool ok;
    const float effort = holdAndMeasure(target, ok);
    if (!ok) {
      if (log) log->printf("[cog] abort: effort limit at idx %u\n", idx);
      return false;
    }
    // Index by actual measured mechanical angle bin (robust to small offsets)
    float m = motor_.sensor->getMechanicalAngle();
    uint16_t bin = (uint16_t)(m * TABLE_SIZE / TWO_PI_F) & (TABLE_SIZE - 1);
    (void)bin;  // logged only; idx is used since loop holds position tightly
    out[idx] = effort;

    if (log && (n % 64 == 0))
      log->printf("[cog] %s %3u/%u  eff=% .4f  bin=%u\n",
                  dir > 0 ? "fwd" : "rev", n, TABLE_SIZE, effort, bin);
  }
  return true;
}

void CoggingCompensator::fourierFit(const float* raw, float* fitted, Print* log) {
  const uint16_t N = TABLE_SIZE;
  const uint16_t K = min<uint16_t>(cfg_.max_harmonic, N / 2 - 1);

  // Precompute twiddle (cos/sin) table once: 2*512 floats = 4 KB
  static float c[TABLE_SIZE], s[TABLE_SIZE];
  for (uint16_t i = 0; i < N; ++i) {
    c[i] = cosf(TWO_PI_F * i / N);
    s[i] = sinf(TWO_PI_F * i / N);
  }

  for (uint16_t i = 0; i < N; ++i) fitted[i] = 0.0f;  // DC intentionally dropped

  float peakAmp = 0; uint16_t peakK = 0;
  for (uint16_t k = 1; k <= K; ++k) {
    double a = 0, b = 0;
    for (uint16_t i = 0; i < N; ++i) {
      const uint16_t p = (uint32_t)(k * i) & (N - 1);
      a += raw[i] * c[p];
      b += raw[i] * s[p];
    }
    a *= 2.0 / N;  b *= 2.0 / N;
    const float amp = sqrtf((float)(a * a + b * b));
    if (amp > peakAmp) { peakAmp = amp; peakK = k; }
    if (amp < cfg_.harmonic_min_amp) continue;

    for (uint16_t i = 0; i < N; ++i) {
      const uint16_t p = (uint32_t)(k * i) & (N - 1);
      fitted[i] += (float)(a * c[p] + b * s[p]);
    }
  }
  if (log) log->printf("[cog] fit: K=%u, dominant harmonic %u (amp %.4f)\n",
                       K, peakK, peakAmp);
}

bool CoggingCompensator::calibrate(Print* log) {
  if (!motor_.sensor) { if (log) log->println("[cog] no sensor"); return false; }

  const bool               prevEn   = enabled_;
  const MotionControlType  prevCtl  = motor_.controller;
  const float              prevVlim = motor_.velocity_limit;
  enabled_ = false;  // never calibrate with compensation active

  motor_.controller     = MotionControlType::angle;
  motor_.velocity_limit = cfg_.cal_velocity_lim;
  motor_.enable();

  runLoopFor(200000);
  // Align start so table index 0 == mechanical angle 0 on current turn.
  const float mech = motor_.sensor->getMechanicalAngle();
  start_angle_ = motor_.shaft_angle - mech * (motor_.sensor_direction == Direction::CCW ? 1 : 1);

  // Heap-allocate scratch to keep stack small (ESP32 loop task = 8 KB)
  float* fwd = (float*)heap_caps_malloc(TABLE_SIZE * sizeof(float), MALLOC_CAP_8BIT);
  float* rev = (float*)heap_caps_malloc(TABLE_SIZE * sizeof(float), MALLOC_CAP_8BIT);
  bool ok = fwd && rev;

  if (ok && log) log->println("[cog] forward sweep...");
  if (ok) ok = sweep(+1, fwd, log);
  if (ok && log) log->println("[cog] reverse sweep...");
  if (ok) ok = sweep(-1, rev, log);

  if (ok) {
    double fr = 0;
    for (uint16_t i = 0; i < TABLE_SIZE; ++i) {
      fr += 0.5 * (fwd[i] - rev[i]);           // friction estimate
      fwd[i] = 0.5f * (fwd[i] + rev[i]);       // friction-cancelled
    }
    if (log) log->printf("[cog] mean friction ~ %.4f\n", fr / TABLE_SIZE);

    // The controller output that *holds* position equals the cogging torque
    // it fights; feeding it forward pre-cancels the cogging.
    float* fitted = rev;  // reuse buffer
    fourierFit(fwd, fitted, log);

    enabled_ = false;
    memcpy(table_, fitted, sizeof(table_));
    valid_ = true;
  }

  if (fwd) free(fwd);
  if (rev) free(rev);

  motor_.target         = motor_.shaft_angle;   // hold where we are
  motor_.veloc