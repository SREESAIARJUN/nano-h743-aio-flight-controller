# AntiCogging Module for SimpleFOC on ESP32-S3

I don't have the earlier "cycles" of your project, so I built this from a clean spec. It is a self-contained module that does the following:

1. **Calibrates** a 512-bin cogging map. It sweeps the rotor through one mechanical revolution in closed-loop angle mode, in both directions, and records the steady-state torque command needed to hold each position.
2. **Fits a sinusoidal model.** It runs a DFT on the raw map, keeps the dominant harmonics (or all harmonics up to N), and rebuilds a clean, noise-free 512-point table.
3. **Persists** the table to ESP32 NVS. The record includes a magic number, a version, CRC32, and sensor/alignment metadata, so a stale table is rejected.
4. **Injects feedforward** after `motor.move()`. It adds to `current_sp` in current modes or to `voltage.q` in voltage mode, with interpolation, a gain, velocity fade-out, and clamping. This works in torque, velocity, and angle motion modes.

If your earlier cycles defined specific interfaces, such as a different storage backend, a fixed-point table, or ISR-driven injection, tell me and I'll adapt it.

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
  static constexpr uint16_t MAX_HARMONICS = 64;
  static constexpr uint32_t STORE_MAGIC   = 0xAC09C0DE;
  static constexpr uint16_t STORE_VERSION = 9;

  struct Config {
    uint32_t settle_ms        = 60;     // hold time before sampling each bin
    uint16_t samples_per_bin  = 64;     // FOC iterations averaged per bin
    uint16_t harmonics_keep   = 24;     // number of sinusoidal terms in model
    bool     select_dominant  = true;   // true: keep largest-K, false: keep 1..K
    float    gain             = 1.0f;   // runtime FF gain (0..~1.2)
    float    fade_start_rad_s = 30.0f;  // FF begins fading above this speed
    float    fade_end_rad_s   = 80.0f;  // FF fully off above this speed
    float    max_ff_fraction  = 0.5f;   // FF clamp as fraction of current/voltage limit
    float    cal_P_angle      = 40.0f;  // stiff angle loop during calibration
    float    cal_velocity_lim = 4.0f;   // rad/s during sweep moves
  };

  enum class Status : uint8_t { NotCalibrated, Calibrating, Ready, Error };

  explicit AntiCogging(BLDCMotor& motor, const char* nvs_namespace = "acog");

  Config config;

  // --- lifecycle ---------------------------------------------------------
  bool  calibrate(Print* log = &Serial);   // blocking, ~1-2 min
  bool  load();                             // load from NVS, validates
  bool  save();                             // persist to NVS
  void  clear();                            // erase NVS + disable

  // --- runtime -----------------------------------------------------------
  // Call right AFTER motor.move() every loop iteration.
  void  apply();
  // Pure lookup (torque-command units) for a mechanical angle [rad].
  float lookup(float mech_angle) const;

  void   enable(bool en) { enabled_ = en && status_ == Status::Ready; }
  bool   enabled() const { return enabled_; }
  Status status()  const { return status_; }
  float  lastInjection() const { return last_ff_; }
  const float* table() const { return table_; }
  void   dumpTable(Print& out) const;

private:
  struct __attribute__((packed)) Header {
    uint32_t magic;
    uint16_t version;
    uint16_t n_bins;
    uint8_t  torque_mode;      // TorqueControlType
    int8_t   sensor_dir;       // Direction
    float    zero_elec_angle;
    uint32_t crc;
  };

  BLDCMotor&  motor_;
  const char* ns_;
  Status      status_  = Status::NotCalibrated;
  bool        enabled_ = false;
  float       last_ff_ = 0.0f;
  uint8_t     torque_mode_ = 0;

  float table_[N_BINS];

  float  torqueCommand() const;
  float  mechAngle() const;
  void   runFOC(uint32_t iterations);
  bool   sweep(float* acc, uint16_t* cnt, int dir, Print* log);
  void   fitSinusoids(const float* raw);
  static uint32_t crc32(const uint8_t* d, size_t n, uint32_t crc = 0);
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& motor, const char* ns)
  : motor_(motor), ns_(ns) {
  memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
float AntiCogging::torqueCommand() const {
  // In voltage mode move() writes voltage.q; in current modes it writes current_sp.
  return (motor_.torque_controller == TorqueControlType::voltage)
           ? motor_.voltage.q
           : motor_.current_sp;
}

float AntiCogging::mechAngle() const {
  float a = motor_.sensor->getMechanicalAngle();   // 0..2PI (sensor frame)
  if (motor_.sensor_direction == Direction::CCW) a = _2PI - a;
  a = fmodf(a, _2PI);
  return a < 0 ? a + _2PI : a;
}

void AntiCogging::runFOC(uint32_t iterations) {
  for (uint32_t i = 0; i < iterations; ++i) {
    motor_.loopFOC();
    motor_.move();
  }
}

// ---------------------------------------------------------------------------
// calibration
// ---------------------------------------------------------------------------
bool AntiCogging::sweep(float* acc, uint16_t* cnt, int dir, Print* log) {
  const float step  = _2PI / N_BINS;
  const float start = motor_.shaft_angle;

  // Pre-travel one bin opposite to sweep so backlash/stiction is consistently loaded
  motor_.target = start - dir * step;
  uint32_t t0 = millis();
  while (millis() - t0 < config.settle_ms * 3) runFOC(1);

  for (uint16_t i = 0; i <= N_BINS; ++i) {
    motor_.target = start + dir * step * i;

    t0 = millis();
    while (millis() - t0 < config.settle_ms) runFOC(1);

    // Average the holding torque command
    float sum = 0.0f, angSin = 0.0f, angCos = 0.0f;
    for (uint16_t s = 0; s < config.samples_per_bin; ++s) {
      runFOC(1);
      sum += torqueCommand();
      float a = mechAngle();
      angSin += sinf(a); angCos += cosf(a);   // circular mean avoids 0/2PI wrap
    }
    float tq = sum / config.samples_per_bin;
    float a  = atan2f(angSin, angCos);
    if (a < 0) a += _2PI;

    uint16_t bin = (uint16_t)lroundf(a / step) % N_BINS;
    acc[bin] += tq;
    cnt[bin] += 1;

    // Sanity: following error must stay small or the table is garbage
    if (fabsf(motor_.shaft_angle - motor_.target) > 4.0f * step * 8) {
      if (log) log->printf("[ACOG] following error too large at bin %u\n", i);
      return false;
    }
    if ((i & 31) == 0) {
      if (log) log->printf("[ACOG] dir %+d  %3u/%u  tq=%.4f\n", dir, i, N_BINS, tq);
      vTaskDelay(1);  // feed watchdog / let other tasks run
    }
  }
  return true;
}

bool AntiCogging::calibrate(Print* log) {
  if (!motor_.sensor || !motor_.enabled) {
    status_ = Status::Error;
    if (log) log->println("[ACOG] motor not initialised / enabled");
    return false;
  }

  status_  = Status::Calibrating;
  enabled_ = false;

  // Save and override controller settings
  MotionControlType prevCtrl = motor_.controller;
  float prevP     = motor_.P_angle.P;
  float prevVLim  = motor_.velocity_limit;
  float prevTgt   = motor_.target;

  motor_.controller     = MotionControlType::angle;
  motor_.P_angle.P      = config.cal_P_angle;
  motor_.velocity_limit = config.cal_velocity_lim;
  torque_mode_          = (uint8_t)motor_.torque_controller;

  // Heap buffers (4 KB) – keep stack small on loopTask
  float*    acc = (float*)   heap_caps_calloc(N_BINS, sizeof(float),    MALLOC_CAP_8BIT);
  uint16_t* cnt = (uint16_t*)heap_caps_calloc(N_BINS, sizeof(uint16_t), MALLOC_CAP_8BIT);
  if (!acc || !cnt) {
    free(acc); free(cnt);
    status_ = Status::Error;
    return false;
  }

  if (log) log->println("[ACOG] calibration start (forward)");
  bool ok = sweep(acc, cnt, +1, log);
  if (ok) {
    if (log) log->println("[ACOG] calibration (reverse)");
    ok = sweep(acc, cnt, -1, log);   // averaging both directions cancels friction
  }

  if (ok) {
    // Fill empty bins by circular linear interpolation
    float* raw = acc;
    uint16_t filled = 0;
    for (uint16_t i = 0; i < N_BINS; ++i)
      if (cnt[i]) { raw[i] /= cnt[i]; ++filled; }

    if (filled < N_BINS / 2) {
      ok = false;
      if (log) log->printf("[ACOG] only %u/%u bins hit\n", filled, N_BINS);
    } else {
      for (uint16_t i = 0; i < N_BINS; ++i) {
        if (cnt[i]) continue;
        int l = i, r = i, dl = 0, dr = 0;
        do { l = (l - 1 + N_BINS) % N_BINS; ++dl; } while (!cnt[l]);
        do { r = (r + 1) % N_BINS;          ++dr; } while (!cnt[r]);
        raw[i] = raw[l] + (raw[r] - raw[l]) * (float)dl / (dl + dr);
      }
      fitSinusoids(raw);
    }
  }

  free(acc); free(cnt);

  // Restore controller
  motor_.controller     = prevCtrl;
  motor_.P_angle.P      = prevP;
  motor_.velocity_limit = prevVLim;
  motor_.target         = (prevCtrl == MotionControlType::angle) ? motor_.shaft_angle : prevTgt;

  status_ = ok ? Status::Ready : Status::Error;
  if (ok) {
    save();
    enabled_ = true;
    if (log) log->println("[ACOG] calibration complete, saved to NVS");
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Sinusoidal model: DFT → keep K harmonics → reconstruct
// ---------------------------------------------------------------------------
void AntiCogging::fitSinusoids(const float* raw) {
  const uint16_t H = min<uint16_t>(N_BINS / 2 - 1, 255);
  float* a = (float*)malloc((H + 1) * sizeof(float));
  float* b = (float*)malloc((H + 1) * sizeof(float));
  float* mag = (float*)malloc((H + 1) * sizeof(float));

  // Precompute twiddles once
  static float cosT[N_BINS], s