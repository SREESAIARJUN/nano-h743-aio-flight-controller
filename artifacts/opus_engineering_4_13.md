# SimpleFOC Anti-Cogging Module for ESP32-S3

I don't know what "Cycle 4" refers to in your project, such as a prior spec, ticket, or iteration. This is a self-contained implementation based on the most common design. Tell me if your Cycle 4 spec differs (table units, storage backend, injection point) and I'll adapt it.

## Design Summary

| Aspect | Approach |
|---|---|
| Table | 512 bins over one mechanical revolution. Units are the same as your torque command (V for voltage mode, A for current modes). |
| Calibration | Closed-loop position hold at each bin, using a stiff PI loop. It records the average torque command needed to hold, which equals minus the cogging torque. |
| Friction rejection | Runs a forward pass and a reverse pass, then averages them. Stiction and friction cancel. The DC term is removed. |
| "Sinusoidal" fit | Runs a DFT on the raw table and keeps the K strongest harmonics. It then rebuilds the table as a pure sum of sinusoids, which removes noise and quantization artifacts. |
| Injection | Linear interpolation with a velocity-dependent phase lead and a velocity fade-out. It runs from IRAM in O(1) time. |
| Persistence | ESP32 NVS (`Preferences`) with a magic number, version, signature, and CRC32. It refuses to load a table if the motor, sensor, or alignment changed. |
| Integration | SimpleFOC runs in `MotionControlType::torque`. Your outer loop adds the feedforward to its own command, so no library patch is needed. |

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
  static constexpr uint16_t TABLE_SIZE   = 512;
  static constexpr uint32_t MAGIC        = 0x41434F47; // 'ACOG'
  static constexpr uint16_t VERSION      = 4;
  static constexpr uint16_t MAX_HARMONIC = TABLE_SIZE / 2 - 1;

  struct Config {
    // --- calibration hold loop (output units = torque command units) ---
    float hold_P            = 8.0f;    // [cmd / rad]
    float hold_I            = 300.0f;  // [cmd / rad / s]
    float hold_D            = 0.0f;
    float hold_limit        = 3.0f;    // max |cmd| during calibration (V or A)
    float hold_ramp         = 1000.0f; // output ramp [cmd/s]

    uint16_t move_ms        = 4;       // time to ramp setpoint to next bin
    uint16_t settle_ms      = 40;      // settle time per bin
    uint16_t samples        = 64;      // averaged FOC iterations per bin
    uint16_t warmup_bins    = 8;       // bins discarded at start of each pass

    // --- sinusoidal (Fourier) post-processing ---
    bool     fourier_fit    = true;
    uint16_t keep_harmonics = 12;      // strongest N harmonics retained
    uint16_t max_harmonic   = 128;     // search range (<= MAX_HARMONIC)

    // --- runtime injection ---
    float gain              = 1.0f;    // global feedforward scale
    float latency_s         = 150e-6f; // phase-lead: angle += vel * latency
    float fade_start_rad_s  = 30.0f;   // full compensation below this speed
    float fade_end_rad_s    = 120.0f;  // zero compensation above this speed
  };

  enum class Status : uint8_t {
    OK = 0, NOT_INITIALIZED, BAD_MOTOR_STATE, ABORTED,
    TRACKING_ERROR, NVS_ERROR, NVS_NOT_FOUND, NVS_CORRUPT, SIGNATURE_MISMATCH
  };

  explicit AntiCogging(FOCMotor& motor, const Config& cfg = Config());

  // Calibration (blocking, roughly 2 * 512 * (move+settle) ms, ~45 s by default)
  Status calibrate(Print* log = nullptr);
  void   requestAbort() { abort_ = true; }

  // Runtime
  float IRAM_ATTR feedforward();                      // uses motor.shaft_angle/velocity
  float IRAM_ATTR feedforward(float angle, float vel);
  float IRAM_ATTR compensate(float torque_cmd) { return torque_cmd + feedforward(); }

  void enable(bool en) { enabled_ = en && valid_; }
  bool isEnabled() const { return enabled_; }
  bool isValid()   const { return valid_; }

  // Persistence
  Status save(const char* ns = "acog");
  Status load(const char* ns = "acog");
  Status erase(const char* ns = "acog");

  // Diagnostics
  const float* table() const { return table_; }
  float peakToPeak() const;
  float rmsTrackingError() const { return rms_err_; }
  void  dumpCSV(Print& out) const;

  Config cfg;

private:
  struct __attribute__((packed)) Blob {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    int32_t  pole_pairs;
    int8_t   sensor_dir;
    int8_t   torque_ctrl;
    float    zero_elec;
    float    sensor_offset;
    float    table[TABLE_SIZE];
    uint32_t crc;
  };

  FOCMotor& motor_;
  alignas(4) float table_[TABLE_SIZE];
  float rms_err_ = 0.0f;
  volatile bool abort_ = false;
  bool valid_ = false, enabled_ = false;

  float shaftAngle();
  bool  runPass(int dir, float start, float* out, uint16_t* hits, float& err2, uint32_t& nerr, Print* log);
  void  holdFor(PIDController& pid, float target, uint32_t ms, float* acc, uint16_t n, float& err2, uint32_t& nerr);
  void  fourierFit();
  static inline float wrap2pi(float a) { a = fmodf(a, _2PI); return a < 0 ? a + _2PI : a; }
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include "esp_rom_crc.h"

static constexpr float BIN = _2PI / AntiCogging::TABLE_SIZE;

AntiCogging::AntiCogging(FOCMotor& m, const Config& c) : cfg(c), motor_(m) {
  memset(table_, 0, sizeof(table_));
}

float AntiCogging::shaftAngle() {
  // Same convention as motor.shaft_angle (direction + offset applied)
  return motor_.shaftAngle();
}

// ---------------------------------------------------------------------------
// Hold the rotor at 'target' for 'ms', optionally averaging the command.
// ---------------------------------------------------------------------------
void AntiCogging::holdFor(PIDController& pid, float target, uint32_t ms,
                          float* acc, uint16_t n, float& err2, uint32_t& nerr) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    motor_.loopFOC();
    float e = target - shaftAngle();
    motor_.move(pid(e));
  }
  if (acc) {
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
      motor_.loopFOC();
      float e = target - shaftAngle();
      float u = pid(e);
      motor_.move(u);
      sum  += u;
      err2 += e * e; nerr++;
    }
    *acc = sum / n;
  }
}

// ---------------------------------------------------------------------------
// One full mechanical revolution in direction 'dir' (+1 / -1).
// ---------------------------------------------------------------------------
bool AntiCogging::runPass(int dir, float start, float* out, uint16_t* hits,
                          float& err2, uint32_t& nerr, Print* log) {
  PIDController pid(cfg.hold_P, cfg.hold_I, cfg.hold_D, cfg.hold_ramp, cfg.hold_limit);
  float target = start;
  holdFor(pid, target, 200, nullptr, 0, err2, nerr);   // initial lock

  const int total = TABLE_SIZE + cfg.warmup_bins;
  for (int k = 0; k < total; k++) {
    if (abort_) return false;

    // Ramp setpoint smoothly to the next bin (avoids jerks that excite the I-term)
    float next = target + dir * BIN;
    uint32_t t0 = millis();
    while (true) {
      float f = (millis() - t0) / (float)max<uint16_t>(cfg.move_ms, 1);
      if (f >= 1.0f) break;
      motor_.loopFOC();
      motor_.move(pid(target + dir * BIN * f - shaftAngle()));
    