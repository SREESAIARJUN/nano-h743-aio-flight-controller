I don't know what "Cycle 7" refers to in your project, such as a design spec or iteration plan. Below is a complete, self-contained implementation based on standard SimpleFOC 2.3.x APIs and the ESP32-S3 Arduino core. If Cycle 7 has specific requirements (table format, harmonic count, storage layout, injection point), tell me and I'll adapt it.

## Design overview

**Table and indexing**
- The table has 512 entries spanning one mechanical revolution.
- It is indexed by the sensor's absolute mechanical angle (`sensor->getMechanicalAngle()`). This needs an absolute sensor such as an AS5600, AS5047, or MT6701.
- Values between entries are found by linear interpolation with wrap-around.

**Calibration**
1. The motor is held at each of the 512 positions using SimpleFOC's own angle loop.
2. After it settles, the holding effort (`current_sp` or `voltage.q`) is averaged over many FOC cycles.
3. A forward pass and a reverse pass are averaged. This cancels Coulomb friction and PID hysteresis.
4. The DC component (load and friction bias) is removed.

**Sinusoidal fit**
- A DFT is taken of the raw table.
- The K strongest harmonics are kept, and the table is rebuilt from them. This removes sampling noise while keeping the true cogging spectrum, which is dominated by the LCM of slots and poles.

**Feedforward injection**
- The compensation is applied after `motor.move()` and before the next `motor.loopFOC()`.
  - In voltage torque mode it adds to `motor.voltage.q`.
  - In current modes it adds to `motor.current_sp`.
- This works in torque, velocity, and angle modes without patching the library.
- A velocity fade-out reduces the compensation at high speed, where cogging is filtered by inertia and phase lag would make the compensation harmful.

**Persistence**
- The table is stored in NVS (`Preferences`) with a magic number, version, pole pairs, torque mode, and a CRC32.
- An invalid or mismatched blob is rejected.

---

### `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

class AntiCogging {
public:
  static constexpr uint16_t TABLE_SIZE   = 512;
  static constexpr uint16_t TABLE_MASK   = TABLE_SIZE - 1;   // power of two
  static constexpr uint16_t MAX_HARMONICS = TABLE_SIZE / 2;

  struct Config {
    // Calibration
    float    settle_time_ms      = 40.0f;   // time to settle at each point
    uint16_t samples_per_point   = 200;     // FOC cycles averaged per point
    float    approach_velocity   = 2.0f;    // rad/s when moving between points
    bool     bidirectional       = true;    // forward + reverse pass
    // Angle-hold loop gains used during calibration (stiff hold)
    float    cal_P_angle         = 20.0f;
    float    cal_vel_P           = 0.2f;
    float    cal_vel_I           = 10.0f;
    float    cal_vel_D           = 0.0f;
    float    cal_vel_LPF_Tf      = 0.01f;
    // Harmonic fit
    bool     harmonic_fit        = true;
    uint16_t harmonics_kept      = 16;      // strongest K harmonics retained
    // Runtime
    float    gain                = 1.0f;    // global compensation gain
    float    fade_start_velocity = 20.0f;   // rad/s, full compensation below
    float    fade_end_velocity   = 60.0f;   // rad/s, zero compensation above
    float    max_compensation    = 0.0f;    // 0 = auto (motor limits)
  };

  enum class Status : uint8_t { NotCalibrated, Calibrating, Ready, Error };

  explicit AntiCogging(BLDCMotor& motor);

  void   setConfig(const Config& c) { cfg_ = c; }
  Config& config() { return cfg_; }

  // Blocking calibration. Motor must be initFOC()'d with an absolute sensor.
  // progress callback receives 0..100 (may be nullptr).
  bool calibrate(void (*progress)(uint8_t pct) = nullptr);

  // Call once per control loop, immediately AFTER motor.move()
  // and before the next motor.loopFOC().
  void IRAM_ATTR apply();

  // Lookup only (no side effects). angle = mechanical angle [0, 2π).
  float IRAM_ATTR lookup(float mech_angle) const;

  void enable(bool en) { enabled_ = en; }
  bool enabled() const { return enabled_; }
  Status status() const { return status_; }
  float lastCompensation() const { return last_comp_; }

  // Persistence (NVS)
  bool save(const char* ns = "anticog");
  bool load(const char* ns = "anticog");
  bool erase(const char* ns = "anticog");

  // Diagnostics
  void   dumpTable(Stream& s) const;
  void   dumpSpectrum(Stream& s, uint16_t n = 32) const;
  const float* table() const { return table_; }

private:
  struct __attribute__((packed)) BlobHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t  pole_pairs;
    uint8_t  torque_mode;
    uint16_t reserved;
    uint32_t crc;
  };
  static constexpr uint32_t MAGIC   = 0x41434F47; // 'ACOG'
  static constexpr uint16_t VERSION = 1;

  BLDCMotor& m_;
  Config cfg_;
  Status status_ = Status::NotCalibrated;
  bool   enabled_ = false;
  float  last_comp_ = 0.0f;

  alignas(16) float table_[TABLE_SIZE];
  float spectrumMag_[MAX_HARMONICS + 1];

  bool  measurePass(float* out, bool forward, uint16_t& done, uint16_t total,
                    void (*progress)(uint8_t));
  float holdAndMeasure(float target_angle);
  void  runFor(float ms, float target);
  void  moveTo(float from, float to);
  float effort() const;
  void  removeMean(float* t);
  void  harmonicFit(float* t);
  static uint32_t crc32(const uint8_t* d, size_t n, uint32_t crc = 0);
  float limit() const;
};
```

### `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <Preferences.h>
#include <math.h>

AntiCogging::AntiCogging(BLDCMotor& motor) : m_(motor) {
  memset(table_, 0, sizeof(table_));
  memset(spectrumMag_, 0, sizeof(spectrumMag_));
}

// ---------------------------------------------------------------- helpers
float AntiCogging::effort() const {
  // Quantity that actually produces torque in the active torque mode
  return (m_.torque_controller == TorqueControlType::voltage)
           ? m_.voltage.q
           : m_.current_sp;
}

float AntiCogging::limit() const {
  if (cfg_.max_compensation > 0.0f) return cfg_.max_compensation;
  return (m_.torque_controller == TorqueControlType::voltage)
           ? m_.voltage_limit * 0.5f
           : m_.current_limit * 0.5f;
}

void AntiCogging::runFor(float ms, float target) {
  uint32_t t0 = micros();
  uint32_t dur = (uint32_t)(ms * 1000.0f);
  uint32_t n = 0;
  while ((micros() - t0) < dur) {
    m_.loopFOC();
    m_.move(target);
    if ((++n & 0x3FF) == 0) yield();     // keep RTOS/WDT happy
  }
}

void AntiCogging::moveTo(float from, float to) {
  // Ramp the angle setpoint to avoid kicks
  float dist = to - from;
  float dur_s = fabsf(dist) / fmaxf(cfg_.approach_velocity, 0.01f);
  uint32_t t0 = micros();
  uint32_t n = 0;
  while (true) {
    float t = (micros() - t0) * 1e-6f;
    if (t >= dur_s) break;
    m_.loopFOC();
    m_.move(from + dist * (t / dur_s));
    if ((++n & 0x3FF) == 0) yield();
  }
}

float AntiCogging::holdAndMeasure(float target) {
  runFor(cfg_.settle_time_ms, target);
  double acc = 0.0;
  for (uint16_t i = 0; i < cfg_.samples_per_point; ++i) {
    m_.loopFOC();
    m_.move(target);
    acc += effort();
  }
  return (float)(acc / cfg_.samples_per_point);
}

// ------------------------------------------------------------ calibration
bool AntiCogging::measurePass(float* out, bool forward, uint16_t& done,
                              uint16_t total, void (*progress)(uint8_t)) {
  const float step = _2PI / TABLE_SIZE;

  // Map table index (absolute mechanical angle) to the shaft_angle frame.
  // shaft_angle grows continuously; mech angle from the sensor is absolute.
  m_.loopFOC();
  float mech0  = m_.sensor->getMechanicalAngle();
  float shaft0 = m_.shaft_angle;
  int   dir    = (m_.sensor_direction == Direction::CCW) ? -1 : 1;

  // Shaft-frame angle at which mech angle == 0
  float base = shaft0 - dir * mech0;
  float prev = shaft0;

  for (uint16_t k = 0; k < TABLE_SIZE; ++k) {
    uint16_t idx = forward ? k : (TABLE_MASK - k);
    // Unwrap so the forward pass sweeps monotonically up, reverse down
    float tgt = base + dir * idx * step;
    if (forward) { while (tgt < prev - 1e-4f) tgt += _2PI; }
    else         { while (tgt > prev + 1e-4f) tgt -= _2PI; }

    moveTo(prev, tgt);
    prev = tgt;

    float e = holdAndMeasure(tgt);
    if (!isfinite(e)) return false;

    // Verify we really are at the expected index (sensor sanity)
    float mech = m_.sensor->getMechanicalAngle();
    int real = (int)lroundf(mech / step) & TABLE_MASK;
    int err = abs(real - (int)idx);
    if (err > TABLE_SIZE / 2) err = TABLE_SIZE - err;
    if (err > 4) {                        // > ~2.8° tracking error
      SIMPLEFOC_DEBUG("ACOG: tracking error at idx", (int)idx);
      return false;
    }
    // Store at the index actually measured
    out[real] = e;

    ++done;
    if (progress && (done % 8 == 0)) progress((uint8_t)(100UL * done / total));
  }
  return true;
}

bool AntiCogging::calibrate(void (*progress)(uint8_t)) {
  if (!m_.sensor || m_.motor_status != FOCMotorStatus::motor_ready) {
    status_ = Status::Error;
    return false;
  }
  status_ = Status::Calibrating;
  enabled_ = false;

  // Save controller state
  auto prevCtrl = m_.controller;
  auto prevPA = m_.P_angle.P;
  auto pvP = m_.PID_velocity.P, pvI = m_.PID_velocity.I, pvD = m_.PID_velocity.D;
  auto pvTf = m_.LPF_velocity.Tf;
  auto pvLim = m_.velocity_limit;

  m_.controller = MotionControlType::angle;
  m_.P_angle.P = cfg_.cal_P_angle;
  m_.PID_velocity.P = cfg_.cal_vel_P;
  m_.PID_velocity.I = cfg_.cal_vel_I;
  m_.PID_velocity.D = cfg_.cal_vel_D;
  m_.PID_velocity.reset();
  m_.LPF_velocity.Tf = cfg_.cal_vel_LPF_Tf;
  m_.velocity_limit = cfg_.approach_velocity * 2.0f;

  static float fwd[TABLE_SIZE];
  static float rev[TABLE_SIZE];
  memset(fwd, 0, sizeof(fwd));
  memset(rev, 0, sizeof(rev));

  uint16_t total = cfg_.