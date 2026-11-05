# Anti-Cogging Calibration and Feedforward Module for SimpleFOC (ESP32-S3)

I don't have context for what "Cycle 18" refers to in your project. It may be an internal iteration or spec. The implementation below is a complete, self-contained module, and you can adapt naming and parameters to your spec.

## How It Works

**Calibration**
1. The motor holds 512 evenly spaced mechanical positions in angle mode. It sweeps forward and then backward.
2. At each position, it averages the holding torque command (`current_sp`).
3. It averages the forward and reverse values. This cancels Coulomb friction, which flips sign with direction, and keeps cogging, which depends only on position.
4. It fits a truncated Fourier series (the "sinusoidal" model). It keeps the strongest N harmonics, which rejects noise, and rebuilds a smooth 512-entry table.
5. It saves the coefficients to NVS (ESP32 `Preferences`).

**Runtime**
- It looks up the table with linear interpolation (`IRAM_ATTR`) and injects the result into `current_sp` or `voltage.q`, depending on the torque mode.
- The injection fades out as speed rises, where cogging is filtered by inertia anyway.

---

### `AntiCogging.h`
```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
  static constexpr uint16_t TABLE_SIZE    = 512;
  static constexpr uint8_t  MAX_HARMONICS = 32;

  struct Config {
    uint16_t settle_ms        = 60;    // wait after each step
    uint16_t samples          = 40;    // samples averaged per bin
    uint16_t sample_period_us = 250;
    uint8_t  harmonics_kept   = 12;    // sinusoidal model order
    uint16_t max_harmonic     = 200;   // highest harmonic considered (< TABLE_SIZE/2)
    float    angle_P          = 25.0f; // stiff position loop during calibration
    float    gain             = 1.0f;  // runtime feedforward gain
    float    fade_start_rad_s = 20.0f; // full injection below this speed
    float    fade_end_rad_s   = 60.0f; // zero injection above this speed
  };

  struct Harmonic { uint16_t order; float a; float b; }; // a*cos + b*sin

  explicit AntiCogging(BLDCMotor& motor) : _motor(motor) {}

  Config cfg;

  bool calibrate(Stream* log = nullptr);
  void IRAM_ATTR apply();             // call after motor.move()
  float IRAM_ATTR lookup(float mech_angle) const;

  bool save(const char* ns = "anticog");
  bool load(const char* ns = "anticog");

  void enable(bool en) { _enabled = en && _valid; }
  bool isValid() const { return _valid; }
  float lastInjection() const { return _last_ff; }
  void dumpTable(Stream& s) const;

private:
  BLDCMotor& _motor;
  float      _table[TABLE_SIZE] = {0};
  Harmonic   _harm[MAX_HARMONICS];
  uint8_t    _n_harm  = 0;
  float      _dc      = 0;
  bool       _valid   = false;
  bool       _enabled = false;
  volatile float _last_ff = 0;

  void  runFor(uint32_t ms);
  float measureBin(float target_angle);
  void  fitHarmonics(const float* raw);
  void  rebuildTable();
};
```

### `AntiCogging.cpp`
```cpp
#include "AntiCogging.h"
#include <math.h>

static inline float wrap2pi(float a) {
  a = fmodf(a, _2PI);
  return a < 0 ? a + _2PI : a;
}

// ---------- Calibration helpers ----------
void AntiCogging::runFor(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    _motor.loopFOC();
    _motor.move();
  }
}

float AntiCogging::measureBin(float target) {
  _motor.target = target;
  runFor(cfg.settle_ms);

  double acc = 0;
  for (uint16_t i = 0; i < cfg.samples; i++) {
    uint32_t t0 = micros();
    while (micros() - t0 < cfg.sample_period_us) {
      _motor.loopFOC();
      _motor.move();
    }
    acc += _motor.current_sp;   // torque command holding the position
  }
  return (float)(acc / cfg.samples);
}

bool AntiCogging::calibrate(Stream* log) {
  if (!_motor.sensor || _motor.motor_status != FOCMotorStatus::motor_ready) {
    if (log) log->println("AC: motor not initialised (initFOC)");
    return false;
  }

  bool prevEn = _enabled;
  _enabled = false;

  // Save controller state
  MotionControlType prevCtrl = _motor.controller;
  float prevP     = _motor.P_angle.P;
  float prevTgt   = _motor.target;
  float prevVelLim = _motor.velocity_limit;

  _motor.controller     = MotionControlType::angle;
  _motor.P_angle.P      = cfg.angle_P;
  _motor.velocity_limit = 5.0f;

  float* fwd = (float*)heap_caps_malloc(TABLE_SIZE * sizeof(float), MALLOC_CAP_DEFAULT);
  float* rev = (float*)heap_caps_malloc(TABLE_SIZE * sizeof(float), MALLOC_CAP_DEFAULT);
  if (!fwd || !rev) { free(fwd); free(rev); return false; }

  // Reference: absolute shaft angle at a known mechanical zero
  _motor.loopFOC();
  float mech0  = _motor.sensor->getMechanicalAngle();
  float base   = _motor.shaft_angle - mech0;   // multi-turn offset
  const float step = _2PI / TABLE_SIZE;

  // Lead-in so backlash/friction is preloaded in the forward direction
  _motor.target = base - 4 * step; runFor(300);

  if (log) log->println("AC: forward sweep");
  for (uint16_t i = 0; i < TABLE_SIZE; i++) {
    fwd[i] = measureBin(base + i * step);
    if (log && (i % 64 == 0)) log->printf("  %u/%u\n", i, TABLE_SIZE);
  }

  _motor.target = base + (TABLE_SIZE + 4) * step; runFor(300);

  if (log) log->println("AC: reverse sweep");
  for (int i = TABLE_SIZE - 1; i >= 0; i--) {
    rev[i] = measureBin(base + i * step);
    if (log && (i % 64 == 0)) log->printf("  %d\n", i);
  }

  // Friction cancellation: cogging = mean of both directions
  for (uint16_t i = 0; i < TABLE_SIZE; i++) fwd[i] = 0.5f * (fwd[i] + rev[i]);

  fitHarmonics(fwd);
  rebuildTable();

  free(fwd); free(rev);

  // Restore
  _motor.controller     = prevCtrl;
  _motor.P_angle.P      = prevP;
  _motor.velocity_limit = prevVelLim;
  _motor.target         = prevTgt;

  _valid   = true;
  _enabled = prevEn || true;
  if (log) log->printf("AC: done, %u harmonics kept\n", _n_harm);
  return true;
}

// ---------- Sinusoidal model (DFT, keep strongest harmonics) ----------
void AntiCogging::fitHarmonics(const float* raw) {
  const uint16_t N = TABLE_SIZE;
  uint16_t kmax = min<uint16_t>(cfg.max_harmonic, N / 2 - 1);
  uint8_t  keep = min<uint8_t>(cfg.harmonics_kept, MAX_HARMONICS);

  double dc = 0;
  for (uint16_t n = 0; n < N; n++) dc += raw[n];
  _dc = 0;  // DC is load/gravity, not cogging -> deliberately dropped
  (void)dc;

  _n_harm = 0;
  for (uint16_t k = 1; k <= kmax; k++) {
    double a = 0, b = 0;
    for (uint16_t n = 0; n < N; n++) {
      float ph = _2PI * k * n / N;
      a += raw[n] * cosf(ph);
      b += raw[n] * sinf(ph);
    }
    Harmonic h{ k, (float)(2.0 * a / N), (float)(2.0 * b / N) };
    float mag = h.a * h.a + h.b * h.b;

    // Insertion into top-'keep' list sorted by magnitude
    if (_n_harm < keep) {
      _harm[_n_harm++] = h;
    } else {
      uint8_t minI = 0; float minM = 1e30f;
      for (uint8_t j = 0; j < _n_harm; j++) {
        float m = _harm[j].a * _harm[j].a + _harm[j].b * _harm[j].b;
        if (m < minM) { minM = m; minI = j; }
      }
      if (mag > minM) _harm[minI] = h;
    }
  }
}

void AntiCogging::rebuildTable() {
  for (uint16_t n = 0; n < TABLE_SIZE; n++) {
    float v = _dc;
    for (uint8_t j = 0; j < _n_harm; j++) {
      float ph = _2PI * _harm[j].order * n / TABLE_SIZE;
      v += _harm[j].a * cosf(ph) + _harm[j].b * sinf(ph);
    }
    _table[n] = v;
  }
}

// ---------- Runtime ----------
float IRAM_ATTR AntiCogging::lookup(float mech_angle) const {
  float pos = wrap2pi(mech_angle) * (TABLE_SIZE / _2PI);
  uint16_t i0 = (uint16_t)pos;
  float frac  = pos - i0;
  i0 &= (TABLE_SIZE - 1);
  uint16_t i1 = (i0 + 1) & (TABLE_SIZE - 1);
  return _table[i0] + frac * (_table[i1] - _table[i0]);
}

void IRAM_ATTR AntiCogging::apply() {
  if (!_enabled) { _last_ff = 0; return; }

  float w = fabsf(_motor.shaft_velocity);
  float fade = 1.0f;
  if (w >= cfg.fade_end_rad_s) fade = 0.0f;
  else if (w > cfg.fade_start_rad_s)
    fade = 1.0f - (w - cfg.fade_start_rad_s) / (cfg.fade_end_rad_s - cfg.fade_start_rad_s);

  float ff = cfg.gain * fade * lookup(_motor.sensor->getMechanicalAngle());
  _last_ff = ff;
  if (ff == 0.0f) return;

  _motor.current_sp += ff;

  // In voltage torque mode, move() already computed voltage.q from current_sp
  if (_motor.torque_controller == TorqueControlType::voltage) {
    float dv = _isset(_motor.phase_resistance) ? ff * _motor.phase_resistance : ff;
    _motor.voltage.q = _constrain(_motor.voltage.q + dv,
                                  -_motor.voltage_limit, _motor.voltage_limit);
  }
}

// ---------- Persistence (NVS) ----------
bool AntiCogging::save(const char* ns) {
  if (!_valid) return false;
  Preferences p;
  if (!p.begin(ns, false)) return false;
  p.putUChar("n", _n_harm);
  p.putFloat("dc", _dc);
  p.putBytes("h", _harm, _n_harm * sizeof(Harmonic));
  p.putUShort("sz", TABLE_SIZE);
  p.end();
  return true;
}

bool AntiCogging::load(const char* ns) {
  Preferences p;
  if (!p.begin(ns, true)) return false;
  if (p.getUShort("sz", 0) != TABLE_SIZE) { p.end(); return false; }
  _n_harm = min<uint8_t>(p.getUChar("n", 0), MAX_HARMONICS);
  _dc     = p.getFloat("dc", 0);
  size_t got = p.getBytes("h", _harm, _n_harm * sizeof(Harmonic));
  p.end();
  if (_n_harm == 0 || got != _n_harm * sizeof(Harmonic)) return false;
  rebuildTable();
  _valid = true;
  return true;
}

void AntiCogging::dumpTable(Stream& s) const {
  for (uint16_t i = 0; i < TABLE_SIZE; i++) {
    s.print(i); s.print('\t'); s.println(_table[i], 5);
  }
}
```

### Usage
```cpp
#include <SimpleFOC.h>
#include "AntiCogging.h"

BLDCMotor motor(7);
BLDCDriver3PWM driver(4, 5, 6, 7);
MagneticSensorSPI sensor(AS5047_SPI, 10);
AntiCogging antiCog(motor);

void setup() {
  Serial.begin(115200);
  sensor.init(); motor.linkSensor(&sensor);
  driver.voltage_power_supply = 24; driver.init(); motor.linkDriver(&driver);
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = M