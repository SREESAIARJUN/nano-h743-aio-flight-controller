# Anti-Cogging Calibration and Feedforward Module for SimpleFOC (ESP32-S3)

I don't know what "Cycle 24" refers to in your project, so the module rests on these assumptions. Adjust them if your spec differs:

- **Table size:** 512 bins, one per mechanical revolution, indexed by the raw sensor mechanical angle. The table therefore stays valid even if `sensor_offset` or the zero electric angle changes.
- **Calibration method:** The motor is held stiffly in SimpleFOC `angle` mode at each bin. The torque setpoint required to hold position (`motor.current_sp`) is averaged and recorded.
  - Two passes are made, forward and reverse. Averaging them cancels Coulomb friction.
  - The DC component is removed, which takes out gravity and bias.
- **"Sinusoidal":** The raw table is projected onto a truncated Fourier series of harmonics 1…K. This denoises it and makes it smooth and continuous. You can optionally keep only multiples of the cogging fundamental (LCM of slots and poles).
- **Runtime:** The feedforward is linearly interpolated, gain-scaled, and faded out with speed. It is injected as an additive torque, in V or A depending on the torque mode.
- **Persistence:** The table is stored in NVS via `Preferences`, with a magic number, version, and CRC32.
- **Calibration loop:** Non-blocking state machine, so `loopFOC()` keeps running at full rate.

---

## `CoggingCompensator.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class CoggingCompensator {
public:
    static constexpr uint16_t N = 512;

    enum class State : uint8_t {
        Idle, Prepare, Step, Settle, Sample, Reverse, Process, Done, Error
    };

    struct Config {
        float    calVelocityLimit   = 2.0f;    // rad/s while moving between bins
        float    calAngleP          = 20.0f;   // stiff position loop during calibration
        uint32_t settleTimeUs       = 30000;   // wait after each step
        uint16_t samplesPerBin      = 64;      // averaged loop iterations per bin
        uint32_t sampleIntervalUs   = 200;     // min spacing between samples
        uint8_t  harmonics          = 48;      // Fourier terms kept (1..K), 0 = no fit
        uint16_t cogFundamental     = 0;       // >0: keep only multiples of this (e.g. LCM(slots,poles))
        float    maxPositionErrRad  = 0.05f;   // abort if tracking error exceeds this
        float    maxTorqueCmd       = 0.0f;    // abort if |current_sp| exceeds (0 = disabled)
    };

    struct Runtime {
        float gain            = 1.0f;   // 0..1 typical
        float fadeStartRadS   = 20.0f;  // full compensation below this speed
        float fadeEndRadS     = 60.0f;  // zero compensation above this speed
        float lowPassAlpha    = 1.0f;   // 1 = no filtering of the output
    };

    explicit CoggingCompensator(BLDCMotor& motor) : _motor(motor) {}

    Config  cfg;
    Runtime rt;

    // ----- calibration -----
    bool  startCalibration();          // motor must be initFOC()'d
    void  abortCalibration();
    // Call every loop() iteration right after motor.loopFOC().
    // While calibrating it calls motor.move() itself.
    void  update();
    State state() const        { return _state; }
    bool  isCalibrating() const{ return _state != State::Idle && _state != State::Done && _state != State::Error; }
    float progress() const;    // 0..1

    // ----- runtime injection -----
    void  enable(bool en)      { _enabled = en && _valid; }
    bool  enabled() const      { return _enabled; }
    bool  valid() const        { return _valid; }
    float feedforward();                         // torque units of current torque mode
    float compensate(float torqueCmd)            { return torqueCmd + feedforward(); }
    // Velocity loop with injection (set motor.controller = MotionControlType::torque)
    void  moveVelocity(float targetRadS);

    // ----- persistence -----
    bool  save(const char* ns = "cogcomp");
    bool  load(const char* ns = "cogcomp");
    void  clear();

    // ----- access -----
    float tableAt(uint16_t i) const { return _table[i % N]; }
    void  dump(Print& out) const;

private:
    BLDCMotor& _motor;
    State      _state = State::Idle;
    bool       _valid = false;
    bool       _enabled = false;

    float      _table[N] = {0};
    float      _sumFwd[N], _sumRev[N];
    uint16_t   _cntFwd[N], _cntRev[N];

    // calibration bookkeeping
    uint16_t   _step = 0;
    int8_t     _dir = 1;
    float      _startShaft = 0;
    float      _target = 0;
    uint32_t   _tMark = 0, _tLastSample = 0;
    uint16_t   _nSamples = 0;
    float      _accTorque = 0;
    uint32_t   _accBinVotes[2] = {0,0};
    int        _accBinLast = -1;
    float      _yLP = 0;

    // saved motor settings
    MotionControlType _savedCtrl;
    float      _savedVelLim = 0, _savedAngleP = 0;

    // helpers
    static float wrap2Pi(float a);
    static int   binOf(float mechAngle);
    float        mechAngle() const;
    void         finishPass();
    bool         processTable();
    void         fillGaps(float* t);
    void         fourierFit(float* t);
    void         restoreMotor();
    static uint32_t crc32(const uint8_t* d, size_t n);
};
```

---

## `CoggingCompensator.cpp`

```cpp
#include "CoggingCompensator.h"
#include <math.h>

static constexpr float TWO_PI_F  = 6.28318530718f;
static constexpr float BIN_WIDTH = TWO_PI_F / CoggingCompensator::N;

// ---------------------------------------------------------------- helpers
float CoggingCompensator::wrap2Pi(float a) {
    a = fmodf(a, TWO_PI_F);
    return a < 0 ? a + TWO_PI_F : a;
}

int CoggingCompensator::binOf(float m) {
    int b = (int)lroundf(wrap2Pi(m) / BIN_WIDTH);
    return b % N;
}

float CoggingCompensator::mechAngle() const {
    // Cached value from the last sensor update performed in loopFOC/move:
    // no extra SPI/I2C transaction here.
    return _motor.sensor ? _motor.sensor->getMechanicalAngle() : 0.0f;
}

float CoggingCompensator::progress() const {
    if (_state == State::Done) return 1.0f;
    if (!isCalibrating()) return 0.0f;
    float p = (float)_step / N;
    return (_dir > 0) ? 0.5f * p : 0.5f + 0.5f * p;
}

// ---------------------------------------------------------------- calibration
bool CoggingCompensator::startCalibration() {
    if (!_motor.sensor || _motor.motor_status != FOCMotorStatus::motor_ready) {
        _state = State::Error;
        return false;
    }
    _enabled = false;

    memset(_sumFwd, 0, sizeof(_sumFwd)); memset(_sumRev, 0, sizeof(_sumRev));
    memset(_cntFwd, 0, sizeof(_cntFwd)); memset(_cntRev, 0, sizeof(_cntRev));

    _savedCtrl   = _motor.controller;
    _savedVelLim = _motor.velocity_limit;
    _savedAngleP = _motor.P_angle.P;

    _motor.controller     = MotionControlType::angle;
    _motor.velocity_limit = cfg.calVelocityLimit;
    _motor.P_angle.P      = cfg.calAngleP;
    _motor.PID_velocity.reset();

    _startShaft = _motor.shaft_angle;
    _target     = _startShaft;
    _motor.target = _target;

    _dir  = 1;
    _step = 0;
    _tMark = micros();
    _state = State::Prepare;
    return true;
}

void CoggingCompensator::abortCalibration() {
    if (isCalibrating()) restoreMotor();
    _state = State::Idle;
}

void CoggingCompensator::restoreMotor() {
    _motor.controller     = _savedCtrl;
    _motor.velocity_limit = _savedVelLim;
    _motor.P_angle.P      = _savedAngleP;
    _motor.target         = (_savedCtrl == MotionControlType::angle) ? _motor.shaft_angle : 0.0f;
    _motor.PID_velocity.reset();
}

void CoggingCompensator::update() {
    if (!isCalibrating()) return;

    _motor.move(_target);                 // stiff angle hold
    const uint32_t now = micros();

    // safety checks
    if (_state == State::Sample || _state == State::Settle) {
        float err = fabsf(_target - _motor.shaft_angle);
        bool torqueBad = cfg.maxTorqueCmd > 0 && fabsf(_motor.current_sp) > cfg.maxTorqueCmd;
        if ((_state == State::Sample && err > cfg.maxPositionErrRad) || torqueBad) {
            restoreMotor();
            _state = State::Error;
            return;
        }
    }

    switch (_state) {
    case State::Prepare:            // let initial hold settle (0.5 s)
        if (now - _tMark > 500000) { _state = State::Settle; _tMark = now; }
        break;

    case State::Step:
        _target = _startShaft + _dir * (float)_step * BIN_WIDTH
                  + (_dir < 0 ? TWO_PI_F