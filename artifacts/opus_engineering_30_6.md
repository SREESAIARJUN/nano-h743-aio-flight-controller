# Anti-Cogging Calibration & Feedforward Module for SimpleFOC (ESP32-S3)

I don't have context for "Cycle 30" (a project milestone, a previous conversation, or an internal spec), so this is a self-contained implementation based on standard anti-cogging practice. If Cycle 30 has specific requirements, such as a fixed harmonic set, a particular storage layout, or current-mode only, tell me and I'll adapt it.

## Design Summary

| Aspect | Approach |
|---|---|
| **Table** | 512 points over one mechanical revolution, indexed by raw `sensor->getMechanicalAngle()` |
| **Calibration** | Step-and-hold in `angle` mode. At each point it waits to settle, then averages `motor.current_sp`, which is the torque the velocity-loop integrator needs to hold position. |
| **Friction rejection** | Sweeps forward and backward, then averages the two. Coulomb friction flips sign between sweeps and cancels; cogging does not. |
| **"Sinusoidal" smoothing** | DFT of the 512-point table, keeping DC-free harmonics 1…K, then resynthesis. This removes measurement noise and keeps the periodic cogging signature. |
| **Runtime** | Linear interpolation, latency/phase-advance compensation, velocity fade-out, and a magnitude clamp. Total cost is about 1 µs on the S3. |
| **Persistence** | ESP32 NVS (`Preferences`), protected by a magic number, a version, and a CRC32 check |
| **Injection** | The motor runs in `torque` mode. The module provides its own outer velocity/angle loop (or you supply a torque command) and adds the feedforward term. |

---

## `AntiCogging.h`

```cpp
#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

class AntiCogging {
public:
    static constexpr uint16_t TABLE_SIZE    = 512;
    static constexpr uint32_t NVS_MAGIC     = 0xAC0C5120;
    static constexpr uint16_t NVS_VERSION   = 1;

    struct Config {
        // --- Calibration ---
        float    settleTimeMs        = 40.0f;   // wait after each step before sampling
        uint16_t samplesPerPoint     = 32;      // samples averaged per point
        float    settleVelThreshold  = 0.05f;   // rad/s; must be below this to sample
        float    maxSettleTimeMs     = 400.0f;  // give up waiting and sample anyway
        float    calVoltageLimit     = 3.0f;    // safety limit during calibration
        float    calP_angle          = 20.0f;   // stiff position hold
        float    calPID_vel_P        = 0.2f;
        float    calPID_vel_I        = 10.0f;   // integrator supplies holding torque
        float    calPID_vel_D        = 0.0f;
        float    calVelLimit         = 4.0f;    // rad/s while stepping
        bool     bidirectional       = true;    // forward + backward to cancel friction

        // --- Sinusoidal (Fourier) smoothing ---
        bool     fourierSmoothing    = true;
        uint16_t maxHarmonic         = 64;      // keep harmonics 1..maxHarmonic (per mech rev)
        bool     removeDC            = true;    // DC = load/gravity bias, not cogging

        // --- Runtime injection ---
        float    gain                = 1.0f;    // scale factor on the table
        float    maxCompensation     = 1.0f;    // clamp |ff| (same units as torque cmd)
        float    latencyS            = 150e-6f; // phase advance = vel * latency
        float    fadeStartVel        = 30.0f;   // rad/s: start fading out
        float    fadeEndVel          = 80.0f;   // rad/s: fully off
    };

    enum class State : uint8_t { Idle, Calibrating, Ready, Error };

    explicit AntiCogging(BLDCMotor& motor);

    Config& config() { return cfg_; }

    // ---------- Calibration ----------
    // Blocking. Calls loopFOC()/move() internally. Motor must be init'ed and
    // initFOC()'d. The shaft must be free (no external load).
    // progressCb(fraction 0..1) is optional.
    bool calibrate(void (*progressCb)(float) = nullptr);

    // ---------- Persistence ----------
    bool saveToNVS(const char* ns = "anticog");
    bool loadFromNVS(const char* ns = "anticog");
    bool eraseNVS(const char* ns = "anticog");

    // ---------- Runtime ----------
    void enable(bool en) { enabled_ = en && state_ == State::Ready; }
    bool isEnabled() const { return enabled_; }
    State state() const { return state_; }

    // Compute feedforward for the current sensor position/velocity.
    // Call once per loop, after motor.loopFOC().
    float IRAM_ATTR compensation();

    // Convenience: motor.move(torqueCmd + compensation()).
    // Motor must be in MotionControlType::torque.
    void IRAM_ATTR moveTorque(float torqueCmd);

    // Convenience velocity loop with injection (motor in torque mode).
    void IRAM_ATTR moveVelocity(float targetVel, PIDController& velPid);

    // Convenience angle loop with injection (motor in torque mode).
    void IRAM_ATTR moveAngle(float targetAngle, PIDController& posPid, PIDController& velPid);

    // ---------- Diagnostics ----------
    const float* table() const { return table_; }
    float lookup(float mechAngle) const;     // raw table value, interpolated
    void  printTable(Stream& s) const;
    float peakToPeak() const;
    float lastCompensation() const { return lastFF_; }

private:
    BLDCMotor& motor_;
    Config     cfg_;
    State      state_   = State::Idle;
    bool       enabled_ = false;
    float      table_[TABLE_SIZE];
    float      lastFF_  = 0.0f;

    // calibration helpers
    void   runFOCFor(float ms, float targetAngle);
    float  holdAndMeasure(float targetAngle);
    void   fourierSmooth();
    static uint32_t crc32(const uint8_t* data, size_t len);
    static float wrap2PI(float a);

    struct SavedMotorState {
        MotionControlType controller;
        float voltage_limit;
        float velocity_limit;
        float P_angle_P;
        float vel_P, vel_I, vel_D;
        float vel_limit;
    };
    SavedMotorState saveMotorState() const;
    void restoreMotorState(const SavedMotorState& s);
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

float AntiCogging::wrap2PI(float a) {
    a = fmodf(a, TWO_PI_F);
    return (a < 0.0f) ? a + TWO_PI_F : a;
}

// ======================================================================
//  Motor state save/restore (calibration temporarily changes tuning)
// ======================================================================
AntiCogging::SavedMotorState AntiCogging::saveMotorState() const {
    SavedMotorState s;
    s.controller     = motor_.controller;
    s.voltage_limit  = motor_.voltage_limit;
    s.velocity_limit = motor_.velocity_limit;
    s.P_angle_P      = motor_.P_angle.P;
    s.vel_P          = motor_.PID_velocity.P;
    s.vel_I          = motor_.PID_velocity.I;
    s.vel_D          = motor_.PID_velocity.D;
    s.vel_limit      = motor_.PID_velocity.limit;
    return s;
}

void AntiCogging::restoreMotorState(const SavedMotorState& s) {
    motor_.controller          = s.controller;
    motor_.voltage_limit       = s.voltage_limit;
    motor_.velocity_limit      = s.velocity_limit;
    motor_.P_angle.P           = s.P_angle_P;
    motor_.PID_velocity.P      = s.vel_P;
    motor_.PID_velocity.I      = s.vel_I;
    motor_.PID_velocity.D      = s.vel_D;
    motor_.PID_velocity.limit  = s.vel_limit;
    motor_.PID_velocity.reset();
    motor_.P_angle.reset();
}

// ======================================================================
//  Calibration
// ======================================================================
void AntiCogging::runFOCFor(float ms, float targetAngle) {
    const uint32_t t0 = micros();
    const uint32_t dur = (uint32_t)(ms * 1000.0f);
    uint32_t lastYield = millis();
    while ((micros() - t0) < dur) {
        motor_.loopFOC();
        motor_.move(targetAngle);
        if (millis() - lastYield > 50) {   // keep the task watchdog happy
            vTaskDelay(1);
            lastYield = millis();
        }
    }
}

float AntiCogging::holdAndMeasure(float targetAngle) {
    // 1) Minimum settle time
    runFOCFor(cfg_.settleTimeMs, targetAngle);

    // 2) Extra settle until velocity is quiet (bounded)
    const uint32_t tMax = millis() + (uint32_t)cfg_.maxSettleTimeMs;
    while (fabsf(motor_.shaft_velocity) > cfg_.settleVelThreshold && millis() < tMax) {
        runFOCFor(2.0f, targetAngle);
    }

    // 3) Average the torque setpoint produced by the velocity loop.
    //    motor.current_sp is the torque command in the same units that
    //    motor.move() expects in torque mode (V or A depending on config).
    double acc = 0.0;
    for (uint16_t i = 0; i < cfg_.samplesPerPoint; i++) {
        runFOCFor(1.0f, targetAngle);
        acc += motor_.current_sp;
    }
    return (float)(acc / cfg_.samplesPerPoint);
}

bool AntiCogging::calibrate(void (*progressCb)(float)) {
    if (!motor_.enabled || motor_.sensor == nullptr) {
        state_ = State::Error;
        return false;
    }

    enabled_ = false;
    state_   = State::Calibrating;
    const SavedMotorState saved = saveMotorState();

    // Stiff position hold with strong integrator
    motor_.controller          = MotionControlType::angle;
    motor_.voltage_limit       = cfg_.calVoltageLimit;
    motor_.velocity_limit      = cfg_.calVelLimit;
    motor_.P_angle.P           = cfg_.calP_angle;
    motor_.PID_velocity.P      = cfg_.calPID_vel_P;
    motor_.PID_velocity.I      = cfg_.calPID_vel_I;
    motor_.PID_velocity.D      = cfg_.calPID_vel_D;
    motor_.PID_velocity.limit  = cfg_.calVoltageLimit;
    motor_.PID_velocity.reset();
    motor_.P_angle.reset();

    // Map table index -> shaft_angle target.
    // The table is indexed by raw sensor mechanical angle (0..2π),
    // but motor.move() in angle mode works in shaft_angle space.
    // shaft_angle = dir * sensorAngle - offset, so we step in shaft
    // space and record each sample at its actual raw sensor index.
    const float  step     = TWO_PI_F / TABLE_SIZE;
    const float  dir      = (motor_.sensor_direction == Direction::CCW) ? -1.0f : 1.0f;
    const float  start    = motor_.shaft_angle;

    static float fwd[TABLE_SIZE];
    static float bwd[TABLE_SIZE];
    static uint8_t hitF[TABLE_SIZE];
    static uint8_t hitB[TABLE_SIZE];
    memset(fwd, 0, sizeof(fwd));   memset(bwd, 0, sizeof(bwd));
    memset(hitF, 0, sizeof(hitF)); memset(hitB, 0, sizeof(hitB));

    // Pre-roll: one revolution's worth of slack removal
    runFOCFor(300.0f, start);

    const uint32_t totalSteps = cfg_.bidir