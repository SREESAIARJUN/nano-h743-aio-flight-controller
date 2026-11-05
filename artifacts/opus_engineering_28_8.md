# Anti-Cogging Module for SimpleFOC on ESP32-S3 (512-point calibration with sinusoidal/harmonic feedforward)

I don't have context on what "Cycle 28" refers to in your project. This implementation is self-contained and makes the following assumptions. Adjust them if your previous iterations defined different conventions.

- **SimpleFOC version:** 2.3.x, `BLDCMotor`.
- **Table index:** the sensor's raw mechanical angle (`sensor->getMechanicalAngle()`, 0–2π). Cogging is tied to rotor/stator geometry, not to electrical angle or multi-turn position.
- **Injection point:** `motor.current_sp`, written after `motor.move()`. `loopFOC()` consumes it on the next iteration in every torque mode (voltage, DC current, FOC current).
- **"Sinusoidal":** the raw 512-bin measurement is decomposed by DFT. It is then rebuilt from a selectable set of harmonics, which removes measurement noise and non-periodic friction artifacts.
- **Persistence:** ESP32 NVS (`Preferences`), protected by a CRC32 check.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>

class AntiCogging {
public:
    static constexpr uint16_t N_POINTS   = 512;           // must be a power of 2
    static constexpr uint16_t IDX_MASK   = N_POINTS - 1;
    static constexpr uint16_t MAX_HARM   = N_POINTS / 2;
    static constexpr uint32_t STORE_MAGIC   = 0xAC0C5120;
    static constexpr uint16_t STORE_VERSION = 3;

    struct Config {
        uint16_t settle_ms        = 25;    // wait after each step before sampling
        uint16_t samples_per_bin  = 40;    // FOC iterations averaged per bin
        uint16_t sample_us        = 150;   // spacing between samples
        uint8_t  revolutions      = 1;     // full fwd+rev passes to average
        uint16_t harmonic_cutoff  = 48;    // keep harmonics 1..cutoff (0 = raw table)
        float    min_harmonic_rel = 0.01f; // drop harmonics < 1% of the largest
        bool     remove_dc        = true;  // DC = friction/gravity bias, not cogging
        float    angle_P          = 40.0f; // stiff angle loop used during calibration
        float    vel_P            = 0.15f;
        float    vel_I            = 6.0f;
        float    vel_limit        = 4.0f;  // rad/s during calibration
        Print*   log              = &Serial;
    };

    enum class Status : uint8_t {
        Uncalibrated, Calibrating, Ready, ErrorNoSensor, ErrorMotorNotReady,
        ErrorAborted, ErrorStorage
    };

    explicit AntiCogging(BLDCMotor& motor);

    // ---- calibration (blocking, motor must be init()+initFOC()'d) ----
    Status calibrate(const Config& cfg);
    Status calibrate() { return calibrate(Config()); }
    void   requestAbort() { abort_ = true; }

    // ---- runtime ----
    // Call in loop():  motor.loopFOC(); motor.move(target); ac.apply();
    void IRAM_ATTR apply();
    float IRAM_ATTR compensation(float mech_angle) const;

    void  enable(bool en)        { enabled_ = en && (status_ == Status::Ready); }
    bool  isEnabled() const      { return enabled_; }
    void  setGain(float g)       { gain_ = constrain(g, 0.0f, 2.0f); }
    float gain() const           { return gain_; }
    void  setVelocityFade(float v_full, float v_zero); // fade FF out at speed
    Status status() const        { return status_; }

    // ---- persistence (NVS) ----
    bool save(const char* ns = "anticog");
    bool load(const char* ns = "anticog");
    bool erase(const char* ns = "anticog");

    // ---- diagnostics ----
    const float* table() const    { return table_; }
    const float* rawTable() const { return raw_; }
    float harmonicAmplitude(uint16_t k) const { return k < MAX_HARM ? hAmp_[k] : 0; }
    void  dumpCSV(Print& out) const;
    void  printHarmonics(Print& out, uint8_t top = 10) const;

private:
    BLDCMotor& m_;
    Status status_ = Status::Uncalibrated;
    volatile bool abort_ = false;
    bool  enabled_ = false;
    float gain_ = 1.0f;
    float fade_full_ = 1e9f, fade_zero_ = 2e9f;
    float last_ff_ = 0;

    // DRAM-resident so lookup in apply() never touches flash
    DRAM_ATTR alignas(16) float table_[N_POINTS];
    float raw_[N_POINTS];
    float hAmp_[MAX_HARM];
    float hPhs_[MAX_HARM];

    static constexpr float BIN_PER_RAD = N_POINTS / _2PI;

    void  runFOC(float target);
    float measureBin(float target, const Config& cfg);
    bool  sweep(float* acc, uint16_t* cnt, bool forward, float base, const Config& cfg);
    void  harmonicFit(const Config& cfg);
    uint32_t crc() const;
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <Preferences.h>
#include <esp_rom_crc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

AntiCogging::AntiCogging(BLDCMotor& motor) : m_(motor) {
    memset(table_, 0, sizeof(table_));
    memset(raw_,   0, sizeof(raw_));
    memset(hAmp_,  0, sizeof(hAmp_));
    memset(hPhs_,  0, sizeof(hPhs_));
}

// ============================================================================
//  Runtime
// ============================================================================

float IRAM_ATTR AntiCogging::compensation(float a) const {
    // a in [0, 2π) from sensor. Linear interpolation, wrap via mask.
    float f   = a * BIN_PER_RAD;
    int32_t i = (int32_t)f;
    float t   = f - (float)i;
    uint16_t i0 = (uint16_t)i & IDX_MASK;
    uint16_t i1 = (i0 + 1) & IDX_MASK;
    return table_[i0] + t * (table_[i1] - table_[i0]);
}

void AntiCogging::setVelocityFade(float v_full, float v_zero) {
    if (v_zero <= v_full) v_zero = v_full + 1e-3f;
    fade_full_ = fabsf(v_full);
    fade_zero_ = fabsf(v_zero);
}

void IRAM_ATTR AntiCogging::apply() {
    if (!enabled_ || !m_.sensor || m_.enabled == 0) { last_ff_ = 0; return; }

    // At higher speed the rotor inertia filters cogging, and sensor/loop
    // latency causes phase error. Fade the injection out linearly.
    float v = fabsf(m_.shaft_velocity);
    float k = gain_;
    if (v > fade_full_) {
        if (v >= fade_zero_) { last_ff_ = 0; return; }
        k *= 1.0f - (v - fade_full_) / (fade_zero_ - fade_full_);
    }

    float ff = k * compensation(m_.sensor->getMechanicalAngle());
    last_ff_ = ff;

    // move() has just written current_sp; loopFOC() reads it next cycle.
    float lim = (m_.torque_controller == TorqueControlType::voltage &&
                 !_isset(m_.phase_resistance))
                ? m_.voltage_limit : m_.current_limit;
    m_.current_sp = _constrain(m_.current_sp + ff, -lim, lim);
}

// ============================================================================
//  Calibration
// ============================================================================

void AntiCogging::runFOC(float target) {
    m_.loopFOC();
    m_.move(target);
}

float AntiCogging::measureBin(float target, const Config& cfg) {
    // Settle: keep the loops running at full rate
    uint32_t t0 = millis();
    while (millis() - t0 < cfg.settle_ms) runFOC(target);

    // Average the torque command the angle/velocity loop needs to hold here.
    double sum = 0;
    for (uint16_t s = 0; s < cfg.samples_per_bin; s++) {
        uint32_t ts = micros();
        runFOC(target);
        sum += m_.current_sp;
        while (micros() - ts < cfg.sample_us) runFOC(target);
    }
    return (float)(sum / cfg.samples_per_bin);
}

bool AntiCogging::sweep(float* acc, uint16_t* cnt, bool fwd, float base,
                        const Config& cfg) {
    const float step = _2PI / N_POINTS;
    // Direction of shaft_angle vs raw mechanical angle (sensor_direction)
    const float dir = (m_.sensor_direction == Direction::CCW) ? -1.0f : 1.0f;

    for (uint16_t n = 0; n < N_POINTS; n++) {
        if (abort_) return false;
        float off    = fwd ? n * step : (N_POINTS - n) * step;
        float target = base + off;

        float u = measureBin(target, cfg);

        // Bin by the *actual* measured mechanical angle, not the commanded one
        float a = m_.sensor->getMechanicalAngle();
        uint16_t idx = (uint16_t)(a * BIN_PER_RAD) & IDX_MASK;
        acc[idx] += u * dir;   // store in raw-sensor torque convention
        cnt[idx]++;

        // Let IDLE run so the task watchdog is fed (motor holds last voltage)
        if ((n & 15) == 15) vTaskDelay(1);
        if ((n & 63) == 0 && cfg.log)
            cfg.log->printf("  %s %3u/%u\n", fwd ? "fwd" : "rev", n, N_POINTS);
    }
    return true;
}

AntiCogging::Status AntiCogging::calibrate(const Config& cfg) {
    if (!m_.sensor)                       return status_ = Status::ErrorNoSensor;
    if (m_.motor_status != FOCMotorStatus::motor_ready)
                                          return status_ = Status::ErrorMotorNotReady;
    status_  = Status::Calibrating;
    abort_   = false;
    enabled_ = false;

    // Save the user's configuration
    MotionControlType ctl = m_.controller;
    float aP = m_.P_angle.P, vP = m_.PID_velocity.P, vI = m_.PID_velocity.I;
    float vD = m_.PID_velocity.D, vl = m_.velocity_limit;

    m_.controller        = MotionControlType::angle;
    m_.P_angle.P         = cfg.angle_P;
    m_.PID_velocity.P    = cfg.vel_P;
    m_.PID_velocity.I    = cfg.vel_I;
    m_.PID_velocity.D    = 0;
    m_.velocity_limit    = cfg.vel_limit;
    m_.PID_velocity.reset();

    static float    acc[N_POINTS];
    static uint16_t cnt[N_POINTS];
    memset(acc, 0, sizeof(acc));
    memset(cnt, 0, sizeof(cnt));

    // Start from the current position to avoid a jump
    runFOC(m_.shaft_angle);
    float base = m_.shaft_angle;

    if (cfg.log) cfg.log->printf("AntiCog: calibrating %u pts, %u rev(s)\n",
                                 N_POINTS, cfg.revolutions);
    bool ok = true;
    for (uint8_t r = 0; r < cfg.revolutions && ok; r++) {
        // Forward then reverse: Coulomb friction flips sign and cancels
        // in the average, while position-dependent cogging does not.
        ok = sweep(acc, cnt, true,  base, cfg) &&
             sweep(acc, cnt, false, base, cfg);
    }

    // Restore the user's configuration
    m_.P_angle.P = aP; m_.PID_velocity.P = vP; m_.PID_velocity.I = vI;
    m_.PID_velocity.D