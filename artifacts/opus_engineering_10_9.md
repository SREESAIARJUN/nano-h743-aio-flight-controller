# Anti-Cogging Calibration & Feedforward Module for SimpleFOC (ESP32-S3)

I don't have context for what "Cycle 10" refers to in your project, so this is a self-contained implementation. My assumptions are listed at the end. Tell me if your previous iterations used different conventions and I'll adapt it.

## How it works

1. **Calibration.** The motor is held in angle mode with a stiff PID at 512 mechanical positions.
   - The velocity loop's integrator settles to exactly the torque needed to hold position against cogging. That value is `current_sp`.
   - The holding torque is recorded in both sweep directions, and the two sweeps are averaged so Coulomb friction (which flips sign) cancels.
2. **Sinusoidal fit.** The raw table is projected onto a truncated Fourier series of the first *H* mechanical harmonics, optionally only multiples of the cogging order.
   - This removes noise and the DC/load offset.
   - It produces a smooth, periodic 512-point table.
3. **Feedforward.** At runtime the table is linearly interpolated at the sensor's mechanical angle.
   - The result is injected into `motor.current_sp` after `move()`.
   - Injection fades out with speed.
   - It is safe against `motion_downsample`, meaning it never accumulates.
4. **Persistence.** The table is stored in NVS (`Preferences`) with a magic number, version, pole pairs, and a CRC32.

---

## `AntiCogging.h`

```cpp
#pragma once
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>

class AntiCogging {
public:
    static constexpr uint16_t N = 512;                 // table resolution (power of 2)
    static constexpr uint32_t MAGIC = 0xAC0C5120;
    static constexpr uint16_t VERSION = 10;

    struct Config {
        // --- calibration motion ---
        float    settle_ms        = 25.0f;   // wait after each step before sampling
        float    sample_ms        = 10.0f;   // averaging window per position
        float    overtravel_bins  = 16;      // extra bins past 1 rev (removes start transient)
        // --- stiff calibration controller (angle mode) ---
        float    cal_P_angle      = 40.0f;
        float    cal_P_vel        = 0.3f;
        float    cal_I_vel        = 15.0f;
        float    cal_D_vel        = 0.0f;
        float    cal_vel_lpf_Tf   = 0.005f;
        float    cal_vel_limit    = 5.0f;    // rad/s
        // --- sinusoidal (Fourier) fit ---
        uint16_t harmonics        = 64;      // highest mechanical harmonic kept (<= N/2)
        uint16_t harmonic_step    = 1;       // keep only multiples of this (e.g. LCM(slots,poles))
        bool     remove_dc        = true;    // DC = gravity/load/friction residue
        // --- runtime injection ---
        float    gain             = 1.0f;    // 0..1.x scale of feedforward
        float    fade_start_vel   = 30.0f;   // rad/s: full compensation below this
        float    fade_end_vel     = 80.0f;   // rad/s: zero compensation above this
        float    max_injection    = 0.0f;    // 0 = no clamp, else |ff| <= this (A or V)
    };

    AntiCogging(BLDCMotor& motor, Sensor& sensor);

    // Blocking calibration (~30 s with defaults). Motor must be initFOC()'d.
    bool calibrate(const Config& cfg, Print* log = &Serial);

    // Call right AFTER motor.move() each loop iteration.
    void IRAM_ATTR inject();

    // Pure lookup (A or V, same units as motor.current_sp).
    float IRAM_ATTR compensation(float mech_angle) const;

    bool save(const char* ns = "anticog");
    bool load(const char* ns = "anticog");
    void clear();

    void enable(bool en)          { enabled_ = en && valid_; }
    bool isEnabled() const        { return enabled_; }
    bool isValid() const          { return valid_; }
    void setGain(float g)         { cfg_.gain = g; }
    Config& config()              { return cfg_; }
    const float* table() const    { return table_; }
    void dump(Print& p) const;

private:
    struct Blob {
        uint32_t magic;
        uint16_t version;
        uint16_t n;
        int32_t  pole_pairs;
        float    gain;
        float    table[N];
        uint32_t crc;
    };

    BLDCMotor& motor_;
    Sensor&    sensor_;
    Config     cfg_;
    float      table_[N];
    bool       valid_   = false;
    bool       enabled_ = false;

    // injection bookkeeping (downsample-safe)
    float base_sp_  = 0.0f;
    float last_out_ = NAN;

    void holdAt(float target, float ms, float* acc, uint32_t* cnt);
    static int   binOf(float mech_angle);
    static void  fillGaps(float* t, const uint32_t* cnt);
    void         fourierFit(float* t, uint16_t H, uint16_t step, bool remove_dc);
    static uint32_t crc32(const uint8_t* d, size_t len);
};
```

---

## `AntiCogging.cpp`

```cpp
#include "AntiCogging.h"
#include <math.h>
#include <string.h>

static constexpr float TWO_PI_F = 6.28318530717958647692f;
static constexpr float BIN_W    = TWO_PI_F / AntiCogging::N;

AntiCogging::AntiCogging(BLDCMotor& motor, Sensor& sensor)
    : motor_(motor), sensor_(sensor) {
    memset(table_, 0, sizeof(table_));
}

// ---------------------------------------------------------------------------
int AntiCogging::binOf(float a) {
    a = fmodf(a, TWO_PI_F);
    if (a < 0) a += TWO_PI_F;
    int b = (int)(a / BIN_W + 0.5f);
    return b & (N - 1);
}

// Run FOC + motion loop at a fixed target. Optionally accumulate holding torque
// into the bin of the *measured* mechanical angle.
void AntiCogging::holdAt(float target, float ms, float* acc, uint32_t* cnt) {
    uint32_t t0 = micros();
    uint32_t dur = (uint32_t)(ms * 1000.0f);
    uint32_t lastYield = millis();
    while ((micros() - t0) < dur) {
        motor_.loopFOC();
        motor_.move(target);
        if (acc) {
            int b = binOf(sensor_.getMechanicalAngle());
            acc[b] += motor_.current_sp;
            cnt[b]++;
        }
        if (millis() - lastYield > 50) {  // keep IDLE task / WDT happy
            vTaskDelay(1);
            lastYield = millis();
        }
    }
}

// ---------------------------------------------------------------------------
bool AntiCogging::calibrate(const Config& cfg, Print* log) {
    cfg_ = cfg;
    if (motor_.motor_status != FOCMotorStatus::motor_ready) {
        if (log) log->println(F("ACOG: motor not ready (run initFOC first)"));
        return false;
    }
    if (cfg_.harmonics > N / 2) cfg_.harmonics = N / 2;
    if (cfg_.harmonic_step == 0) cfg_.harmonic_step = 1;

    enabled_ = false;
    valid_   = false;

    // ---- save controller state ----
    MotionControlType old_ctrl = motor_.controller;
    PIDController oldPA = motor_.P_angle;
    PIDController oldPV = motor_.PID_velocity;
    float oldTf  = motor_.LPF_velocity.Tf;
    float oldVL  = motor_.velocity_limit;
    unsigned int oldDS = motor_.motion_downsample;

    // ---- stiff hold controller ----
    motor_.controller = MotionControlType::angle;
    motor_.motion_downsample = 0;
    motor_.P_angle.P = cfg_.cal_P_angle;
    motor_.P_angle.I = 0; motor_.P_angle.D = 0;
    motor_.PID_velocity.P = cfg_.cal_P_vel;
    motor_.PID_velocity.I = cfg_.cal_I_vel;
    motor_.PID_velocity.D = cfg_.cal_D_vel;
    motor_.PID_velocity.output_ramp = 0;   // no ramp limit during cal
    motor_.LPF_velocity.Tf = cfg_.cal_vel_lpf_Tf;
    motor_.velocity_limit  = cfg_.cal_vel_limit;
    motor_.P_angle.reset();
    motor_.PID_velocity.reset();

    // ---- buffers (heap; ~8 KB) ----
    float*    accF = (float*)calloc(N, sizeof(float));
    float*    accR = (float*)calloc(N, sizeof(float));
    uint32_t* cntF = (uint32_t*)calloc(N, sizeof(uint32_t));
    uint32_t* cntR = (uint32_t*)calloc(N, sizeof(uint32_t));
    if (!accF || !accR || !cntF || !cntR) {
        free(accF); free(accR); free(cntF); free(cntR);
        if (log) log->println(F("ACOG: alloc failed"));
        return false;
    }

    motor_.loopFOC();
    const float start = motor_.shaft_angle;
    const int   over  = (int)cfg_.overtravel_bins;
    const int   steps = N + over;

    if (log) log->printf("ACOG: calibrating %u pts, start=%.4f rad\n", N, start);

    // pre-roll: settle at start, then back up 'over' bins so forward