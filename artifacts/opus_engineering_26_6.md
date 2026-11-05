# AM32-Style ESC Firmware: Commutation Timing, DShot1200 Decoding, and Active Braking

This is an original, AM32-style implementation for an STM32G071 (64 MHz) or a comparable MCU. It follows AM32's architecture:

- comparator-based zero-cross (ZC) detection
- timer-driven commutation
- DMA edge-capture DShot decoding
- complementary PWM for damped-light braking

Register-level details are abstracted behind small HAL macros so you can port them to F051, AT32, or GD32 targets.

---

## 1. Configuration and Shared State

```c
// config.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ          64000000UL
#define PWM_FREQ_HZ          24000UL
#define PWM_ARR              (CPU_FREQ_HZ / PWM_FREQ_HZ)      // ~2666 counts
#define DEAD_TIME_NS         300                               // match your MOSFET driver
#define COM_TIMER_HZ         2000000UL                         // 0.5 us/tick commutation timer

#define DSHOT_TIMER_HZ       CPU_FREQ_HZ                       // input-capture timer, no prescale
#define DSHOT_EDGES          32                                // 16 bits x 2 edges

#define MIN_DUTY             (PWM_ARR / 20)                    // 5% floor once running
#define STARTUP_DUTY         (PWM_ARR / 12)
#define MAX_DUTY             (PWM_ARR - 10)

typedef struct {
    uint8_t  timing_advance_deg;   // 0..30 electrical degrees (AM32 default ~15-22.5)
    bool     comp_pwm;             // damped light / synchronous rectification
    bool     brake_on_stop;        // all low-sides on at zero throttle
    uint8_t  brake_strength;       // 0..10 proportional brake on stop
    bool     bidirectional;        // inverted DShot + eRPM telemetry
    bool     reversed;
    uint8_t  motor_poles;
} esc_settings_t;

extern esc_settings_t settings;

typedef struct {
    volatile uint8_t  step;                 // 1..6
    volatile uint32_t commutation_interval; // COM_TIMER ticks per 60 deg electrical
    volatile uint32_t last_zc_time;
    volatile uint16_t duty;
    volatile bool     running;
    volatile bool     zc_found;
    volatile uint8_t  good_zc_count;
    volatile uint8_t  bad_zc_count;
    volatile bool     rising_expected;
    volatile uint16_t filter_level;         // comparator sample count
} motor_state_t;

extern motor_state_t motor;
```

---

## 2. Six-Step Commutation Table and Phase Driver

Each step drives one phase with PWM, holds one phase low, and leaves one phase floating. The comparator watches the floating phase.

```c
// phases.c
#include "config.h"
#include "hal_bridge.h"   // phase_pwm(), phase_low(), phase_float(), comp_select()

typedef struct { uint8_t hi, lo, flt; bool rising; } step_def_t;

// A=0, B=1, C=2
static const step_def_t step_table[7] = {
    {0,0,0,0},
    {0,1,2, true },   // 1: A+ B-  C float, BEMF rising
    {0,2,1, false},   // 2: A+ C-  B float, falling
    {1,2,0, true },   // 3: B+ C-  A float, rising
    {1,0,2, false},   // 4: B+ A-  C float, falling
    {2,0,1, true },   // 5: C+ A-  B float, rising
    {2,1,0, false},   // 6: C+ B-  A float, falling
};

void apply_step(uint8_t step)
{
    const step_def_t *s = &step_table[step];

    // Order matters: float first to avoid shoot-through across transitions
    phase_float(s->flt);
    phase_low(s->lo);

    // comp_pwm: high side PWM with complementary low side (damped light)
    // else:     high side PWM, low side off during off-time (freewheel via body diode)
    phase_pwm(s->hi, settings.comp_pwm);

    comp_select(s->flt);                  // route floating phase to comparator +input
    motor.rising_expected = s->rising;
}
```

The HAL bridge maps these calls to timer CCER/CCMR bits:

```c
// hal_bridge.h (sketch for TIM1 CH1-3 / CH1N-3N)
static inline void phase_pwm(uint8_t ph, bool comp) {
    tim1_set_oc_mode(ph, OC_PWM1);
    tim1_enable_high(ph, true);
    tim1_enable_low(ph, comp);          // complementary output with dead time
}
static inline void phase_low(uint8_t ph) {
    tim1_set_oc_mode(ph, OC_FORCE_INACTIVE); // high off, complementary low on
    tim1_enable_high(ph, true);
    tim1_enable_low(ph, true);
}
static inline void phase_float(uint8_t ph) {
    tim1_enable_high(ph, false);
    tim1_enable_low(ph, false);
}
```

---

## 3. Motor Timing: Zero-Cross Detection to Commutation

### 3.1 Timing Model

- One commutation interval `T` equals 60° electrical.
- The ZC occurs nominally 30° (that is, `T/2`) before the ideal commutation point.
- Advance `A°` shifts commutation earlier, which gives the delay:

```
delay = T/2 − T·A/60 = T·(30 − A)/60
```

### 3.2 Zero-Cross Detection

ZC detection is polled in a fast loop at high speed and interrupt-driven at low speed. The version below uses the comparator interrupt with a software majority filter.

```c
// commutation.c
#include "config.h"

motor_state_t motor;

#define BLANKING_FRACTION   8      // ignore comparator for T/8 after commutation (demag)

static inline bool comp_output(void) { return COMP_READ(); }

void comp_isr(void)     // comparator edge interrupt
{
    uint32_t now = COM_TIMER_COUNT();

    // Demagnetization blanking: inductive kickback causes false edges
    if ((now - motor.last_zc_time) < motor.commutation_interval / BLANKING_FRACTION + 2)
        return;

    // Filter: require consistent comparator state for N samples.
    // N scales down at high eRPM where intervals are short.
    for (uint16_t i = 0; i < motor.filter_level; i++) {
        if (comp_output() != motor.rising_expected) {
            motor.bad_zc_count++;
            return;
        }
    }

    COMP_IRQ_DISABLE();                       // one ZC per step
    uint32_t this_zc = now;
    uint32_t measured = this_zc - motor.last_zc_time;
    motor.last_zc_time = this_zc;

    // IIR smoothing of the 60-degree interval (weight 1/4 new)
    motor.commutation_interval = (3 * motor.commutation_interval + measured) >> 2;

    uint32_t T = motor.commutation_interval;
    uint32_t delay = (T * (30u - settings.timing_advance_deg)) / 60u;

    // Schedule commutation in the one-shot commutation timer
    COM_TIMER_SET_COMPARE(this_zc + delay);
    COM_TIMER_IRQ_ENABLE();

    motor.zc_found = true;
    if (motor.good_zc_count < 255) motor.good_zc_count++;
    motor.bad_zc_count = 0;
}

void com_timer_isr(void)   // fires at scheduled commutation instant
{
    COM_TIMER_IRQ_DISABLE();

    if (settings.reversed) motor.step = (motor.step == 1) ? 6 : motor.step - 1;
    else                   motor.step = (motor.step == 6) ? 1 : motor.step + 1;

    apply_step(motor.step);

    // Adaptive filter: more samples at low speed (noisy), fewer at high speed
    uint32_t T = motor.commutation_interval;
    motor.filter_level = (T > 2000) ? 12 : (T > 500) ? 6 : (T > 150) ? 3 : 2;

    COMP_IRQ_CLEAR_PENDING();
    COMP_SET_EDGE(motor.rising_expected ? EDGE_RISING : EDGE_FALLING);
    COMP_IRQ_ENABLE();
}
```

### 3.3 Startup and Desync Protection

```c
#define STARTUP_STEP_TICKS   4000     // 2 ms per step at 0.5 us ticks
#define ZC_LOCK_THRESHOLD    20
#define DESYNC_TIMEOUT_MUL   4        // no ZC within 4*T -> desync

void motor_start(void)
{
    motor.step = 1;
    motor.commutation_interval = STARTUP_STEP_TICKS;
    motor.good_zc_count = 0;
    motor.duty = STARTUP_DUTY;
    motor.running = true;
    motor.last_zc_time = COM_TIMER_COUNT();
    apply_step(motor.step);
    COMP_IRQ_ENABLE();
}

// Called at ~20 kHz from the main control tick
void motor_supervisor(void)
{
    if (!motor.running) return;

    uint32_t since_zc = COM_TIMER_COUNT() - motor.last_zc_time;

    if (since_zc > motor.commutation_interval * DESYNC_TIMEOUT_MUL) {
        // Lost sync: force a blind commutation and back off
        motor.good_zc_count = 0;
        motor.commutation_interval = STARTUP_STEP_TICKS;
        motor.last_zc_time = COM_TIMER_COUNT();
        com_timer_isr();
        if (motor.duty > STARTUP_DUTY) motor.duty = STARTUP_DUTY;
    }

    // Until locked, cap duty to protect FETs on stalled/desync starts
    if (motor.good_zc_count < ZC_LOCK_THRESHOLD && motor.duty > STARTUP_DUTY * 2)
        motor.duty = STARTUP_DUTY * 2;
}
```

---

## 4. High-Resolution DShot1200 Decoding

### 4.1 Signal Parameters

At 64 MHz, the DShot1200 bit timing works out as follows:

| Parameter | Time | Ticks |
|---|---|---|
| Bit period | 833 ns | 53.3 |
| "1" high time | 625 ns | 40 |
| "0" high time | 313 ns | 20 |
| Decision threshold | ~470 ns | **30** |

The frame layout is: `[11-bit value][1 telemetry bit][4-bit CRC]`, sent MSB first.

The CRC is computed as `(v ^ v>>4 ^ v>>8) & 0xF`. For bidirectional DShot, the signal is idle-high and the CRC is inverted (`~crc`).

### 4.2 Capture Strategy

1. The input-capture timer runs on both edges, and DMA writes 32 timestamps (rising and falling for each bit).
2. The DMA transfer-complete interrupt triggers decoding.
3. The decoder computes the high width as the difference between each edge pair.

Edge-difference decoding is independent of absolute bit rate. Using ratio-based classification against the measured frame period allows the same code to auto-detect DShot300, DShot600, and DShot1200.

```c
// dshot.c
#include "config.h"

static volatile uint16_t dma_buf[DSHOT_EDGES];
volatile uint16_t dshot_value;       // 0..2047
volatile bool     dshot_telem_req;
volatile uint16_t dshot_good_frames, dshot_bad_frames;
volatile bool     dshot_frame_ready;

void dshot_init(void)
{
    // Timer: no prescaler, max ARR, CH1 input capture on BOTH edges,
    // input filter ~2 samples to reject ringing (do not over-filter at 1200).
    IC_TIMER_CONFIG(/*psc*/0, /*arr*/0xFFFF, IC_BOTH_EDGES, /*filter*/2);
    DMA_CONFIG(IC_TIMER_CCR1, dma_buf, DSHOT_EDGES, DMA_P2M | DMA_HALFWORD);
    DMA_ENABLE_TC_IRQ();
    IC_TIMER_START();
}

static inline uint8_t dshot_crc(uint16_t v12)
{
    return (v12 ^ (v12 >> 4) ^