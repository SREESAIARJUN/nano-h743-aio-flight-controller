# AM32-Style ESC Core: Commutation Timing, DShot1200 Decoding, and Active Braking

This is a reference implementation modeled on the AM32 architecture. It is **not** a verbatim copy of AM32 source. It targets an STM32G071 or STM32F051-class MCU at 64 MHz, with these peripherals:

- **TIM1** drives complementary 3-phase PWM with dead time.
- **TIM3** captures the DShot input on both edges via DMA.
- **COMP** detects back-EMF zero crossings.
- **TIM14** schedules commutation.
- **TIM17** is a free-running 1 MHz interval timer.

Register-level helpers are abstracted as `hal_*` functions. They map directly to LL/HAL calls on your target.

---

## 1. Configuration and Shared State

```c
// esc_config.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ            64000000UL
#define PWM_FREQ_HZ            24000UL
#define PWM_ARR                ((CPU_FREQ_HZ / PWM_FREQ_HZ) - 1)   // 2665
#define DEAD_TIME_TICKS        40          // ~625 ns, tune per FET gate charge

#define MOTOR_POLES            14
#define TIMING_ADVANCE_STEPS   2           // 0..4 => 0, 7.5, 15, 22.5, 30 deg
#define MIN_DUTY               (PWM_ARR / 40)
#define STARTUP_DUTY           (PWM_ARR / 12)
#define MAX_RAMP_PER_LOOP      8           // duty ticks per 20 kHz control tick

#define DRAG_BRAKE_STRENGTH    10          // 0..10
#define BRAKE_ON_STOP          1
#define COMPLEMENTARY_PWM      1           // "damped light" / synchronous rectification
#define BIDIRECTIONAL_DSHOT    0
```

```c
// esc_state.h
#pragma once
#include "esc_config.h"

typedef struct {
    volatile uint8_t  step;               // 1..6
    volatile bool     running;
    volatile bool     reversed;
    volatile uint16_t commutation_interval;   // us per 60 electrical degrees
    volatile uint16_t zc_timestamp;
    volatile uint16_t last_zc_timestamp;
    volatile uint8_t  bad_zc_count;
    volatile uint8_t  good_zc_count;
    volatile uint16_t duty;               // 0..PWM_ARR
    volatile uint16_t duty_target;
    volatile uint32_t e_rpm;
} motor_t;

typedef struct {
    volatile uint16_t throttle;           // 0, 48..2047
    volatile uint16_t command;            // 1..47 special commands
    volatile bool     telemetry_req;
    volatile bool     new_frame;
    volatile uint32_t good_frames;
    volatile uint32_t crc_errors;
    volatile uint32_t last_frame_ms;
} dshot_t;

extern motor_t motor;
extern dshot_t dshot;
```

---

## 2. Six-Step Commutation with Complementary PWM

In complementary mode the low-side FET of the PWM'd phase conducts during the off period. This gives synchronous rectification ("damped light"). It cuts diode losses and enables regenerative deceleration when throttle drops.

```c
// commutation.c
#include "esc_state.h"
#include "hal.h"

motor_t motor;

typedef enum { PH_FLOAT, PH_LOW, PH_PWM } phase_mode_t;

static inline void set_phase(uint8_t ph, phase_mode_t m)
{
    switch (m) {
    case PH_PWM:
        // High side = PWM, low side = complementary PWM (or off if not damped)
        hal_tim1_ch_mode(ph, TIM_OCMODE_PWM1);
        hal_tim1_ch_enable(ph, true);
        hal_tim1_chn_enable(ph, COMPLEMENTARY_PWM);
        break;
    case PH_LOW:
        // Force high side inactive, low side fully on
        hal_tim1_ch_mode(ph, TIM_OCMODE_FORCE_INACTIVE);
        hal_tim1_ch_enable(ph, true);
        hal_tim1_chn_enable(ph, true);
        break;
    case PH_FLOAT:
    default:
        hal_tim1_ch_enable(ph, false);
        hal_tim1_chn_enable(ph, false);
        break;
    }
}

/* Step table: {phaseA, phaseB, phaseC, floating phase for BEMF, ZC rising?} */
static const struct {
    phase_mode_t a, b, c;
    uint8_t      sense;
    bool         rising;
} step_table[7] = {
    {0},
    { PH_PWM,   PH_LOW,   PH_FLOAT, 2, false },   // 1: A+ B-  sense C falling
    { PH_PWM,   PH_FLOAT, PH_LOW,   1, true  },   // 2: A+ C-  sense B rising
    { PH_FLOAT, PH_PWM,   PH_LOW,   0, false },   // 3: B+ C-  sense A falling
    { PH_LOW,   PH_PWM,   PH_FLOAT, 2, true  },   // 4: B+ A-  sense C rising
    { PH_LOW,   PH_FLOAT, PH_PWM,   1, false },   // 5: C+ A-  sense B falling
    { PH_FLOAT, PH_LOW,   PH_PWM,   0, true  },   // 6: C+ B-  sense A rising
};

void comStep(uint8_t s)
{
    // Order matters: float first, then drive, to avoid shoot-through on transition
    if (step_table[s].a == PH_FLOAT) set_phase(0, PH_FLOAT);
    if (step_table[s].b == PH_FLOAT) set_phase(1, PH_FLOAT);
    if (step_table[s].c == PH_FLOAT) set_phase(2, PH_FLOAT);
    set_phase(0, step_table[s].a);
    set_phase(1, step_table[s].b);
    set_phase(2, step_table[s].c);
    hal_tim1_generate_com_event();          // latch preloaded CCER/CCMR atomically
}

void changeCompInput(void)
{
    uint8_t s = motor.step;
    bool rising = step_table[s].rising ^ motor.reversed;
    hal_comp_select_input(step_table[s].sense);   // mux to floating phase vs. virtual neutral
    hal_comp_set_edge(rising ? COMP_EDGE_RISING : COMP_EDGE_FALLING);
}

void commutate(void)
{
    if (!motor.reversed) motor.step = (motor.step % 6) + 1;
    else                 motor.step = (motor.step == 1) ? 6 : motor.step - 1;

    comStep(motor.step);
    changeCompInput();
}
```

---

## 3. Zero-Cross Detection and Timing Advance

The firmware runs a commutation interval filter, then schedules the next commutation:

`wait = interval/2 − advance`

where `advance = interval × TIMING_ADVANCE_STEPS / 8`. Each step is 7.5°, since 60° / 8 = 7.5°.

```c
// timing.c
#include "esc_state.h"
#include "hal.h"

#define BLANKING_US(intv)   ((intv) >> 3)      // ignore comparator ringing after commutation
#define ZC_FILTER_LEVEL     3                  // consecutive confirming samples

static vol