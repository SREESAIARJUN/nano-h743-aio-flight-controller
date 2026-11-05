# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is a self-contained reference implementation written in the architecture and conventions of AM32. It is not a verbatim copy of the GPL source. It targets an **STM32G071 at 64 MHz**, the common AM32 MCU, and ports to the F051 or AT32F421 by changing the timer clock constants. "Cycle 22" is used here as the revision tag for this design iteration.

---

## 1. System Architecture

| Peripheral | Role | Clock / Rate |
|---|---|---|
| TIM1 | 3-phase complementary PWM (CH1-3 / CH1N-3N), dead-time | 64 MHz, 24–48 kHz PWM |
| TIM2 | Free-running commutation interval timer | 2 MHz (0.5 µs/tick) |
| TIM14 | One-shot commutation delay (ZC → commutate) | 2 MHz |
| TIM3 CH1 + DMA | DShot input capture (both edges) and bidir telemetry output | 64 MHz |
| COMP1/COMP2 | BEMF zero-cross versus virtual neutral | Interrupt on edge |
| ADC + DMA | Vbus, current, temperature | Background |
| TIM6 | 20 kHz control loop (duty slew, braking, telemetry) | — |

Interrupt priority, highest first:

1. DShot DMA complete
2. Comparator zero-cross
3. TIM14 commutation
4. TIM6 control loop

DShot must preempt everything. A corrupted frame costs a full packet, whereas a commutation that is a few hundred nanoseconds late is harmless.

---

## 2. Configuration and Shared State

```c
// esc_config.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_HZ              64000000UL
#define PWM_FREQ_HZ         24000UL
#define TIM1_ARR            ((CPU_HZ / PWM_FREQ_HZ) - 1)   // 2665
#define DEADTIME_TICKS      40                              // ~625 ns; tune per FET

#define COMM_TICK_HZ        2000000UL                       // TIM2/TIM14: 0.5 us
#define MIN_COMM_INTERVAL   40      // 20 us  -> ~250k eRPM ceiling
#define MAX_COMM_INTERVAL   20000   // 10 ms  -> below this is "stalled"

typedef struct {
    uint8_t  advance_level;      // 0..4 => 0, 7.5, 15, 22.5, 30 deg electrical
    bool     variable_advance;   // scale advance with RPM
    bool     comp_pwm;           // complementary PWM = damped light
    bool     brake_on_stop;
    uint8_t  drag_brake_pct;     // 0..100, low-side duty when stopped
    bool     bidirectional;      // 3D mode
    bool     bidir_dshot;        // inverted DShot + eRPM telemetry
    uint16_t vbus_brake_limit_mv;// regen clamp
    uint8_t  motor_poles;
} esc_cfg_t;

extern esc_cfg_t cfg;

typedef struct {
    volatile uint8_t  step;               // 1..6
    volatile uint8_t  forward;
    volatile uint16_t comm_interval;      // filtered, TIM2 ticks (60 deg e)
    volatile uint16_t last_zc_time;
    volatile uint16_t zc_count;
    volatile uint8_t  running;
    volatile uint8_t  closed_loop;        // 0 = polling/startup, 1 = interrupt ZC
    volatile uint8_t  bemf_rising;        // expected comparator edge
    volatile uint16_t duty;               // 0..TIM1_ARR
    volatile uint16_t duty_target;
    volatile uint8_t  desync;
} motor_t;

extern motor_t m;
```

---

## 3. Six-Step Commutation and Phase Drive

Each step drives one phase high with PWM, holds one phase low, and floats the third. The floating phase's BEMF is compared against the virtual neutral.

**Damped light (complementary PWM):** during the PWM off-time, the high-side phase's low FET is switched on, so current recirculates through a FET channel instead of a body diode. This is synchronous rectification. Its useful side effect is that when duty drops, the motor's BEMF drives current back into the bus, which actively decelerates the rotor. That is the "damped" behaviour.

```c
// phase_drive.c
#include "stm32g0xx.h"
#include "esc_config.h"

// Per-phase output modes
typedef enum { PH