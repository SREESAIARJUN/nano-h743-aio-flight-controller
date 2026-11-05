# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Damped-Light Active Braking

This is a clean-room implementation in the architecture of AM32. It is not verbatim AM32 source. The target is an STM32G071-class MCU at 64 MHz. The hardware-specific parts sit behind a thin HAL layer so the code ports to F051, F031 and AT32F421.

---

## 0. System Overview

```
             ┌────────────────────────────────────────────────────┐
 DShot in ──►│ TIM3 CH1 input capture ──DMA──► edge buffer [32]   │
             │        │ DMA TC IRQ (prio 0)                       │
             │        ▼                                           │
             │  dshot_decode() ─► CRC ─► throttle / command       │
             │                                                    │
 BEMF A/B/C─►│ COMP + filter ─► zero_cross_isr() (prio 1)         │
             │        │ interval measure, advance calc            │
             │        ▼                                           │
             │  TIM14 one-shot (commutation timer) ─► commutate() │
             │                                                    │
             │ TIM1 CH1-3 + CH1N-3N complementary PWM (24 kHz)    │
             │   damped-light: low side driven in PWM off-time    │
             └────────────────────────────────────────────────────┘
```

**Interrupt priorities**

| Priority | Source | Reason |
|---|---|---|
| 0 | DShot DMA | Must finish before the next frame (about 13 µs at DShot1200) |
| 1 | Comparator ZC, commutation timer | Timing-critical |
| 2 | 20 kHz control loop | Ramp, braking, telemetry |
| 3 | Main loop | EEPROM, LEDs |

---

## 1. Configuration and Shared Types

```c
/* esc_config.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ            64000000UL
#define PWM_FREQ_HZ            24000UL
#define PWM_ARR                ((CPU_FREQ_HZ / PWM_FREQ_HZ) - 1)   /* 2665 */
#define DEADTIME_NS            300
#define COMM_TIMER_HZ          1000000UL     /* 1 µs tick, TIM14/TIM2 */

#define MOTOR_POLES            14
#define MIN_DUTY               (PWM_ARR / 20)
#define STARTUP_DUTY_MAX       (PWM_ARR / 6)
#define MAX_RAMP_PER_TICK      8              /* duty counts per 50 µs */

/* DShot */
#define DSHOT_BITS             16
#define DSHOT_EDGES            (DSHOT_BITS * 2)
#define DSHOT_CMD_MAX          47
#define DSHOT_THROTTLE_MIN     48
#define DSHOT_THROTTLE_