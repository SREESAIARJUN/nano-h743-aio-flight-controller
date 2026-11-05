# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking (Cycle 8)

This reference implementation follows AM32's architecture: comparator-based BEMF zero-cross detection, timer-scheduled commutation, DMA input-capture DShot decoding, and complementary PWM for damped-light braking. It is written against an STM32G071 / AT32F421-class MCU at 64 MHz. **It is not verbatim AM32 source.** Register macros are abstracted in `hal_esc.h` so you can port them.

---

## 0. System Architecture

```
            ┌──────────────┐  DMA (32 edges)   ┌──────────────────┐
DShot pin ─►│ TIM3 IC CH1  │──────────────────►│ dshot_buffer[]   │
            └──────────────┘   TC IRQ          └────────┬─────────┘
                                                        ▼
                                              decode_dshot() → newinput
                                                        │
          ┌─────────────────────────────────────────────▼──────────┐
          │ 20 kHz control loop (TIM6): throttle→duty, ramp, brake │
          └─────────────────────────────────────────────┬──────────┘
                                                        ▼
BEMF ─► COMP1 ─► EXTI/COMP IRQ ─► zc_found() ─► TIM14 (commutation delay)
                                                        ▼
                                                 com_step() → TIM1 CH1-3/CH1N-3N
```

**Timer roles**

| Timer | Purpose | Clock |
|---|---|---|
| TIM1 | 3-phase complementary PWM with deadtime | 64 MHz, ARR = 2000 → 32 kHz center-aligned |
| TIM2 | Free-running interval timer | 1 MHz (1 µs ticks) |
| TIM14 | Commutation delay one-shot | 1 MHz |
| TIM3 | DShot input capture + DMA | 64 MHz, PSC = 0 |
| TIM6 | Control loop | 20 kHz |

---

## 1. Common Definitions

```c
// esc_config.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ            64000000UL
#define PWM_ARR                1999        // 32 kHz center-aligned → 16 kHz effective? no: edge-aligned 32 kHz
#define DEAD_TIME_TICKS        40          // ~625 ns @ 64 MHz, adjust for FETs
#define MIN_DUTY               (PWM_ARR / 20)
#define MAX_DUTY               PWM_ARR
#define STARTUP_DUTY_MAX       (PWM_ARR / 6)

#define POLE_PAIRS_DEFAULT     7
#define TIMING_ADVANCE_DEFAULT 2           // 0..3 → 0, 7.5, 15, 22.5 deg
#define ZC_FILTER_MIN          2
#define ZC_FILTER_MAX          12

typedef struct {
    uint8_t  advance_level;        // 0..3 (×7.5°)
    uint8_t  pole_pairs;
    bool     comp_pwm;             // complementary PWM (damped light)
    bool     brake_on_stop;
    uint8_t  drag_brake_strength;  // 0..10
    bool     bidirectional_dshot;
    bool     reversed;
} esc_settings_t;

extern esc_settings_t settings;
```

---

## 2. Motor Timing: Commutation and Zero-Cross Detection

### 2.1 Commutation Table

There are six steps. Each step names a HIGH phase (PWM), a LOW phase (sinks current), and a FLOAT phase (BEMF sensing). For each float phase, the comparator input mux selects that phase and records whether it expects a rising or falling crossing.

```c
// commutation.c
#include "esc_config.h"
#include "hal_esc.h"

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
    phase_t high;