# AM32-Style ESC Core: Motor Timing, DShot1200 Decoding, and Active Braking (Cycle 19)

This is an original reference implementation that follows the AM32 architecture: sensorless 6-step commutation, comparator-based BEMF zero-cross detection, DMA-captured DShot, and complementary-PWM damped light. It is not copied from the AM32 repository. It targets an **STM32F051 at 48 MHz**, which is the classic AM32 target. Pin and peripheral mappings are kept behind small hooks so the code can be ported to G071, AT32F421 or GD32E230.

---

## 1. System Architecture

| Subsystem | Peripheral | Clock / Resolution | Priority |
|---|---|---|---|
| 3-phase PWM | TIM1 CH1–3 + CH1N–3N (complementary) | 48 MHz, 24 kHz center-aligned | — |
| Zero-cross interval | TIM2 (free-running) | 0.5 µs/tick (PSC=23) | — |
| Commutation scheduler | TIM14 (one-shot compare) | 0.5 µs/tick | 0 (highest) |
| BEMF comparator | COMP1 + EXTI21 | async | 1 |
| DShot input | TIM3 CH1 input capture, both edges, + DMA | 48 MHz (20.8 ns) | 1 |
| Control loop | TIM6 | 20 kHz tick | 2 |
| Main loop | — | background | — |

The interrupt priorities enforce one rule: **commutation is never delayed by input decoding**. The DShot DMA completes autonomously, and its interrupt work (~2 µs) only runs between edges.

---

## 2. Shared Definitions

```c
// esc_config.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx.h"

#define CPU_HZ              48000000UL
#define PWM_FREQ_HZ         24000UL
#define PWM_ARR             ((CPU_HZ / PWM_FREQ_HZ / 2) - 1)   // center-aligned: 999
#define TIMER_TICK_NS       500                              // TIM2/TIM14 at 2 MHz

#define DEADTIME_NS         300        // tune to FET Qg / driver; see §6.3
#define BLANKING_PERCENT    25         // ignore comparator for first 25% of step
#define ZC_FILTER_MIN       2          // comparator re-read count at high rpm
#define ZC_FILTER_MAX       12         // at low rpm
#define MIN_COMMUTATION_US  40         // ~250k eRPM ceiling guard
#define MAX_COMMUTATION_US  20000      // below this speed -> considered stalled

typedef struct {
    uint8_t  timing_advance;       // 0..4  -> 0, 7.5, 15, 22.5, 30 deg electrical
    bool     damped_light;         // complementary PWM (active freewheel braking)
    bool     brake_on_stop;
    uint8_t  brake_strength;       // 0..10
    uint8_t  drag_brake_strength;  // 0..10, applied at zero throttle while spinning
    bool     bidirectional;        // 3D mode
    bool     reversed;
    uint16_t motor_kv;
    uint8_t  motor_poles;
} esc_settings_t;

extern esc_settings_t cfg;
```

---

## 3. High-Resolution DShot Decoding (DShot150 → DShot1200, auto-detect)

### 3.1 Timing budget at 48 MHz

| Protocol | Bit period | T0H (37.5%) | T1H (75%) | Ticks/bit @48 MHz | Frame |
|---|---|---|---|---|---|
| DShot300 | 3.333 µs | 1.25 µs | 2.50 µs | 160 | 53 µs |
| DShot600 | 1.667 µs | 0.625 µs | 1.25 µs | 80 | 26.7 µs |
| **DShot1200** | **0.833 µs** | **0.3125 µs** | **0.625 µs** | **40** | **13.3 µs** |

At DShot1200 the gap between a "0" (15 ticks) and a "1" (30 ticks) is only 15 ticks. For that reason:

- The timer runs **unprescaled** (PSC=0).
- Edges are captured by **hardware input capture + DMA**. No GPIO polling is used, so ISR latency cannot corrupt the measurement.
- The decision threshold is **der