# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking (Cycle 13)

This is a reference implementation in the style of AM32's architecture. It is **not a verbatim copy of AM32 source**. Register-level details target an STM32G071 at 64 MHz, the class of MCU needed for reliable DShot1200. Pin, timer, and comparator mappings must be adapted to your target's `targets.h`.

---

## 1. System Architecture

```
            ┌──────────────┐  TIM3 CH1 IC + DMA (both edges)
 Signal ───►│ DShot decode │──────────────► throttle / commands
            └──────┬───────┘                   │
                   │ (bidir: TX eRPM via same pin)
                   ▼                           ▼
            ┌──────────────┐           ┌───────────────┐
 COMP1/2 ──►│ Zero-cross   │──────────►│ Commutation   │──► TIM1 6-ch PWM
 (BEMF)     │ detect+filter│  TIM14    │ timing/advance│    (complementary,
            └──────────────┘ (interval)└───────┬───────┘     deadtime)
                                               ▼
                                      ┌────────────────┐
                                      │ Damped light / │
                                      │ brake on stop  │
                                      └────────────────┘
```

**Timers used:**

| Timer | Purpose |
|---|---|
| TIM1 | 3-phase complementary PWM, 24–48 kHz, hardware deadtime |
| TIM2 | Commutation timing (1 MHz tick) |
| TIM3 | DShot input capture with DMA |
| TIM14 | Interval / zero-cross timestamping |
| TIM16 | Commutation delay one-shot interrupt |

---

## 2. Configuration (`config.h`)

```c
#pragma once
#include <stdint.h>

#define CPU_FREQ_HZ          64000000UL
#define PWM_FREQ_HZ          24000
#define TIM1_ARR             (CPU_FREQ_HZ / PWM_FREQ_HZ - 1)   // 2665
#define DEADTIME_NS          300

// ---- Motor timing ----
#define TIMING_ADVANCE_STEPS 2      // units of 7.5°, so 2 = 15°  (range 0..4 → 0..30°)
#define MIN_COMM_INTERVAL_US 40     // ~ 250k eRPM ceiling
#define MAX_COMM_INTERVAL_US 20000  // below this speed → desync / restart
#define BEMF_FILTER_MIN      2
#define BEMF_FILTER_MAX      12

// ---- DShot1200 ----
// Bit period 833 ns. T1H ≈ 625 ns, T0H ≈ 313 ns.
// TIM3 runs at 64 MHz (PSC=0), so a bit is ≈ 53 ticks.
#define DSHOT_BIT_TICKS      53
#define DSHOT_THRESHOLD      ((DSHOT_BIT_TICKS * 6) / 10)   // ~32 ticks (≈ 0.47 µs)
#define DSHOT_MIN_BIT        (DSHOT_BIT_TICKS / 2)
#define DSHOT_MAX_BIT        (DSHOT_BIT_TICKS * 2)
#define DSHOT_EDGES          32     // 16 bits × 2 edges
#define DSHOT_BIDIR          1

// ---- Braking ----
#define COMP_PWM_ENABLED     1      // "damped light" (synchronous rectification)
#define BRAKE_ON_STOP        1
#define DRAG_BRAKE_STRENGTH  10     // 1..10, applied while disarmed / zero throttle
```

---

## 3. High-Resolution DShot1200 Decoding (`dshot.c`)

### Strategy

- Capture **every edge** (both polarities) with TIM3 input capture.
- DMA the edges into a 32-word buffer, so there is no per-bit CPU load.
- When DMA completes, compute each bit's **high time** relative to its own period. This ratiometric approach tolerates FC clock drift and lets one decoder handle DShot300, 600, and 1200.
- Validate the CRC (inverted for bidirectional DShot).

```c
#include "config.h"
#include "stm32g0xx.h"

static volatile uint32_t dma_buf[DSHOT_EDGES];
volatile uint16_t dshot_throttle = 0;   //