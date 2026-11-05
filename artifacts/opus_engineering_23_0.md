# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is a reference implementation in the style of AM32 for a sensorless 6-step BLDC ESC. It targets an STM32G071 at 64 MHz. It is my own write-up of the architecture, not a verbatim copy of AM32 source. AM32 itself is GPL-3.0, so if you merge this into it, keep your derived work GPL-compatible.

"Cycle 23" doesn't correspond to anything in AM32's versioning, so I have treated this as a self-contained design.

---

## 1. System Architecture

| Block | Peripheral | Purpose |
|---|---|---|
| Input capture | TIM3 CH1 + DMA (both edges) | DShot pulse-edge timestamps |
| PWM | TIM1 CH1–3 + CH1N–3N, center-aligned, dead-time | Bridge drive, complementary (damped-light) PWM |
| Commutation timer | TIM14 (1 MHz) | Delayed commutation after zero crossing |
| Interval timer | TIM2 (free-running, 1 MHz) | Measures the commutation period |
| BEMF sensing | COMP1/COMP2 + EXTI | Zero-crossing detection |
| 10 kHz tick | TIM16 | Throttle ramp, braking, arming, telemetry scheduling |

**Interrupt priorities** (lower number = higher priority):

0. Comparator / commutation
1. DShot DMA complete
2. 10 kHz control loop
3. Main loop (telemetry, EEPROM)

---

## 2. Common Definitions

```c
// esc_config.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ          64000000UL
#define PWM_FREQ_HZ          24000UL
#define PWM_ARR              (CPU_FREQ_HZ / PWM_FREQ_HZ / 2)  // center-aligned -> 1333
#define DEAD_TIME_NS         300
#define TIMER_1MHZ_PSC       (CPU_FREQ_HZ / 1000000UL - 1)

#define MOTOR_POLES          14
#define DSHOT_FRAME_BITS     16
#define DSHOT_EDGES          (DSHOT_FRAME_BITS * 2)   // rising + falling per bit

typedef struct {
    uint8_t  timing_advance;      // 0..3  -> 0, 7.5, 15, 22.5 deg (AM32 style)
    bool     comp_pwm;            // complementary PWM ("damped light")
    bool     brake_on_stop;
    uint8_t  drag_brake_strength; // 1..10
    bool     bidirectional;       // 3D mode
    bool     bidir_dshot;         // inverted DShot with eRPM telemetry
    uint8_t  motor_kv_div;
    uint16_t startup_power;       // duty units
    bool     reversed;
} esc_settings_t;

extern esc_settings_t cfg;
```

---

## 3. High-Resolution DShot1200 Decoding

### 3.1 Timing budget

| Protocol | Bit period | T1H | T0H | Ticks/bit @64 MHz |
|---|---|---|---|---|
| DShot300 | 3.333 µs | 2.500 µs | 1.250 µs | 213 |
| DShot600 | 1.667 µs | 1.250 µs | 0.625 µs | 107 |
| **DShot1200** | **0.833 µs** | **0.625 µs** | **0.3125 µs** | **53.3** |

DShot1200 leaves about 20 timer ticks of margin between a "0" (20 ticks high) and a "1" (40 ticks high). For that reason the capture timer runs unprescaled at 64 MHz.

Decoding is **rate-adaptive**: instead of a fixed threshold, it compares each high-time to the measured bit period. This makes the same code work for DShot150 through DShot1200 and tolerates FC clock drift.

### 3.2 Capture setup

```c
// dshot.c
#include "esc_config.h"
#include "stm32g0xx.h"

static volatile uint16_t dma_buf[DSHOT_EDGES + 2];
volatile uint16_t dshot_throttle   = 0;   // 0..2047 raw
volatile uint8_t  dshot_command    = 0;
volatile bool     dshot_telem_req  = false;
volatile bool     inverted_input   = false; // bidirectional DShot idles low->high inverted
volatile uint32_t dshot_good_frames = 0, dshot_bad_frames = 0;
volatile uint16_t signal_timeout   