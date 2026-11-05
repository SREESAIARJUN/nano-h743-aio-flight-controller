# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is an original reference implementation that follows the AM32 architecture. It targets STM32G071 (64 MHz) and notes where STM32F051 (48 MHz) differs. It is not a verbatim copy of AM32, which is GPL-3.0. Use it to understand the design or as a starting point, and validate it on a current-limited supply before flying.

---

## 1. System Architecture

| Peripheral | Role |
|---|---|
| TIM1 CH1–3 + CH1N–3N | 3-phase complementary PWM with hardware dead-time (damping light) |
| TIM3 CH1 + DMA | DShot input capture of both edges, 32 timestamps per frame |
| TIM14 | Commutation timer at 0.5 µs/tick (2 MHz), free-running |
| TIM16 | One-shot "commutate-at" interval timer |
| COMP1/COMP2 + EXTI | Back-EMF zero-crossing detection against the virtual neutral |
| TIM6 | 20 kHz control loop (ramping, desync check, input timeout) |

**Interrupt priorities** (lower number = higher priority):
1. DShot DMA transfer-complete: 0
2. Comparator zero-cross: 0
3. Commutation timer: 0
4. Control loop: 2
5. Telemetry: 3

---

## 2. Configuration and Shared State

```c
/* esc_config.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_HZ               64000000UL
#define COM_TICK_HZ          2000000UL          // 0.5 us per tick
#define PWM_FREQ_HZ          24000UL
#define PWM_ARR              ((CPU_HZ / PWM_FREQ_HZ) - 1)   // 2665
#define DEAD_TIME_NS         300                 // FET-dependent

/* DShot1200: bit = 0.8333 us, T0H = 0.3125 us, T1H = 0.625 us */
#define DSHOT1200_BIT_TICKS  (CPU_HZ / 1200000UL)           // 53 @64 MHz (40 @48 MHz)
#define DSHOT_FRAME_EDGES    32

/* Timing */
#define ADVANCE_DEG_DEFAULT  16      // electrical degrees (0..30)
#define MIN_COM_INTERVAL     40      // ticks (20 us) -> ~500k eRPM ceiling
#define MAX_COM_INTERVAL     40000   // ticks (20 ms) -> below this = stalled
#define ZC_FILTER_MIN        2
#define ZC_FILTER_MAX        12

typedef struct {
    volatile uint16_t throttle;          // 0..2000 (DShot 48..2047 mapped)
    volatile bool     armed;
    volatile bool     running;
    volatile bool     bemf_closed_loop;
    volatile uint8_t  step;              // 1..6
    volatile bool     forward;
    volatile uint32_t com_interval;      // ticks per 60 electrical degrees
    volatile uint32_t last_zc_tick;
    volatile uint16_t zc_filter;
    volatile uint16_t duty;              // 0..PWM_ARR
    volatile uint8_t  advance_deg;
    volatile bool     damping_light;     // complementary PWM
    volatile bool     brake_on_stop;
    volatile uint8_t  drag_brake_pct;    // 0..100
    volatile uint32_t erpm;
    volatile uint16_t desync_count;
    volatile bool     bidir_dshot;
} esc_state_t;

extern esc_state_t esc;
```

---

## 3. High-Resolution DShot1200 Decoding

### 3.1 Capture Strategy

TIM3 runs unprescaled at 64 MHz, giving 15.6 ns resolution. That is about 53 counts per bit, so each bit can be classified with a wide margin.

The timer captures **both edges**. DMA stores 32 timestamps (16 rising and 16 falling edges). For each bit, the decoder computes:

```
high_time = t_fall - t_rise
```

A `1` has a ~40-count high time and a `0` has ~20. The decision threshold is half the bit period (~26 counts).

The decoder is self-calibrating: it measures the actual bit period from the frame itself. This tolerates flight-controller clock error and works for DShot300, 600, and 1200 alike.

```c
/* dshot.c */
#include "esc_config.h"
#include "stm32g0xx.h"

#define DSHOT_BUF_LEN  DSHOT_FRAME_EDGES
static volatile uint16_t dma_buf[DSHOT_BUF_LEN];
static volatile uint32_t dshot_good = 0, dshot_bad = 0;
static volatile uint32_t input_timeout = 0;
static volatile uint16_t pending_cmd = 0;
static volatile uint8_t  cmd_repeat = 0;

static void dshot_process_command(uint16_t cmd);

void dshot_capture_init(void)
{
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    RCC->AHBENR  |= RCC_AH