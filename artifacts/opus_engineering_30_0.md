# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is an original reference implementation that follows AM32's architecture: comparator-based back-EMF zero-cross detection, timer-scheduled commutation, DMA-captured DShot, and complementary PWM. It targets an **STM32G071 at 64 MHz** with a 3-phase gate driver. Peripheral access goes through thin macros so you can port it to F051, L431 or AT32 targets.

---

## 1. System Architecture

| Function | Peripheral | Clock / Rate |
|---|---|---|
| 3-phase PWM, complementary with dead-time | TIM1 CH1–3 / CH1N–3N | 64 MHz, ARR=2666 → 24 kHz (center-aligned off) |
| Commutation interval measurement | TIM2 (free-running) | 1 MHz (PSC=63), 1 µs/tick |
| Commutation scheduler (one-shot) | TIM14 | 1 MHz |
| DShot input capture (both edges) + DMA | TIM3 CH1 → DMA1 Ch1 | 64 MHz, PSC=0 (15.6 ns resolution) |
| Back-EMF zero-cross | COMP1/COMP2 + EXTI | Virtual-neutral vs floating phase |
| Control loop (ramping, braking, telemetry) | TIM6 | 20 kHz |

**Interrupt priorities (highest first):**
1. Comparator zero-cross (0)
2. Commutation timer (0)
3. DShot DMA transfer-complete (1)
4. Control loop (2)

Commutation must never be delayed by signal decoding.

---

## 2. Global Definitions

```c
#include <stdint.h>
#include <stdbool.h>

#define CPU_HZ              64000000UL
#define PWM_ARR             2666u          // 24 kHz
#define DEAD_TIME_TICKS     40u            // ~625 ns, set per FET/driver
#define TIMER_TICK_US       1u             // TIM2/TIM14 at 1 MHz

#define DSHOT_FRAME_EDGES   32u
#define DSHOT_MIN_THROTTLE  48u
#define DSHOT_MAX_THROTTLE  2047u

typedef enum { DIR_FWD = 0, DIR_REV = 1 } dir_t;

typedef struct {
    // --- Timing ---
    volatile uint32_t commutation_interval;   // µs per 60° electrical (filtered)
    volatile uint32_t last_zc_time;
    volatile uint32_t zc_timestamp;
    volatile uint8_t  step;                   // 1..6
    volatile bool     running;
    volatile bool     bemf_valid;             // closed-loop sensorless
    volatile uint16_t zc_good_count;
    uint8_t           advance_deg_x10;        // timing advance, 0.1° units (e.g. 150 = 15°)
    bool              auto_advance;

    // --- Throttle / braking ---
    volatile uint16_t input;                  // 0..2047 normalised
    volatile uint16_t duty;                   // 0..PWM_ARR
    bool              comp_pwm;               // damped light (synchronous rectification)
    bool              brake_on_stop;
    uint8_t           drag_brake_strength;    // 0..10
    bool              bidirectional;
    dir_t             dir;

    // --- DShot ---
    volatile bool     dshot_bidir;
    volatile uint16_t dshot_errors;
    volatile uint32_t dshot_good;
} esc_t;

static esc_t esc;
```

---

## 3. High-Resolution DShot1200 Decoding

### 3.1 Protocol Timing at 64 MHz (no prescaler)

| Parameter | DShot1200 | Ticks @ 64 MHz |
|---|---|---|
| Bit period | 0.833 µs | 53.3 |
| T1H ("1" high time) | 0.625 µs (75%) | 40 |
| T0H ("0" high time) | 0.3125 µs (37.5%) | 20 |
| Frame (16 bits) | 13.3 µs | ~853 |

A resolution of 15.6 ns gives about 20 ticks of margin between a 0 and a 1. The decoder does not use fixed thresholds. It compares each high-time against that bit's measured period. This makes it work across DShot150 to DShot1200 without reconfiguration (auto-detect) and tolerates FC clock drift.

### 3.2 Capture Setup

TIM3 CH1 captures **both edges** into a 32-entry buffer via DMA. When the transfer completes, the ISR decodes the frame and immediately re-arms the DMA.

```c
static volatile uint16_t dshot_edges[DSHOT_FRAME_EDGES + 1];

static void dshot_arm_capture(void)
{
    DMA_DISABLE(DMA_CH_DSHOT);
    TIM3->CNT = 0;
    DMA_SET(DMA_CH_DSHOT, &TIM3->CCR1, dshot_edges, DSHOT_FRAME_EDGES);
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP; // both edges
    TIM3->DIER |= TIM_DIER_CC1DE;
    DMA_ENABLE(DMA_CH_DSHOT);
}
```

**Bidirectional DShot** idles high and the signal is inverted. Edge 0 is then a falling edge, but the decode below only uses differences between edges, so it works either way. The `invert` flag selects which interval counts as the "high" pulse.

### 3.3 Decoder

```c
static inline uint8_t dshot_crc(uint16_t v12_4) // v = 12-bit value + telem bit
{
    return (v12_4 ^ (v12_4 >> 4) ^ (