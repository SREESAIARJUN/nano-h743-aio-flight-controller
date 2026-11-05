# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking/Damping (Cycle 25)

This is a reference implementation written in the architecture and style of AM32. It is **not a verbatim copy of the upstream AM32 source**. It targets an STM32G071 at 64 MHz, and the peripheral mapping is easy to port to F051, L431 or AT32.

---

## 0. System Architecture

| Function | Peripheral | Clock / Resolution |
|---|---|---|
| 3-phase complementary PWM | TIM1 CH1/2/3 + CH1N/2N/3N | 64 MHz, 24 kHz carrier (ARR = 2666) |
| DShot input capture | TIM3 CH1, both edges, with DMA | 64 MHz (15.6 ns/tick) |
| Commutation interval timer | TIM2 (32-bit) | 2 MHz (0.5 µs/tick) |
| Commutation event scheduler | TIM14 (one-shot) | 2 MHz |
| Back-EMF zero-cross detection | COMP1/COMP2 with EXTI | Async |
| Control loop (ramping, desync check) | TIM16 | 20 kHz |

Interrupt priorities, highest first: **COMP (zero-cross) → TIM14 (commutate) → DMA (DShot) → TIM16 (control loop)**.

---

## 1. Configuration and Shared State

```c
// esc_config.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define TIM1_ARR            2666u      // 64 MHz / 24 kHz
#define DEAD_TIME_TICKS     40u        // ~625 ns, tune to your FET gate charge
#define TIMER_TICK_US_X2    1          // commutation timers run at 2 MHz

#define MIN_DUTY            (TIM1_ARR / 20)    // ~5 % minimum running duty
#define STARTUP_DUTY        (TIM1_ARR / 12)
#define MAX_DUTY            TIM1_ARR

#define POLLING_MODE_INTERVAL  4000u   // >2 ms per step -> use polled BEMF
#define MIN_BLANK_TICKS        8u      // 4 us minimum blanking
#define ZC_FILTER_MAX          12u
#define ZC_FILTER_MIN          2u

typedef struct {
    uint8_t  timing_advance;     // 0..3, each step = 7.5 deg electrical
    bool     comp_pwm;           // damped light / synchronous rectification
    bool     brake_on_stop;
    uint8_t  drag_brake_strength;// 1..10
    bool     bidirectional_dshot;
    bool     reversed;
} esc_settings_t;

extern volatile esc_settings_t settings;

// Commutation state
extern volatile uint8_t  step;                 // 1..6
extern volatile uint32_t commutation_interval; // 0.5 us ticks per 60 deg electrical
extern volatile uint32_t zc_timestamp;
extern volatile bool     running;
extern volatile bool     armed;
extern volatile uint16_t duty_target, duty_current;
extern volatile uint16_t dshot_throttle;       // 0..2047 raw
```

---

## 2. High-Resolution DShot1200 Decoding

### 2.1 Protocol Timing

| Rate | Bit period | T1H (75 %) | T0H (37.5 %) | Ticks @ 64 MHz (bit / 1 / 0) |
|---|---|---|---|---|
| DShot300 | 3.333 µs | 2.50 µs | 1.25 µs | 213 / 160 / 80 |
| DShot600 | 1.667 µs | 1.25 µs | 0.625 µs | 107 / 80 / 40 |
| DShot1200 | 0.833 µs | 0.625 µs | 0.3125 µs | 53 / 40 / 20 |

At DShot1200 there are only about 20 ticks between a 0 and a 1. To stay robust against clock skew and edge jitter, the decoder **measures each bit's own period** and classifies the bit by the ratio of high time to period. A fixed threshold is not used. The decision boundary is the midpoint of 37.5 