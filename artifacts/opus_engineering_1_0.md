# AM32-Style ESC Firmware, Cycle 1: Commutation Timing, DShot1200 Decoding, and Active Braking

This is an original implementation in the style of AM32's architecture. It targets the **STM32G071** at 64 MHz using CMSIS register access. You can port it to F051/L431 by changing the peripheral mappings. AM32 itself is GPL-3.0. If you merge this into that codebase, the result follows its license.

---

## 0. System Architecture

```
            ┌───────────────┐   TIM3 IC + DMA (both edges)
 DShot ───► │ dshot.c       │──► throttle / commands / telemetry req
            └──────┬────────┘
                   ▼
            ┌───────────────┐   TIM1: 3× complementary PWM (24 kHz)
            │ control loop  │──► duty, braking mode
            └──────┬────────┘
                   ▼
            ┌───────────────┐   COMP1 (BEMF vs virtual neutral)
            │ commutation.c │◄── zero-cross IRQ
            │               │──► TIM14 one-shot = commutation delay
            └───────────────┘   TIM2 = free-running 1 MHz interval timer
```

| Timer | Role | Clock |
|---|---|---|
| TIM1 | 3-phase complementary PWM with dead time | 64 MHz |
| TIM2 | Commutation interval measurement (32-bit) | 1 MHz (PSC = 63) |
| TIM3 CH1 | DShot input capture, both edges, via DMA | 64 MHz |
| TIM14 | Commutation timeout and delay scheduler | 1 MHz |
| TIM16 | 20 kHz control loop (ramping, braking, failsafe) | 1 MHz |

---

## 1. `config.h`

```c
#pragma once
#include "stm32g0xx.h"
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ          64000000UL
#define PWM_FREQ_HZ          24000UL
#define PWM_ARR              ((CPU_FREQ_HZ / PWM_FREQ_HZ) - 1)   // 2665
#define DEAD_TIME_TICKS      40          // ~625 ns, tune to FET gate driver

#define MOTOR_POLES          14
#define TIMING_ADVANCE       2           // 0..4 units of 7.5° (2 = 15°)
#define STARTUP_DUTY         (PWM_ARR / 12)
#define MIN_DUTY             (PWM_ARR / 40)
#define MAX_DUTY             PWM_ARR

#define DSHOT_BITS           16
#define DSHOT_EDGES          (DSHOT_BITS * 2)
#define DSHOT_SIGNAL_TIMEOUT_MS 250

// Braking
#define BRAKE_ON_STOP        1
#define BRAKE_STRENGTH       (PWM_ARR / 2)   // drag brake duty at zero throttle
#define USE_COMP_PWM         1               // damped light / sync rectification
```

---

## 2. Motor Timing and Commutation (`commutation.c`)

### 2.1 Timing Theory

- One zero crossing (ZC) occurs every **60° electrical**. TIM2 measures the period between ZCs. The result is called `commutation_interval` (µs).
- The ideal commutation point is **30° after the ZC**, which is `interval / 2`.
- Each advance step equals **7.5° = interval / 8**.
- The delay is therefore:

  `delay = interval/2 − advance·interval/8`

- **Blanking:** after each commutation, the comparator output is ignored for a short window. The demagnetization spike and PWM switching noise would otherwise cause false ZCs.
- **Filtering:** the comparator must agree for N consecutive samples. N shrinks as RPM rises.

```c
#include "config.h"

typedef enum { PHASE_A, PHASE_B, PHASE_C } phase_t;

volatile uint8_t  step = 1;                  // 1..6
volatile uint32_t commutation_interval = 10000;  // µs per 60° (filtered)
volatile uint32_t last_zc_time = 0;
volatile uint32_t zc_timestamps[6];
volatile bool     running = false;
volatile bool     rising_bemf;
volatile uint8_t  bad_count = 0;
volatile uint16_t duty_cycle = 0;
volatile uint8_t  filter_level = 8;
extern volatile bool comp_pwm_enabled;

/* ---------------- Phase drive primitives ----------------
 * Each phase has 3 states:
 *   PWM   : high side PWM (+ complementary low side if comp_pwm)
 *   LOW   : low side fully on
 *   FLOAT : both off (BEMF sensing)
 * CCER bits on TIM1: CCxE = high side, CCxNE = low side.
 * Force-low uses OCxM = "forced inactive" with CCxNE enabled, so the low side conducts.
 */
static inline void phase_pwm(phase_t p) {
    uint32_t sh = p * 4;
    volatile uint32_t *ccmr = (p < 2) ? &TIM1->CCMR1 : &TIM1->CCMR2;
    uint32_t ofs = (p == PHASE_B) ? 8 : 0;
    *ccmr = (*ccmr & ~(0x7UL << (4 + ofs))) | (0x6UL << (4 + ofs));  // PWM mode 1
    TIM1->CCER |=  (TIM_CCER_CC1E << sh);
    if (comp_pwm_enabled) TIM1->CCER |=  (TIM_CCER_CC1NE << sh);  // damped light
    else                  TIM1->CCER &= ~(TIM_CCER_CC1NE << sh);
}

static inline void phase_low(phase_t p) {
    uint32_t sh = p * 4;
    volatile uint32_t *ccmr = (p < 2) ? &TIM1->CCMR1 : &TIM1->CCMR2;
    uint32_t ofs = (p == PHASE_B) ? 8 : 0;
    *ccmr = (*ccmr & ~(0x7UL << (4 + ofs))) | (0x4UL << (4 + ofs));  // forced inactive
    TIM1->CCER |= (TIM_CCER_CC1E << sh) | (TIM_CCER_CC1NE << sh);     // OCxN = active
}

static inline void phase_float(phase_t p) {
    TIM1->CCER &= ~((TIM_CCER_CC1E | TIM_CCER_CC1NE) << (p * 4));
}

/* Comparator input mux: select floating phase vs virtual neutral */
static void comp_select(phase_t floating) {
    static const uint32_t inp_sel[3] = { 0x0, 0x1, 0x2 }; // map to board pins
    COMP1->CSR = (COMP1->CSR & ~COMP_CSR_INPSEL) |
                 (inp_sel[floating] << COMP_CSR_INPSEL_Pos);
}

/* Six-step table: {PWM phase, LOW phase, FLOAT phase, BEMF rising?} */
static const struct { phase_t hi, lo, fl; bool rising; } steps[7] = {
    {0},
    { PHASE_A, PHASE_B, PHASE_C, false },
    { PHASE_C, PHASE_B, PHASE_A, true  },
    { PHASE_C, PHASE_A, PHASE_B, false },
    { PHASE_B, PHASE_A, PHASE_C, true  },
    { PHASE_B, PHASE_C, PHASE_A, false },
    { PHASE_A, PHASE_C, PHASE_B, true  },
};

void commutate(void) {
    step = (step >= 6) ? 1 : step + 1;
    __disable_irq();
    phase_float(steps[step].fl);
    phase_low  (steps[step].lo);
    phase_pwm  (steps[step].hi);
    TIM1->EGR = TIM_EGR_COMG;           // apply preloaded CCER atomically
    __enable_irq();
    comp_select(steps[step].fl);
    rising_bemf = steps[step].rising;
}

/* ---------- Commutation scheduling ---------- */
static inline uint32_t advance_delay(uint32_t interval) {
    return (interval >> 1) - ((interval * TIMING_ADVANCE) >> 3);
}

static inline void schedule_commutation(uint32_t delay_us) {
    TIM14->CNT = 0;
    TIM14->ARR = (delay_us > 2) ? delay_us : 2;
    TIM14->SR  = 0;
    TIM14->DIER |= TIM_DIER_UIE;
}

/* ----------