# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is a reference implementation that follows AM32's architecture: STM32F051/G071-class MCU, TIM1 for 3-phase PWM, a comparator for back-EMF, and timer capture with DMA for input. It is a coherent, buildable design pattern, **not a verbatim copy of the AM32 repository**. Pin and peripheral mappings must match your target's `targets.h`.

Always test with a current-limited bench supply and props removed.

---

## 0. System Constants and Shared State

```c
// esc_config.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ            48000000UL
#define PWM_FREQ_HZ            24000
#define TIM1_ARR               ((CPU_FREQ_HZ / PWM_FREQ_HZ) - 1)   // 1999
#define DEAD_TIME_TICKS        60          // ~1.25us @48MHz, tune per FET

#define COM_TIMER              TIM14       // schedules commutation
#define INTERVAL_TIMER         TIM2        // measures ZC-to-ZC, 1 tick = 0.5us
#define INTERVAL_PRESCALER     23          // 48MHz/24 = 2MHz

#define MOTOR_POLES            14
#define TIMING_ADVANCE_STEPS   2           // 0..3 → 0, 7.5, 15, 22.5 deg (x7.5°)
#define MIN_DUTY               (TIM1_ARR / 20)
#define STARTUP_DUTY           (TIM1_ARR / 12)

#define DSHOT_FRAME_EDGES      32          // 16 bits, rising+falling each

typedef enum { PHASE_A, PHASE_B, PHASE_C } phase_t;

extern volatile uint8_t  step;             // 1..6
extern volatile uint32_t commutation_interval;
extern volatile uint16_t duty_cycle;
extern volatile bool     running, forward, comp_pwm, brake_on_stop;
extern volatile uint16_t newinput;         // 0..2047 throttle domain
```

---

## 1. Power Stage Primitives (Complementary PWM / Damped Light)

AM32's "damped light" is synchronous complementary PWM. During the PWM off-time, the low-side FET of the driven phase turns on instead of letting current freewheel through the body diode. Two consequences follow:

- Deceleration becomes active (regenerative braking) when throttle drops.
- Response is more linear and FETs run cooler because the body diodes conduct less.

Each phase supports three states:

```c
// phaseouts.c
#include "esc_config.h"
#include "stm32f0xx.h"

// TIM1 CH1/CH1N = A, CH2/CH2N = B, CH3/CH3N = C
// OCxM = 110 (PWM1); CCxE / CCxNE gate high/low outputs.

static inline void phase_pwm(phase_t p) {
    // High side PWM; low side complementary (comp_pwm) or forced off
    uint32_t ccer = TIM1->CCER;
    uint32_t sh = p * 4;
    ccer |=  (TIM_CCER_CC1E << sh);
    if (comp_pwm) ccer |=  (TIM_CCER_CC1NE << sh);
    else          ccer &= ~(TIM_CCER_CC1NE << sh);
    TIM1->CCER = ccer;
    volatile uint32_t *ccmr = (p < 2) ? &TIM1->CCMR1 : &TIM1->CCMR2;
    uint32_t off = (p == PHASE_B) ? 8 : 0;
    *ccmr = (*ccmr & ~(7u << (4 + off))) | (6u << (4 + off));      // PWM mode 1
}

static inline void phase_low(phase_t p) {
    // Force OCxREF inactive → high off, low ON (complementary output active)
    uint32_t sh = p * 4;
    TIM1->CCER |= (TIM_CCER_CC1E << sh) | (TIM_CCER_CC1NE << sh);
    volatile uint32_t *ccmr = (p < 2) ? &TIM1->CCMR1 : &TIM1->CCMR2;
    uint32_t off = (p == PHASE_B) ? 8 : 0;
    *ccmr = (*ccmr & ~(7u << (4 + off))) | (4u << (4 + off));      // force inactive
}

static inline void phase_float(phase_t p) {
    uint32_t sh = p * 4;
    TIM1->CCER &= ~((TIM_CCER_CC1E << sh) | (TIM_CCER_CC1NE << sh));
}

void all_off(void)        { phase_float(PHASE_A); phase_float(PHASE_B); phase_float(PHASE_C); }
void full_brake(void)     { phase_low(PHASE_A);   phase_low(PHASE_B);   phase_low(PHASE_C); }

void pwm_init(void) {
    TIM1->PSC  = 0;
    TIM1->ARR  = TIM1_ARR;
    TIM1->BDTR = DEAD_TIME_TICKS | TIM_BDTR_MOE | TIM_BDTR_OSSR;   // dead time + run-state off
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;              // preload
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM1->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

void set_duty(uint16_t d) {
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = d;
}
```

Notes on these primitives:

- **`OSSR`** keeps outputs driven at their inactive level when disabled. Without it, `phase_low` would not hold the low FET on cleanly.
- **Dead time** is mandatory with complementary PWM. If it is too short you get shoot-through; if it is too long you lose efficiency and get zero-cross distortion.

---

## 2. Six-Step Commutation Table and BEMF Comparator Routing

```c
// commutate.c
#include "esc_config.h"
extern void phase_pwm(phase_t), phase_low(phase_t), phase_float(phase_t);

volatile uint8_t step = 1;
volatile bool    rising;           // expected BEMF edge direction on floating phase

// step : PWM phase, LOW phase, FLOAT phase, BEMF edge
static const struct { phase_t hi, lo, fl; bool rise; } comm_table[7] = {
    {0},
    { PHASE_A, PHASE_B, PHASE_C, false }, // 1: AB
    { PHASE_C, PHASE_B, PHASE_A, true  }, // 2: CB
    { PHASE_C, PHASE_A, PHASE_B, false }, // 3: CA
    { PHASE_B, PHASE_A, PHASE_C, true  }, // 4: BA
    { PHASE_B, PHASE_C, PHASE_A, false }, // 5: BC
    { PHASE_A, PHASE_C, PHASE_B, true  }, // 6: AC
};

static void comp_select(phase_t floating, bool rise) {
    // Route comparator inverting input to the floating phase via mux / GPIO EXTI
    // Target-specific: COMP1 INM select or external divider network.
    COMP->CSR = (COMP->CSR & ~COMP_CSR_COMP1INSEL) | comp_insel_for(floating);
    EXTI->RTSR = rise ? COMP_EXTI_LINE : 0;
    EXTI->FTSR = rise ? 0 : COMP_EXTI_LINE;
}

void com_step(uint8_t s) {
    const typeof(comm_table[0]) *c = &comm_table[s];
    phase_float(c->fl);          // float first to avoid transient shoot-through
    phase_low(c->lo);
    phase_pwm(c->hi);
    rising = c->rise;
    comp_select(c->fl, c->rise);
}

void commutate(void) {
    if (forward) step = (step == 6) ? 1 : step + 1;
    else         step = (step == 1) ? 6 : step - 1;
    com_step(step);
}
```

---

## 3. Motor Timing: Zero-Cross → Advanced Commutation

The timing model works as follows:

- Commutation should ideally occur **30° electrical after the zero crossing**. One step is 60° electrical, so the ideal delay is `interval/2`.
- Timing advance subtracts `advance × 7.5°`, which is `interval/8` per step.
- Blanking ignores the comparator just after commutation to reject demagnetization (flyback) spikes.
- A filter requires N consistent comparator samples. N scales down as RPM rises.

```c
// motor_timing.c
#include "esc_config.h"

volatile uint32_t commutation_interval = 10000;  // 0.5us ticks
volatile uint32_t average_interval     = 10000;
volatile uint32_t zc_timestamp, last_zc;
volatile uint16_t advance, wait_time;
volatile uint8_t  filter_level = 8;
volatile uint8_t  bad_count;
volatile bool     zc_found, blanking;
extern volatile bool rising;
extern void commutate(void);

static inline bool comp_out(void) { return (COMP->CSR & COMP_CSR_COMP1OU