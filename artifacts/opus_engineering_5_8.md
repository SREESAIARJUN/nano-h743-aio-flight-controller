# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking (Cycle 5)

> **Scope note:** This is a clean-room reference implementation that follows AM32's architecture: STM32F051/G071/AT32F421-class MCU, comparator-based back-EMF zero-cross detection, and timer input-capture DShot. It is not a verbatim copy of the AM32 source. Register names assume STM32 LL/CMSIS. I've read "Cycle 5" as the fifth design iteration, so it includes refinements such as adaptive DShot thresholds, filtered commutation intervals and proportional drag braking.

---

## 1. System Architecture and Timer Allocation

| Resource | Role | Clock |
|---|---|---|
| TIM1 | 3-phase complementary PWM (CH1–3 + CH1N–3N), dead-time | 48–72 MHz |
| TIM2 | Commutation timer: free-running interval measurement | 1 MHz (1 µs/tick) |
| TIM14/TIM16 | Commutation scheduler: one-shot "commutate at" event | 1 MHz |
| TIM3 (or TIM15) CH1 | DShot input capture, both edges, via DMA | Full sysclk (no prescaler) |
| COMP1/COMP2 | Back-EMF comparator (floating phase vs. virtual neutral) | — |
| TIM17 | 20 kHz control loop: throttle ramp, braking, telemetry | — |

### Interrupt priorities

Lower numbers are higher priority.

| Priority | Interrupt | Why |
|---|---|---|
| 0 | Comparator / zero-cross | Timing-critical |
| 0 | Commutation timer | Timing-critical |
| 1 | DMA transfer-complete for DShot | Decode must finish before the next frame |
| 2 | 20 kHz control loop | — |
| 3 | Telemetry UART, EEPROM | — |

---

## 2. Global State and Configuration

```c
// esc_config.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ            48000000UL
#define PWM_FREQ_HZ            24000
#define TIM1_ARR               ((CPU_FREQ_HZ / PWM_FREQ_HZ) - 1)   // ~1999
#define DEAD_TIME_TICKS        40                                   // ~830 ns @48 MHz

#define MOTOR_POLES            14
#define MIN_COMMUTATION_US     40      // ~250k eRPM ceiling
#define MAX_COMMUTATION_US     20000   // below this, treat as stalled
#define BEMF_FILTER_DEFAULT    3       // comparator consistency samples

typedef struct {
    uint8_t  timing_advance;      // 0..4  -> 0°, 7.5°, 15°, 22.5°, 30°
    bool     brake_on_stop;       // hold all low-sides when stopped
    uint8_t  drag_brake_strength; // 0..10, proportional braking on throttle decrease
    bool     complementary_pwm;   // "damped light" synchronous rectification
    bool     bidirectional_dshot;
    bool     reversed_direction;
    uint16_t startup_power;       // duty floor during open-loop start
} esc_settings_t;

extern esc_settings_t settings;

typedef struct {
    volatile uint8_t  step;                   // 1..6
    volatile uint32_t commutation_interval;   // filtered, µs per 60° electrical
    volatile uint32_t last_zc_time;
    volatile uint32_t this_zc_time;
    volatile uint16_t duty_cycle;             // 0..TIM1_ARR
    volatile bool     running;
    volatile bool     zc_found;
    volatile uint8_t  bemf_filter_level;
    volatile uint8_t  bad_zc_count;
    volatile uint32_t e_rpm;                  // electrical RPM / 100
    volatile bool     rising;                 // expected BEMF edge polarity
} motor_state_t;

extern motor_state_t motor;
```

---

## 3. Six-Step Commutation Engine

### 3.1 Phase-state primitives

Each phase can be in one of four states:

- **PWM_COMP:** high-side PWM with complementary low-side (synchronous rectification, "damped light").
- **PWM_HI:** high-side PWM only, low side off (non-complementary).
- **LOW:** low side held on.
- **FLOAT:** both FETs off, so the comparator can sense back-EMF.

```c
// phases.c
#include "esc_config.h"
#include "stm32f0xx.h"

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

static inline void phase_pwm(phase_t p) {
    // OCxM = PWM mode 1, enable CCxE and (optionally) CCxNE
    volatile uint32_t *ccmr = (p == PH_C) ? &TIM1->CCMR2 : &TIM1->CCMR1;
    uint32_t shift = (p == PH_B) ? 8 : 0;
    *ccmr = (*ccmr & ~(0x7U << (4 + shift))) | (0x6U << (4 + shift));
    uint32_t en = TIM_CCER_CC1E << (p * 4);
    if (settings.complementary_pwm) en |= TIM_CCER_CC1NE << (p * 4);
    TIM1->CCER = (TIM1->CCER & ~(0x5U << (p * 4))) | en;
}

static inline void phase_low(phase_t p) {
    // Force inactive on OCx so CCxN (low side) is ON
    volatile uint32_t *ccmr = (p == PH_C) ? &TIM1->CCMR2 : &TIM1->CCMR1;
    uint32_t shift = (p == PH_B) ? 8 : 0;
    *ccmr = (*ccmr & ~(0x7U << (4 + shift))) | (0x4U << (4 + shift)); // force low
    TIM1->CCER = (TIM1->CCER & ~(0x5U << (p * 4)))
               | (TIM_CCER_CC1E << (p * 4)) | (TIM_CCER_CC1NE << (p * 4));
}

static inline void phase_float(phase_t p) {
    TIM1->CCER &= ~(0x5U << (p * 4));   // both outputs disabled -> Hi-Z
}
```

### 3.2 Commutation table and comparator routing

| Step | High (PWM) | Low | Float (sensed) | BEMF edge |
|---|---|---|---|---|
| 1 | A | B | C | falling |
| 2 | A | C | B | rising |
| 3 | B | C | A | falling |
| 4 | B | A | C | rising |
| 5 | C | A | B | falling |
| 6 | C | B | A | rising |

```c
// commutation.c
#include "esc_config.h"

static const uint8_t step_hi[7]    = {0, PH_A, PH_A, PH_B, PH_B, PH_C, PH_C};
static const uint8_t step_lo[7]    = {0, PH_B, PH_C, PH_C, PH_A, PH_A, PH_B};
static const uint8_t step_fl[7]    = {0, PH_C, PH_B, PH_A, PH_C, PH_B, PH_A};
static const bool    step_rising[7]= {0, false, true, false, true, false, true};

static void comparator_select(uint8_t floating_phase, bool rising) {
    // COMP1 inverting input muxed to the floating phase; non-inverting = virtual neutral
    static const uint32_t inm_sel[3] = { COMP_INM_PA4, COMP_INM_PA5, COMP_INM_PA0 }; // board-specific
    COMP->CSR = (COMP->CSR & ~COMP_CSR_COMP1INSEL) | inm_sel[floating_phase];

    // EXTI line 21 (COMP1 output): select edge
    EXTI->RTSR &= ~EXTI_RTSR_TR21;
    EXTI->FTSR &= ~EXTI_FTSR_TR21;
    if (rising) EXTI->RTSR |= EXTI_RTSR_TR21;
    else        EXTI->FTSR |= EXTI_FTSR_TR21;
    motor.rising = rising;
}

void commutate(void) {
    if (settings.reversed_direction) {
        motor.step = (motor.step == 1) ? 6 : motor.step - 1;
    } else {
        motor.step = (motor.step == 6) ? 1 : motor.step + 1;
    }
    uint8_t s = motor.step;

    __disable_irq();                // atomic phase change
    phase_float(step_fl[s]);
    phase_low(step_lo[s]);
    phase_pwm(step_hi[s]);
    TIM1->EGR = TIM_EGR_COMG;       // latch preload (if CCPC used)
    __enable_irq();

    bool rising = step_rising[s] ^ settings.reversed_direction;
    comparator_select(step_fl[s], rising);
    motor.zc_found = false;
}
```

---

## 4. Motor Timing: Zero-Cross Detection and Advance

### 4.1 Timing model

For one 60° electrical sector with interval `T`:

- The zero crossing (ZC) occurs nominally 30° after the previous commutation, which is `T/2`.
- The ideal commutation point is 30° after the ZC, also `T/2`.
- With advance `θ_adv`, commutate at `T/2 − T·θ_adv/60°` after the ZC.

| Advance level | Advance | Delay from ZC to commutation |
|---|---|---|
| 0 | 0° | `T/2` |
| 1 | 7.5° | `T/2 − T/8` |
| 2 | 15° | `T/2 − T/4` |
| 3 | 22.5° | `T/2 − 3T/8` |
| 4 | 30° | 0 (commutate immediately) |

Each level of 7.5° equals `T/8`, so the delay is `(T>>1) − level·(T>>3)`.

**Blanking window:** after commutation, the floating phase rings from the demagnetization (flyback diode conduction). Comparator interrupts are ignored for roughly `T/4`, plus a fixed minimum of about 8 µs at low speed.

### 4.2 Zero-cross ISR and commutation scheduler

```c
// timing.c
#include "esc_config.h"

#define BLANKING_MIN_US 8

static inline uint32_t advance_delay(uint32_t interval) {
    uint32_t half = interval >> 1;
    uint32_t adv  = (interval >> 3) * settings.timing_advance;
    return (adv >= half) ? 1 : (half - adv);
}

// Called when commutation completes: arm blanking, then enable comparator
static void start_blanking(void) {
    uint32_t blank = motor.commutation_interval >> 2;
    if (blank < BLANKING_MIN_US) blank = BLANKING_MIN_US;
    EXTI->IMR &= ~EXTI_IMR_MR21;        // mask comparator
    TIM14->CNT = 0;
    TIM14->ARR = blank;
    TIM14->SR  = 0;
    TIM14->DIER = TIM_DIER_UIE;
    TIM14->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
}

typedef enum { SCHED_BLANKING, SCHED_COMMUTATE } sched_mode_t;
static volatile sched_mode_t sched_mode = SCHED_BLANKING;

// Comparator EXTI ISR: back-EMF zero crossing
void ADC1_COMP_IRQHandler(void) {
    EXTI->PR = EXTI_PR_PR21;
    uint32_t now = TIM2->CNT;

    // Digital filter: confirm comparator state N times (rejects PWM-switching noise)
    for (uint8_t i = 0; i < motor.bemf_filter_level; i++) {
        bool out = (COMP->CSR & COMP_CSR_COMP1OUT) != 0;
        if (out != motor.rising) return;   // glitch, wait for the next edge
    }

    EXTI->IMR &= ~EXTI_IMR_MR21;        // one ZC per sector
    motor.last_zc_time = motor.this_zc_time;
    motor.this_zc_time = now;
    uint32_t raw = now - motor.last_zc_time;   // TIM2 wraps at 32 bit, so subtraction is safe

    // Reject implausible intervals (noise / desync)
    if (raw < (motor.commutation_interval >> 1) || raw > (motor.commutation_interval << 1)) {
        if (++motor.bad_zc_count > 20) { motor.running = false; /* -> restart logic */ }
        raw = motor.commutation_interval;      // coast on previous estimate
    } else if (motor.bad_zc_count) {
        motor.bad_zc_count--;
    }

    // IIR filter: interval = 3/4 old + 1/4 new  (Cycle-5 smoothing)
    motor.commutation_interval = (3 * motor.commutation_interval + raw) >> 2;
    if (motor.commutation_interval < MIN_COMMUTATION_US)
        motor.commutation_interval = MIN_COMMUTATION_US;

    // Adaptive BEMF filter: less filtering at high speed (less time available)
    motor.bemf_filter_level = (motor.commutation_interval < 100) ? 1 :
                              (motor.commutation_interval < 400) ? 2 : BEMF_FILTER_DEFAULT;

    // Schedule commutation
    sched_mode = SCHED_COMMUTATE;
    TIM14->CNT = 0;
    TIM14->ARR = advance_delay(motor.commutation_interval);
    TIM14->SR  = 0;
    TIM14->CR1 |= TIM_CR1_CEN | TIM_CR1_OPM;
    motor.zc_found = true;
}

// One-shot scheduler ISR: either end of blanking, or commutation time
void TIM14_IRQHandler(void) {
    TIM14->SR = 0;
    if (sched_mode == SCHED_COMMUTATE) {
        commutate();
        sched_mode = SCHED_BLANKING;
        start_blanking();
    } else {
        EXTI->PR  = EXTI_PR_PR21;          // clear stale edges captured during blanking
        EXTI->IMR |= EXTI_IMR_MR21;        // arm zero-cross detection
    }
}

// eRPM for telemetry: one electrical revolution = 6 sectors
uint32_t compute_erpm(void) {
    if (!motor.running || motor.commutation_interval == 0) return 0;
    return 60000000UL / (motor.commutation_interval * 6);
}
```

### 4.3 Desync protection and open-loop startup

```c
// startup.c
void motor_start(void) {
    motor.step = 1;
    motor.commutation_interval = 8000;      // 8 ms per sector initial guess
    motor.bemf_filter_level = 6;            // heavy filtering at very low speed
    motor.bad_zc_count = 0;
    motor.running = true;
    TIM2->CNT = 0;
    motor.this_zc_time = 0;
    commutate();
    start_blanking();
}

// Called from 20 kHz loop: if no ZC seen within 2x expected interval,