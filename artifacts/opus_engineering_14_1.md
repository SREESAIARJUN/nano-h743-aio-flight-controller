# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is an original, AM32-architecture-compatible implementation targeting STM32G071/F051-class MCUs at 64 MHz. It is not copied from the AM32 repository. The structure, timing math, and protocol handling follow the same principles, so it should drop into an AM32-like project layout.

I don't have context for what "Cycle 14" refers to in your series. I've treated it as the next full revision and made each module self-contained.

---

## 1. System Architecture

```
            ┌────────────────────┐
 DShot ───► │ TIM15 CH1 IC + DMA │──► dshot_decode() ──► throttle / commands
            └────────────────────┘
                                         │
                                         ▼
 ┌────────────┐  ZC IRQ   ┌───────────────────────┐  TIM14 one-shot  ┌──────────────┐
 │ COMP (BEMF)│ ────────► │ zero_cross_handler()  │ ───────────────► │ commutate()  │
 └────────────┘           │ interval + advance    │                  │ 6-step table │
                          └───────────────────────┘                  └──────┬───────┘
                                                                            ▼
                                     TIM1 3-phase complementary PWM (damped light)
```

**Timer allocation:**

| Timer | Role | Clock |
|---|---|---|
| TIM1 | 3-phase PWM, CH1-3 + CH1N-3N, dead-time | 64 MHz |
| TIM2 | Free-running interval timer (commutation period) | 1 MHz (0.5 µs at /32 option) |
| TIM14 | Commutation delay one-shot | 1 MHz |
| TIM15 | DShot input capture (both edges) + DMA | 64 MHz |
| TIM16 | 20 kHz control loop / timeouts | — |

---

## 2. Shared Definitions (`esc.h`)

```c
#ifndef ESC_H
#define ESC_H
#include <stdint.h>
#include <stdbool.h>

#define CPU_HZ                64000000UL
#define PWM_FREQ_HZ           24000UL
#define PWM_ARR               ((CPU_HZ / PWM_FREQ_HZ) - 1)   /* 2665 */
#define DEAD_TIME_TICKS       60      /* ~940 ns; tune to FET gate charge */

#define INTERVAL_TIMER_HZ     1000000UL   /* TIM2 / TIM14 tick = 1 us */

/* Throttle */
#define DSHOT_THROTTLE_MIN    48
#define DSHOT_THROTTLE_MAX    2047
#define MIN_DUTY              (PWM_ARR / 20)      /* ~5% idle */
#define MAX_DUTY              PWM_ARR

/* Commutation limits (in 1 us ticks, per 60 deg electrical step) */
#define MIN_COMM_INTERVAL     20          /* ~500k eRPM ceiling */
#define MAX_COMM_INTERVAL     20000       /* below this speed we are "stalled" */

typedef enum { ADV_0 = 0, ADV_7_5, ADV_15, ADV_22_5, ADV_30 } timing_adv_t;

typedef struct {
    /* config (EEPROM) */
    timing_adv_t advance;
    bool     comp_pwm;            /* damped light / synchronous rectification */
    bool     brake_on_stop;
    uint16_t brake_strength;      /* 0..PWM_ARR, low-side duty at stop */
    uint8_t  drag_brake_strength; /* 1..10 */
    bool     bidirectional_dshot;
    bool     reversed;

    /* runtime */
    volatile uint8_t  step;               /* 1..6 */
    volatile bool     running;
    volatile bool     rising;             /* BEMF edge expected */
    volatile uint32_t comm_interval;      /* filtered, us per step */
    volatile uint32_t last_zc_tick;
    volatile uint16_t duty;
    volatile uint16_t throttle;           /* 0..2047 */
    volatile uint16_t zc_filter_level;
    volatile uint8_t  bad_zc_count;
    volatile uint32_t e_period;           /* full electrical period, us */
} esc_t;

extern esc_t esc;

/* phase.c */
void phase_set_step(uint8_t step, bool comp_pwm);
void phase_all_float(void);
void phase_all_low(void);
void phase_brake_proportional(uint16_t duty);
void pwm_set_duty(uint16_t duty);

/* comparator.c */
void comp_select_phase(uint8_t step, bool rising);
bool comp_output(void);

/* dshot.c */
void dshot_init(void);
void dshot_process_frame(void);
void dshot_send_telemetry(uint32_t e_period_us);

#endif
```

---

## 3. Six-Step Commutation and Motor Timing (`motor.c`)

### 3.1 Timing Math

Each commutation step spans 60° electrical. In an ideal BLDC, the BEMF zero crossing of the floating phase occurs **30° before** the optimal commutation point. With advance angle *A*:

```
delay_after_ZC = 30° − A  =  interval × (30 − A) / 60
```

Using step-quantized advance (7.5° = interval/8):

```
delay = (interval / 2) − (interval / 8) × adv_index
```

| adv_index | Advance | Delay |
|---|---|---|
| 0 | 0° | 30° |
| 2 | 15° | 15° |
| 3 | 22.5° | 7.5° |

### 3.2 Implementation

```c
#include "esc.h"
#include "stm32g0xx.h"

esc_t esc = {
    .advance = ADV_15,
    .comp_pwm = true,
    .brake_on_stop = false,
    .brake_strength = PWM_ARR / 4,
    .drag_brake_strength = 10,
    .bidirectional_dshot = true,
    .step = 1,
};

/* --- Commutation table ---------------------------------------------------
 * step : HIGH(PWM) / LOW(on) / FLOAT(sense)  BEMF edge on float phase
 *  1   :   A  / B  / C   falling
 *  2   :   C  / B  / A   rising
 *  3   :   C  / A  / B   falling
 *  4   :   B  / A  / C   rising
 *  5   :   B  / C  / A   falling
 *  6   :   A  / C  / B   rising
 * ---------------------------------------------------------------------- */
static const bool step_rising[7] = { 0, false, true, false, true, false, true };

static inline uint32_t interval_now(void) { return TIM2->CNT; }

/* Called from TIM14 update IRQ when the advance delay expires */
static void commutate(void)
{
    if (esc.reversed) { esc.step = (esc.step == 1) ? 6 : esc.step - 1; }
    else              { esc.step = (esc.step == 6) ? 1 : esc.step + 1; }

    esc.rising = step_rising[esc.step] ^ esc.reversed;

    phase_set_step(esc.step, esc.comp_pwm);
    comp_select_phase(esc.step, esc.rising);

    /* Blanking: ignore comparator for demag/ringing period after switching.
       ~1/4 of a step, minimum 8 us, scales down at high speed. */
    uint32_t blank = esc.comm_interval >> 2;
    if (blank < 8) blank = 8;
    TIM14->ARR = blank;
    TIM14->CNT = 0;
    TIM14->SR  = 0;
    /* Mark state: next TIM14 update re-enables ZC interrupt instead of commutating */
}

typedef enum { TS_BLANKING, TS_WAIT_COMMUTATE } t14_state_t;
static volatile t14_state_t t14_state;

static void zc_irq_enable(bool en)
{
    if (en) { EXTI->RPR1 = EXTI->FPR1 = (1u << 17); EXTI->IMR1 |=  (1u << 17); }
    else    {                                       EXTI->IMR1 &= ~(1