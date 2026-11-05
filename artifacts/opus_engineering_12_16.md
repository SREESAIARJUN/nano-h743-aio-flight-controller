# AM32-Style ESC Firmware: Commutation Timing, DShot1200 Decoding, and Active Braking

This is a self-contained, register-level reference implementation in the architecture of AM32. It targets an **STM32G071 at 64 MHz**, with TIM1 driving the bridge, COMP1/2 doing back-EMF sensing, TIM3 capturing DShot, and TIM2 as the commutation timebase.

It is **not the literal AM32 source**. It is a clean re-derivation of the same techniques, written so every timing number can be traced. Adapt the pin and peripheral mapping to your target (F051, G071, L431, AT32F421, …).

"Cycle 12" has no defined meaning that I can reference. I've treated it as a request for the full, consolidated version of these three subsystems.

---

## 0. System Timing Budget

| Item | Value | Derivation |
|---|---|---|
| CPU / timer clock | 64 MHz | 15.625 ns/tick |
| PWM frequency | 32 kHz edge-aligned | ARR = 1999 → 2000 steps |
| Dead time | ~470 ns | DTG = 30 × 15.625 ns |
| Commutation timebase | 2 MHz (0.5 µs/tick) | TIM2 PSC = 31 |
| DShot1200 bit period | 0.8333 µs | 53.3 ticks @ 64 MHz |
| DShot1200 T0H / T1H | 0.3125 / 0.625 µs | 20 / 40 ticks |
| DShot1200 frame | 13.33 µs | 16 bits |
| Bidir telemetry bit | 0.6667 µs | 5/4 × 1200 kbit/s = 1500 kbit/s |

---

## 1. Shared Header — `esc.h`

```c
#ifndef ESC_H
#define ESC_H
#include "stm32g0xx.h"
#include <stdint.h>
#include <stdbool.h>

#define PWM_ARR              1999u
#define PWM_MAX              PWM_ARR
#define DEAD_TIME_DTG        30u          /* ~470 ns @ 64 MHz */
#define COMM_TICK_NS         500u         /* TIM2 tick = 0.5 us */

#define MIN_DUTY_RUNNING     60u          /* ~3 % keeps BEMF readable */
#define STARTUP_DUTY         120u
#define MAX_DUTY_RAMP_STEP   8u           /* per 1 kHz control tick */

/* ---- motor state ---- */
typedef enum { MOTOR_STOPPED, MOTOR_STARTING, MOTOR_RUNNING, MOTOR_BRAKING } motor_state_t;

typedef struct {
    volatile motor_state_t state;
    volatile uint8_t  step;               /* 1..6 */
    volatile bool     forward;
    volatile uint32_t comm_interval;      /* filtered, 0.5 us ticks (60 deg elec) */
    volatile uint32_t last_zc_time;
    volatile uint16_t zc_good_count;
    volatile uint16_t desync_count;
    volatile uint16_t duty;               /* applied duty 0..PWM_MAX */
    volatile uint16_t duty_target;
    uint8_t  advance_deg;                 /* 0..30 electrical degrees */
    bool     comp_pwm;                    /* "damped light" synchronous rectification */
    bool     brake_on_stop;
    uint16_t drag_brake_strength;         /* 0..PWM_MAX */
} motor_t;

extern motor_t motor;

/* ---- dshot ---- */
typedef struct {
    volatile uint16_t throttle;           /* 0..2047 raw */
    volatile bool     new_frame;
    volatile bool     telemetry_req;
    volatile uint32_t good_frames, crc_errors, frame_errors;
    bool bidirectional;
} dshot_t;

extern dshot_t dshot;

void bridge_init(void);
void bridge_commutate(uint8_t step);
void bridge_set_duty(uint16_t d);
void bridge_all_float(void);
void bridge_full_brake(void);
void bridge_proportional_brake(uint16_t strength);

void comm_init(void);
void comm_start_motor(void);
void comm_stop_motor(void);
void comm_control_tick_1khz(void);

void dshot_init(void);
void dshot_process(void);
void dshot_send_telemetry(uint32_t erpm_period_us);

#endif
```

---

## 2. Power Stage and Braking — `bridge.c`

### Bridge Output Logic

Each half-bridge (TIM1 CHx = high side, CHxN = low side) has four useful states:

| Phase state | OCxM | CCxE | CCxNE | High FET | Low FET |
|---|---|---|---|---|---|
| **PWM + damped** (comp. PWM) | PWM1 | 1 | 1 | PWM | ~PWM (dead-timed) |
| **PWM, undamped** | PWM1 | 1 | 0 | PWM | off (diode freewheel) |
| **LOW** | Force inactive | 1 | 1 | off | on |
| **FLOAT** | Force inactive | 0 | 0 | off | off (OSSR=1) |

**Damped light** means complementary PWM on the driven phase. During PWM off-time, the low-side FET conducts instead of its body diode. This gives:
- Synchronous rectification, so less heat.
- Regenerative braking whenever the commanded duty is below the BEMF-equivalent duty.

This is what gives fast, symmetric deceleration on multirotors.

**Proportional brake** uses a subtle RM table entry. With `CCxE=0, CCxNE=1, OSSR=1`, the low-side output follows **OCxREF directly, not inverted**. In PWM1 mode the low-side on-fraction is therefore exactly `CCR/(ARR+1)`, and the high side is guaranteed off with no shoot-through possible.

```c
#include "esc.h"

#define OCM_PWM1        (6u)
#define OCM_FORCE_LOW   (4u)

/* CCMR field helpers for channels 1..3 */
static inline void set_ocm(uint8_t ch, uint32_t mode)
{
    switch (ch) {
    case 1: TIM1->CCMR1 = (TIM1->CCMR1 & ~(TIM_CCMR1_OC1M)) | (mode << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; break;
    case 2: TIM1->CCMR1 = (TIM1->CCMR1 & ~(TIM_CCMR1_OC2M)) | (mode << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE; break;
    case 3: TIM1->CCMR2 = (TIM1->CCMR2 & ~(TIM_CCMR2_OC3M)) | (mode << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE; break;
    }
}

static inline uint32_t ccer_mask(uint8_t ch) { return 0x5u << ((ch - 1) * 4); }  /* CCxE|CCxNE */
static inline uint32_t ccxe(uint8_t ch)      { return 0x1u << ((ch - 1) * 4); }
static inline uint32_t ccxne(uint8_t ch)     { return 0x4u << ((ch - 1) * 4); }

/* Phase actions — written to preload; latched atomically by COM event */
static uint32_t ccer_shadow;

static void phase_pwm(uint8_t ch)
{
    set_ocm(ch, OCM_PWM1);
    ccer_shadow &= ~ccer_mask(ch);
    ccer_shadow |= ccxe(ch) | (motor.comp_pwm ? ccxne(ch) : 0);
}
static void phase_low(uint8_t ch)
{
    set_ocm(ch, OCM_FORCE_LOW);
    ccer_shadow |= ccer_mask(ch);
}
static void phase_float(uint8_t ch)
{
    set_ocm(ch, OCM_FORCE_LOW);
    ccer_shadow &= ~ccer_mask(ch);
}

void bridge_init(void)
{