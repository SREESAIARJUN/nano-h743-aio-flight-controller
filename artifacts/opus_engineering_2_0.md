# AM32-Style ESC Core: Motor Timing, DShot1200 Decoding, and Active Braking (Cycle 2)

This is an original reference implementation modeled on the AM32 architecture. It is not a copy of the upstream GPL source. It targets an STM32G071 (64 MHz) with:

- **TIM1** driving three complementary half-bridge pairs.
- **COMP1/COMP2** sensing back-EMF (BEMF) through a phase multiplexer.
- **TIM3** capturing the DShot input via DMA.
- **TIM14** as the commutation scheduler and **TIM2** as a free-running interval timer.

Register access goes through a thin HAL (`hal_*`), so the logic is portable to F051, G431, AT32 and similar parts.

---

## 0. Architecture overview

```
 DShot pin ──► TIM3 CH (both edges) ──DMA──► dma_buf[32] ──► dshot_decode() (DMA-TC ISR)
                                                             │
                                                             ▼
                                       throttle / command ──► control_loop() 20 kHz
                                                             │
     ┌─────────────────────── duty, direction, brake mode ◄──┘
     ▼
  TIM1 PWM (complementary, dead-time) ◄── commutate(step) ◄── TIM14 one-shot (ZC + 30° − advance)
                                                                  ▲
  COMP (floating phase vs. virtual neutral) ──► zc_detected() ────┘
                                                (TIM2 measures 60° intervals)
```

---

## 1. Configuration and shared state (`esc.h`)

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ---------- Clocks ---------- */
#define CPU_HZ              64000000UL
#define PWM_FREQ_HZ         24000UL
#define PWM_ARR             (CPU_HZ / PWM_FREQ_HZ - 1)       /* 2665 */
#define DEADTIME_NS         300
#define INTERVAL_TICK_HZ    2000000UL                         /* TIM2: 0.5 us ticks */

/* ---------- User EEPROM settings ---------- */
typedef struct {
    uint8_t  timing_advance_deg;   /* 0..30 electrical degrees, typ. 15 */
    bool     comp_pwm;             /* damped light / synchronous rectification */
    bool     brake_on_stop;
    uint8_t  brake_strength;       /* 0..10 */
    uint8_t  drag_brake_strength;  /* 0..10, decel braking while spinning */
    bool     bidirectional;        /* 3D mode */
    bool     bidir_dshot;          /* inverted DShot + eRPM telemetry */
    bool     reversed;
    uint16_t min_duty;             /* startup floor, in PWM counts */
    uint8_t  motor_poles;
    uint16_t max_decel_per_tick;   /* duty slew limit, regen protection */
} esc_settings_t;

extern esc_settings_t cfg;

/* ---------- Runtime state ---------- */
typedef enum { PH_FLOAT = 0, PH_LOW, PH_PWM } phase_mode_t;

typedef struct {
    volatile uint8_t  step;                 /* 1..6 */
    volatile bool     running;
    volatile bool     armed;
    volatile bool     forward;
    volatile uint32_t commutation_interval; /* ticks per 60° elec. (avg) */
    volatile uint32_t zc_time;              /* last raw 60° measurement */
    volatile uint16_t zc_found_count;       /* consecutive good ZCs */
    volatile uint16_t bad_zc_count;
    volatile uint16_t duty;                 /* current applied, 0..PWM_ARR */
    volatile uint16_t duty_target;
    volatile uint16_t throttle;             /* 0..2000 normalized */
    volatile bool     zc_interr