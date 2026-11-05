# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking (Cycle 4)

This is an original, AM32-compatible implementation of three subsystems:

1. **Commutation timing**: zero-cross detection, timing advance, blanking and demag handling.
2. **DShot1200 decoding**: edge-capture DMA, adaptive bit thresholding, CRC checking, and the bidirectional eRPM reply.
3. **Braking**: complementary-PWM damping ("damping light"), drag brake and brake-on-stop.

It targets an STM32G071 at 64 MHz, the most common AM32 MCU. Peripheral access goes through thin `hal_*` wrappers, so porting to F051, L431 or AT32 only touches `hal_*.c`.

---

## 0. Hardware Map and Global Configuration

```c
/* config.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ            64000000UL

/* TIM1: 3-phase complementary PWM, center-aligned off, up-counting */
#define PWM_FREQ_HZ            24000UL
#define PWM_ARR                ((CPU_FREQ_HZ / PWM_FREQ_HZ) - 1)   /* 2665 */
#define DEADTIME_NS            300            /* tune per FET/driver */

/* TIM2: free-running 1 MHz interval timer (32-bit) */
/* TIM14: one-shot commutation delay timer, 1 MHz */
/* TIM3 CH1: DShot input capture (both edges), no prescaler (64 MHz) */
/* COMP1/COMP2: back-EMF comparator, inputs muxed per step */

typedef struct {
    uint8_t  timing_advance_deg;   /* 0..30 electrical degrees, AM32 default 15 */
    bool     comp_pwm;             /* damping light: complementary PWM */
    bool     brake_on_stop;
    uint8_t  drag_brake_strength;  /* 1..10, 10 = strongest */
    uint8_t  brake_on_stop_strength; /* 1..10 */
    bool     bidirectional_dshot;
    bool     dir_reversed;
    bool     mode_3d;
    uint16_t min_duty;             /* idle duty in PWM_ARR counts */
    uint8_t  motor_poles;
} esc_config_t;

extern esc_config_t cfg;
```

---

## 1. Motor Timing / Six-Step Commutation

### 1.1 Timing model

- One commutation step is 60° electrical. `comm_interval` is measured in µs between successive zero crossings (ZC).
- For a trapezoidal BLDC drive, the ZC occurs 30° before the ideal commutation point.
- With timing advance *A*, commutation is scheduled after the ZC by:

  ```
  delay = comm_interval * (30 - A) / 60
  ```

- Blanking prevents the comparator from seeing commutation ringing and demag spikes. The comparator is ignored for:

  ```
  blank = max(BLANK_MIN_US, comm_interval / 8)
  ```

- ZC filtering requires N consecutive comparator reads in the expected state. N shrinks as speed rises, because there is less time per step.

### 1.2 Commutation table

| Step | High (PWM) | Low (ON) | Float (sensed) | BEMF edge |
|------|-----------|----------|----------------|-----------|
| 1 | A | B | C | rising |
| 2 | A | C | B | falling |
| 3 | B | C | A | rising |
| 4 | B | A | C | falling |
| 5 | C | A | B | rising |
| 6 | C | B | A | falling |

### 1.3 Code

```c
/* commutation.c */
#include "config.h"
#include "hal.h"

typedef enum { PH_A = 0, PH_B, PH_C } phase_t;
typedef enum { PS_FLOAT, PS_PWM, PS_LOW } phase_state_t;

typedef struct { phase_t hi, lo, fl; bool rising; } step_t;

static const step_t STEP_TABLE[6] = {
    {PH_A, PH_B, PH_C, true }, {PH_A, PH_C, PH_B, false},
    {PH_B, PH_C, PH_A, true }, {PH_B, PH_A, PH_C, false},
    {PH_C, PH_A, PH_B, true }, {PH_C, PH_B, PH_A, false},
};

#define BLANK_MIN_US          20
#define STARTUP_ZC_REQUIRED   30
#define MAX_COMM_INTERVAL_US  20000   /* below this speed -> desync / restart */
#define MIN_COMM_INTERVAL_US  12      /* ~ 830k eRPM ceiling */

volatile uint8_t  step = 0;              /* 0..5 */
volatile uint32_t comm_interval = 4000;  /* us, filtered */
volatile uint32_t last_zc_time;
volatile bool     running = false;
volatile bool     closed_loop = false;
volatile uint16_t zc_good_count = 0;
volatile uint16_t duty_cmd = 0;          /* 0..PWM_ARR */

static volatile bool zc_armed = false;
static uint8_t filter_level = 8;

/* ---------------- phase drive ---------------- */
static void set_phase(phase_t p, phase_state_t s)
{
    switch (