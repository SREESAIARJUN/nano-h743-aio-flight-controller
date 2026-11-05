# AM32-Style ESC Core: Motor Timing, DShot1200 Decoding, and Active Braking

This is a self-contained reference implementation that follows AM32's architecture and design choices. It is written fresh rather than copied from the AM32 GPL source, so identifiers and structure differ from upstream.

**Assumptions:**
- **MCU:** STM32G071-class, 64 MHz system clock.
- **TIM1:** 3-phase complementary PWM with hardware dead time.
- **TIM2:** 32-bit, 1 MHz free-running timebase for commutation and interval timing.
- **TIM14:** one-shot commutation scheduler.
- **TIM3:** input capture + DMA for DShot.
- **COMP1/2:** back-EMF (BEMF) zero-cross detection.

Port the hardware-abstraction (HAL) macros to your target (F051, F421, L431, and so on). The algorithms are target-independent.

---

## 0. Shared Definitions

```c
// esc_core.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define SYSCLK_HZ            64000000UL
#define PWM_FREQ_HZ          24000UL
#define PWM_ARR              ((SYSCLK_HZ / PWM_FREQ_HZ) - 1)   // 2665
#define DEADTIME_NS          300                                 // gate-driver dependent
#define TIMEBASE_HZ          1000000UL                           // TIM2/TIM14 tick = 1 us

#define DSHOT_EDGES          32          // 16 bits * 2 edges
#define DSHOT_MIN_FRAME_TCK  (SYSCLK_HZ / 1000000UL * 10)  // ~10 us @1200 (13.3 us nominal)
#define DSHOT_MAX_FRAME_TCK  (SYSCLK_HZ / 1000000UL * 120) // covers DShot150..1200

typedef enum { PH_A = 0, PH_B = 1, PH_C = 2 } phase_t;

typedef struct {
    // configuration (EEPROM)
    uint8_t  timing_advance_deg;   // 0..30 electrical degrees (AM32 default ~15-22.5)
    bool     comp_pwm;             // damped light / synchronous rectification
    bool     bidirectional;        // 3D mode
    bool     brake_on_stop;
    uint8_t  drag_brake_pct;       // 0..100
    uint8_t  motor_poles;
    bool     reversed;
    uint16_t min_duty, max_duty;
    uint16_t startup_duty;
} esc_config_t;

typedef struct {
    volatile uint8_t  step;                 // 1..6
    volatile bool     running;
    volatile bool     forward;
    volatile uint32_t commutation_interval; // us per 60 deg electrical
    volatile uint32_t last_zc_time;
    volatile uint32_t zc_timeout;
    volatile uint8_t  good_zc_count;
    volatile uint8_t  bad_zc_count;
    volatile uint16_t duty;                 // 0..PWM_ARR
    volatile uint16_t duty_target;
    volatile bool     zc_armed;
    volatile bool     bemf_rising;          // expected comparator edge polarity
} motor_state_t;

extern esc_config_t  cfg;
extern motor_state_t m;
```

---

## 1. Six-Step Commutation with Damped Light

The key idea behind damped light (complementary PWM) is that during the PWM off-time, the low-side FET of the driven phase switches on instead of letting current freewheel through its body diode. This has three effects:

- **Synchronous rectification:** lower losses and cooler FETs.
- **Linear throttle response:** current decays actively.
- **Implicit active braking:** when the commanded duty is below the back-EMF-equivalent duty, the motor regenerates into the bus. This is the "light" brake felt on deceleration.

Each step drives one phase with PWM (high side, complementary if damped), holds one phase low-side ON, and leaves one phase floating for BEMF sensing.

```c
// commutation.c
#include "esc_core.h"
#include "hal.h"

// Per step: {pwm phase, low phase, floating phase, BEMF edge on floating phase}
typedef struct { phase_t hi, lo, fl; bool rising; } step_map_t;

static const step_map_t STEP_FWD[7] = {
    {0},
    {PH_A, PH_B, PH_C, false},   // 1: A+ B-  C falling
    {PH_C, PH_B, PH_A, true },   // 2: C+ B-  A rising
    {PH_C, PH_A, PH_B, false},   // 3: C+ A-  B falling
    {PH_B, PH_A, PH_C, true },   // 4: B+ A-  C rising
    {PH_B, PH_C, PH_A, false},   // 5: B+ C-  A falling
    {PH_A, PH_C, PH_B, true },   // 6: A+ C-  B rising