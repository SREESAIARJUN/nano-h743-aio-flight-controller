# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is an **original reference implementation** written in the architecture and style of AM32. It is not copied from the AM32 source. The canonical GPL-3.0 code is at `github.com/am32-firmware/AM32`. Treat this as a design you can study, port, or compare against.

- **Target:** STM32G071 at 64 MHz, using the CMSIS register API.
- **Hardware abstraction:** pin, comparator-mux and DMAMUX IDs are isolated in `target.h`. Verify them against your board's schematic and RM0444 before flashing.

---

## 1. System Architecture

| Function | Peripheral | Clock / Resolution | Notes |
|---|---|---|---|
| Phase PWM (complementary + dead time) | TIM1 CH1–3 / CH1N–3N | 64 MHz, 24 kHz PWM (ARR = 2666) | Center-aligned |
| DShot input capture | TIM3 CH1 + DMA1 Ch1 | 64 MHz (15.6 ns) | Both edges, 32 timestamps per frame |
| Commutation interval measurement | TIM2 (32-bit free-running) | 2 MHz (0.5 µs) | Zero-cross timestamps |
| Commutation scheduling | TIM14 one-shot | 2 MHz | Fires the 30° minus advance delay |
| BEMF zero-cross detection | COMP1 + EXTI | — | Input mux switched per step |
| Control loop | TIM6 | 20 kHz | Ramp, braking, startup, watchdog |

**Interrupt priority** (highest first):

1. Commutation (TIM14)
2. Comparator (EXTI)
3. DShot DMA transfer-complete
4. Control loop (TIM6)

Commutation is the most timing-critical event, so it must never be delayed by frame decoding.

---

## 2. Configuration (`config.h`)

```c
#pragma once
#include <stdint.h>

#define CPU_FREQ_HZ          64000000UL
#define PWM_FREQ_HZ          24000UL
#define PWM_ARR              ((CPU_FREQ_HZ / PWM_FREQ_HZ) / 2)   /* center-aligned: 1333 */
#define TIMING_TICK_HZ       2000000UL                           /* TIM2/TIM14 tick = 0.5 us */

/* Dead time: DTG[7:5]=0b0xx -> DT = DTG * tDTS (15.625 ns @ 64 MHz)
 * 0x28 = 40 * 15.6 ns = 625 ns. Size it to your MOSFET + driver turn-off time. */
#define DEAD_TIME_DTG        0x28

typedef struct {
    uint8_t  timing_advance;      /* 0..4 -> 0, 7.5, 15, 22.5, 30 deg electrical */
    uint8_t  comp_pwm;            /* 1 = complementary PWM (damped light / sync rect.) */
    uint8_t  brake_on_stop;       /* 1 = engage active brake at zero throttle */
    uint8_t  brake_strength;      /* 0..10, scales low-side brake duty */
    uint8_t  drag_brake_strength; /* 0..10, limits regen when decelerating */
    uint8_t  dir_reversed;
    uint8_t  bidirectional_dshot;
    uint8_t  motor_kv_div;        /* unused here, for telemetry scaling */
    uint16_t min_duty;            /* minimum running duty (PWM counts) */
    uint16_t startup_duty;
    uint8_t  demag_compensation;  /* 0 off, 1 low, 2 high */
} esc_config_t;

extern esc_config_t cfg;
```

---

## 3. DShot1200 Decoder (`dshot.c`)

### 3.1 Signal timing at 64 MHz

| Parameter | Time | Timer ticks |
|---|---|---|
| Bit period | 0.833 µs | 53.3 |
| T0H ("0" high time, 37.5 %) | 0.3125 µs | 20 |
| T1H ("1" high time, 75 %) | 0.625 µs | 40 |
| Frame (16 bits) | 13.3 µs | ~853 |

### 3.2 Decoding approach

- **Capture both edges with DMA.** 32 timestamps cover 16 bits.
- **Adaptive per-bit threshold.** Each bit's high time is compared with half of its own measured period, computed from the rising edge to the next rising edge. This makes the decoder independent of DShot speed. The same code accepts DShot150 through DShot1200 and tolerates FC clock error.
- **Last bit.** It has no following rising edge, so it uses the frame's average bit period.
- **Bidirectional mode.** The line idles high and the signal is inverted. The capture polarity is swapped, and the CRC is the bitwise inverse.

### 3.3 Code

```c
#include "stm32g0xx.h"
#include "config.h"
#include "target.h"     /* DSHOT_DMAMUX_REQ (TIM3_CH1 = 32 on G0; verify), pin AF */

#define DSHOT_EDGES   32

static volatile uint16_t cap_buf[DSHOT_EDGES];

volatile uint16_t dshot_throttle;      /* 0 = stop, 1..2000 */
volatile uint8_t  dshot_new_frame;
volatile uint8_t  dshot_telemetry_req;
volatile uint32_t dshot_good_frames, dshot_bad_frames;
volatile uint8_t  signal_timeout_ms;   /* reset on every good frame; failsafe at 250 */

static uint8_t  last_cmd, cmd_repeat;

void esc_handle_command(uint8_