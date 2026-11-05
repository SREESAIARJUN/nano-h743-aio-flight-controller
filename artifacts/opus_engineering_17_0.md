# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is a reference implementation modeled on the AM32 architecture: STM32F051/G071-class MCU, comparator-based back-EMF sensing, and timer-driven commutation. It is written to be internally consistent and buildable against a typical HAL/LL layer. It is **not** a verbatim copy of the AM32 source. Pin, timer, and comparator mappings are hardware-specific, so the low-level hooks are isolated in `hw_*` functions.

---

## 1. System Architecture

```
            ┌──────────────┐  DMA (edge timestamps)  ┌──────────────────┐
 Signal ───►│ TIM3 IC CH1  │────────────────────────►│ dshot_decode()   │──► throttle, cmds
            └──────────────┘                         └──────────────────┘
                                                              │
            ┌──────────────┐  ZC interrupt            ┌───────▼──────────┐
 BEMF ─────►│ COMP1 (mux)  │─────────────────────────►│ zero_cross_isr() │
            └──────────────┘                          └───────┬──────────┘
                                                              │ schedule
            ┌──────────────┐  timeout ISR             ┌───────▼──────────┐
            │ TIM14 (1MHz) │─────────────────────────►│ commutate()      │
            └──────────────┘                          └───────┬──────────┘
                                                              │
            ┌──────────────┐  complementary PWM      ┌────────▼─────────┐
            │ TIM1 CH1-3   │◄────────────────────────│ phase_* / brake  │
            └──────────────┘  + dead time            └──────────────────┘
```

**Interrupt priorities (highest to lowest):**
1. DShot DMA transfer complete
2. Commutation timer
3. Comparator zero-cross
4. Control loop (20 kHz tick)

---

## 2. Configuration (`config.h`)

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CPU_FREQ_HZ           48000000u
#define PWM_FREQ_HZ           24000u
#define PWM_ARR               (CPU_FREQ_HZ / PWM_FREQ_HZ)      // 2000 counts
#define DEAD_TIME_NS          300u                              // MOSFET dependent!

#define COMM_TIMER_HZ         1000000u                          // 1 us resolution (TIM14)

/* DShot1200 at 48 MHz input-capture clock
 * bit period = 0.833 us  -> 40 ticks
 * T1H = 0.625 us         -> 30 ticks
 * T0H = 0.3125 us        -> 15 ticks
 * decision threshold ~ midpoint = 22-23 ticks                */
#define DSHOT_EDGES           32u                               // 16 bits * (rise+fall)
#define DSHOT1200_BIT_TICKS   40u
#define DSHOT1200_THRESH      23u
#define DSHOT_FRAME_MIN       (DSHOT1200_BIT_TICKS * 15u)       // sanity window
#define DSHOT_FRAME_MAX       (DSHOT1200_BIT_TICKS * 17u)

#define TIMING_ADVANCE_STEPS  4u                                // 0, 7.5, 15, 22.5 deg
#define MIN_COMM_INTERVAL_US  40u                               // ~ 250k eRPM ceiling
#define MAX_COMM_INTERVAL_US  20000u                            // below = desync/startup

#define ZC_FILTER_MIN         2u
#define ZC_FILTER_MAX         12u

typedef struct {
    uint8_t  advance_level;        // 0..3  -> *7.5 deg electrical
    bool     comp_pwm;             // complementary PWM ("damped light")
    bool     brake_on_stop;
    uint8_t  brake_strength;       // 0..10 drag brake when throttle == 0
    bool     bidirectional;        // 3D mode
    bool     bidir_dshot;          // inverted DShot with eRPM telemetry
    bool     reversed;
} esc_config_t;

extern esc_config_t cfg;
```

---

## 3. High-Resolution DShot1200 Decoding (`dshot.c`)

### Strategy
- TIM3 CH1 runs in **input capture on both edges** at full 48 MHz (PSC = 0).
- DMA copies 32 captured timestamps into a buffer. The timestamps start from the first rising edge.
- In the DMA transfer-complete ISR:
  1. Validate the frame duration.
  2. Compute each pulse width as `fall - rise`, with wrap-safe 16-bit subtraction.
  3. Threshold each width into a bit.
  4. Verify the CRC.
  5. Dispatch the result as a throttle value or a command.
- An **adaptive threshold** is derived from the measured bit period. This tolerates FC clock skew of a few percent. That matters at 1200 kbit/s, where one tick is about 2.5% of a bit.

```c
#include "config.h"
#include "dshot.h"

static volatile uint16_t ic_buf[DSHOT_EDGES];

volatile uint16_t dshot_throttle   = 0;     // 0, or 48..2047
volatile bool     dshot_telem_req  = false;
volatile uint32_t dshot_good_frames = 0;
volatile uint32_t dshot_bad_frames  = 0;
volatile uint8_t  signal_timeout    = 0;

static uint8_t cmd_repeat = 0, last_cmd = 0;

/* CRC for normal DShot: XOR of the three nibbles.
 * Bidirectional (inverted) DShot uses the bitwise inverse. */
static inline uint8_t dshot_crc(uint16_t v12, bool inverted)
{
    uint8_t c = (v12 ^ (v12 >> 4) ^ (v12 >> 8)) & 0x0F;
    return inverted ? (~c & 0x0F) : c;
}

static void dshot_command(u