# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is an original reference implementation that follows the architecture used by AM32. It is **not** the AM32 source, which is GPL-licensed and available at `github.com/am32-firmware/AM32`. Use this to understand the design, or as a base for your own firmware.

**Assumed hardware:** STM32G071 at 64 MHz, 3-phase half-bridge, comparator-based back-EMF (BEMF) sensing, and a single signal input on a timer capture channel. Register and HAL details are abstracted behind small `hw_*` functions so you can port it.

---

## 1. System Architecture

```
            ┌────────────────────────────────────────────────────────┐
 DShot ───► │ TIM3 CH1 input capture ─► DMA (32 edges) ─► EXTI/SW IRQ │
 signal     │        decode_dshot() → throttle / command             │
            └────────────────────────────────────────────────────────┘
                                   │
                                   ▼
 ┌──────────────┐   duty    ┌──────────────────┐  phase map ┌───────────┐
 │ control loop │ ────────► │ TIM1 3ch compl.  │ ─────────► │ gate drv  │
 │ (20 kHz tick)│           │ PWM + deadtime   │            │ + FETs    │
 └──────┬───────┘           └──────────────────┘            └─────┬─────┘
        │                                                         │ BEMF
        ▼                                                         ▼
 ┌──────────────┐  ZC event  ┌──────────────────┐         ┌──────────────┐
 │ commutation  │ ◄───────── │ COMP IRQ + filter│ ◄────── │ comparator   │
 │ timer TIM14  │            │ + blanking       │         │ mux (float φ)│
 └──────────────┘            └──────────────────┘         └──────────────┘
```

**Interrupt priorities, highest first:**
1. Comparator (zero-cross)
2. Commutation timer
3. DShot DMA complete
4. 20 kHz control tick
5. Main loop: telemetry and EEPROM

---

## 2. Global Configuration

```c
// config.h
#include <stdint.h>
#include <stdbool.h>

#define CPU_HZ                64000000UL
#define PWM_FREQ_HZ           24000UL
#define PWM_TOP               (CPU_HZ / PWM_FREQ_HZ)      // 2666 counts
#define DEAD_TIME_NS          300                          // FET-dependent!

#define COMMUTATION_TIMER_HZ  2000000UL                    // TIM14 @ 0.5 µs/tick
#define MAX_COMM_INTERVAL     40000                        // 20 ms → treat as stalled
#define MIN_COMM_INTERVAL     60                           // ~30 µs, ≈ 230k eRPM ceiling

#define DSHOT_TIMER_HZ        CPU_HZ                       // capture at full clock
#define DSHOT_FRAME_EDGES     32                           // 16 bits × 2 edges

typedef struct {
    uint8_t  timing_advance;       // 0..3  → 0°, 7.5°, 15°, 22.5°
    bool     comp_pwm;             // complementary PWM ("damped light")
    bool     brake_on_stop;
    uint8_t  brake_strength;       // 0..10
    uint8_t  drag_brake_strength;  // 0..10, applied on throttle decrease
    bool     bidirectional_dshot;
    bool     reversed;
    uint16_t startup_power;        // duty floor during startup
} esc_settings_t;

extern esc_settings_t cfg;
```

---

## 3. High-Resolution DShot1200 Decoding

### 3.1 Signal timing

| Protocol  | Bit period | "1" high time | "0" high time | Ticks/bit @64 MHz |
|-----------|-----------:|--------------:|--------------:|------------------:|
| DShot150  | 6.67 µs    | 5.00 µs       | 2.50 µs       | 427               |
| DShot300  | 3.33 µs    | 2.50 µs       | 1.25 µs       | 213               |
| DShot600  | 1.67 µs    | 1.25 µs       | 0.625 µs      | 107               |
| DShot1200 | 0.833 µs   | 0.625 µs      | 0.3125 µs     | **53**            |

At DShot1200, a "0" is about 20 ticks and a "1" about 40 ticks. A fixed threshold works, but **adaptive thresholding** is more robust against flight-controller clock drift and edge distortion.

The decoder therefore measures the whole frame and derives the bit period from it: `period = (last_rise − first_rise) / 15`. Each bit is then classified by `high_time > period/2`.

### 3.2 Capture setup

- The timer captures **both edges** on TI1 into a DMA buffer.
- The DMA is armed for 32 transfers; its transfer-complete interrupt triggers decoding.
- A frame-gap watchdog re-syncs if edges get misaligned (noise, hot-plug).
- In bidirectional mode the line idles high, so the input is inverted.

```c
// dshot.c
#include "config.h"

static volatile uint32_t dma_buf[DSHOT_FRAME_EDGES];
volatile uint16_t dshot_value      = 0;     // 0..2047 raw
volatile bool     dshot_new_frame  = false;
volatile bool     dshot_telem_req  = false;
volatile uint32_t dshot_good_frames = 0, dshot_bad_frames = 0;
volatile uint8_t  signal_timeout   = 0;     // incremented in 1 kHz tick

static bool     inverted;                   // bidirectional DShot
static uint8_t  detected_speed;             // 0=unknown, 1=150 ... 4=1200

void dshot_init(bool bidir)
{
    inverted = bidir;
    hw_capture_configure(/*both_edges=*/true, /*prescaler=*/0, /*invert=*/bidir);
    hw_capture_dma_start((uint32_t*)dma_buf, DSHOT_FRAME_EDGES);
}

/* Called from DMA transfer-complete ISR. Must be fast: ~3–5 µs budget. */
void dshot_dma_complete_isr(void)
{
    hw_capture_dma_stop();

    uint32_t t[DSHOT_FRAME_EDGES];
    for (int i = 0; i < DSHOT_FRAME_EDGES; i++) t[i] = dma_buf[i];

    //