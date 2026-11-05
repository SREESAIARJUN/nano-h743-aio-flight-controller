# AM32-Style ESC Firmware Reference: Commutation Timing, DShot1200 Decoding, and Active Braking

This is a **reference implementation modeled on AM32's architecture**, not a verbatim copy of the AM32 repository. It targets an STM32F051/G071-class MCU (Cortex-M0/M0+, 48–64 MHz) driving a 3-phase half-bridge with complementary PWM. Register-level details are shown for TIM1, TIM2/TIM3, and COMP1. Port pin and peripheral mappings must be adapted to your target's `targets.h`.

---

## 1. System Architecture

```
            ┌──────────────┐  DMA (edges)   ┌───────────────┐
 DShot ───► │ TIM3 IC CH1  │ ─────────────► │ dshot_buf[32] │
 input      └──────────────┘                └──────┬────────┘
                                                   │ DMA TC IRQ (prio 1)
                                                   ▼
                                         decode_dshot() → throttle / cmd
                                                   │
 ┌────────────┐   ZC IRQ (prio 0)   ┌──────────────▼───────────┐
 │ COMP1 BEMF │ ──────────────────► │ zero-cross → schedule    │
 └────────────┘                     │ commutation (TIM14 OC)   │
                                    └──────────────┬───────────┘
                                                   ▼
                                    ┌──────────────────────────┐
                                    │ TIM1 complementary PWM   │
                                    │ (damped light = sync rect)│
                                    └──────────────────────────┘
```

**Interrupt priorities (lower number = higher priority):**

| Priority | Source | Purpose |
|---|---|---|
| 0 | COMP / ZC, commutation timer | Motor timing |
| 1 | DShot DMA transfer complete | Frame decode |
| 2 | Telemetry / control loop tick (20 kHz) | Ramp, braking, eRPM |
| 3 | SysTick, UART | Housekeeping |

---

## 2. Global Configuration

```c
// config.h
#define CPU_FREQ_HZ          48000000UL
#define PWM_FREQ_HZ          24000UL
#define TIM1_ARR             ((CPU_FREQ_HZ / PWM_FREQ_HZ) - 1)   // 1999
#define DEAD_TIME_TICKS      40          // ~830 ns; tune to FET gate charge

#define COMMUTATION_TIMER_HZ 2000000UL   // 0.5 us/tick (TIM2, PSC = 23)

#define MIN_INTERVAL_US      50          // ~100k eRPM ceiling guard
#define MAX_INTERVAL_US      20000       // below this speed -> desync/startup

#define TIMING_ADVANCE_DEFAULT 2         // 0..3 => 0, 7.5, 15, 22.5 deg
#define DSHOT_THROTTLE_MIN   48
#define DSHOT_THROTTLE_MAX   2047

typedef struct {
    uint8_t  advance_level;      // 0-3 (x 7.5 deg)
    uint8_t  brake_on_stop;      // 1 = hold low sides when throttle = 0
    uint8_t  drag_brake_strength;// 1-10, 10 = full
    uint8_t  comp_pwm;           // 1 = complementary (damped light)
    uint8_t  bidirectional;      // 1 = bidir DShot (inverted + eRPM telem)
    uint8_t  reversed;
} esc_settings_t;

extern esc_settings_t settings;
```

---

## 3. High-Resolution DShot1200 Decoding

### 3.1 Signal Timing

| Protocol | Bit rate | Bit period | T1H (1) | T0H (0) | Frame (16 bit) |
|---|---|---|---|---|---|
| DShot300 | 300 kbit/s | 3.33 µs | 2.50 µs | 1.25 µs | 53.3 µs |
| DShot600 | 600 kbit/s | 1.67 µs | 1.25 µs | 0.625 µs | 26.7 µs |
| **DShot1200** | **1200 kbit/s** | **0.833 µs** | **0.625 µs** | **0.3125 µs** | **13.3 µs** |

At 48 MHz with prescaler 0, one DShot1200 bit is **40 ticks**: a "1" is high for about 30 ticks and a "0" is high for about 15 ticks. The decision threshold is about 22 ticks.

### 3.2 Capture Strategy

TIM3 CH1 runs in input capture on **both edges**, with DMA writing 32 timestamps (16 rising plus 16 falling) per frame. This removes any per-edge CPU load, which is required at 1.2 Mbit/s.

```c
// dshot.c
#include "config.h"

#define DSHOT_EDGES 32
static volatile uint16_t dshot_buf[DSHOT_EDGES];

volatile uint16_t dshot_throttle   = 0;
volatile uint8_t  dshot_command    = 0;
volatile uint8_t  dshot_telem_req  = 0;
volatile uint32_t dshot_good_frames = 0;
volatile uint32_t dshot_bad_frames  = 0;
volatile uint8_t  dshot_signal_ok  = 0;
static   uint16_t signal_timeout   = 0;

// Auto-detected per-protocol thresholds (in timer ticks)
static uint16_t bit_threshold = 22;   // DShot1200 @ 48 MHz default
static uint16_t frame_max     = 900;  // reject frames longer than this

void dshot_input_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;

    TIM3->PSC   = 0;              // full 48 MHz resolution
    TIM3->ARR   = 0xFFFF;
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0          // CC1 = TI1 input
                | (0x2 << TIM_CCMR1_IC1F_Pos); // light filter (fCK, N=4)
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP; // both edges
    TIM3->DIER  = TIM_DIER_CC1DE;

    DMA1_Channel4->CPAR  = (uint32_t)&TIM3->CCR1;
    DMA1_Channel4->CMAR  = (uint32_t)dshot_buf;
    DMA1_Channel4->CNDTR = DSHOT_EDGES;
    DMA1_Channel4->CCR   = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0
                         | DMA_CCR_TCIE | DMA_CCR_PL_1;
    NVIC_SetPriority(DMA1_Channel4_5_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

    DMA1_Channel4->CCR |= DMA_CCR_EN;
    TIM3->CR1 |= TIM_CR1_CEN;
}
```

### 3.3 Protocol Auto-Detection

On the first frames, the decoder measures the full frame span (first edge to last edge) to select the thresholds:

```c
static void dshot_autodetect(uint16_t frame_ticks)
{
    // frame_ticks ~= 15.5 bit periods
    uint16_t bit_period = frame_ticks / 15;   // approximate
    bit_threshold = (bit_period * 9) / 16;    // ~56% of bit: between 37.5% and 75%
    frame_max     = bit_period * 17;
}
```

### 3.4 Frame Decode and CRC

The frame layout is `[11-bit value][1-bit telemetry request][4-bit CRC]`.

- **Normal DShot:** `crc = (v ^ v>>4 ^ v>>8) & 0xF`
- **Bidirectional DShot:** the line is inverted (idle low → high) and the CRC is inverted (`~crc & 0xF`).

```c
static void decode_dshot(void)
{
    uint16_t frame_span = dshot_buf[31] - dshot_buf[0];
    if (frame_span > frame_max || frame_span < 200) {   // 200 ticks ~ DShot2400 floor
        dshot_bad_frames++;
        return;
    }

    uint16_t packet = 0;
    for (int i = 0; i < 16; i++) {
        // high-time = falling - rising (polarity handled by bidir flag)
        uint16_t high = dshot_buf[2*i + 1] - dshot_buf[2*i];
        packet = (packet << 1) | (high > bit_threshold ? 1 : 0);
    }

    uint16_t value  = packet >> 4;           // 12 bits: throttle(11) + telem(1)
    uint8_t  crc_rx = packet & 0x0F;
    uint8_t  crc    = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    if (settings.bidirectional) crc = (~crc) & 0x0F;

    if (crc != crc_rx) { dshot_bad_frames++; return; }

    dshot_good_frames++;
    signal_timeout  = 0;
    dshot_signal_ok = 1;

    uint16_t throttle = value >> 1;
    dshot_telem_req   = value & 1;

    if (throttle == 0) {
        dshot_throttle = 0;
    } else if (throttle < DSHOT_THROTTLE_MIN) {
        dshot_command = throttle;            // 1..47 = special commands
    } else {
        dshot_throttle = throttle - 47;      // 1..2000
    }

    if (settings.bidirectional) schedule_erpm_telemetry(); // reply ~30 us later
}

void DMA1_Channel4_5_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF4) {
        DMA1->IFCR = DMA_IFCR_CTCIF4;
        decode_dshot();
        // Re-arm: reset DMA so the next frame aligns to its first edge.
        DMA1_Channel4->CCR  &= ~DMA_CCR_EN;
        DMA1_Channel4->CNDTR = DSHOT_EDGES;
        DMA1_Channel4->CCR  |= DMA_CCR_EN;
    }
}

// Called at 1 kHz from housekeeping
void dshot_timeout_tick(void)
{
    if (++signal_timeout > 250) {   // 250 ms without a valid frame
        dshot_signal_ok = 0;
        dshot_throttle  = 0;        // failsafe
    }
}
```

**Frame alignment.** If DMA ever becomes desynchronized mid-frame, the span check fails. A recovery routine then resets the DMA during the inter-frame gap. Detect the gap with TIM3 update or a CC2 timeout: no edge for more than 2 bit periods means "idle", so re-arm.

### 3.5 Bidirectional eRPM Telemetry (GCR)

```c
static const uint8_t gcr_map[16] = {
    0x19,0x1B,0x12,0x13,0x1D,0x15,0x16,0x17,
    0x1A,0x09,0x0A,0x0B,0x1E,0x0D,0x0E,0x0F
};

uint32_t build_erpm_frame(uint32_t period_us)   // electrical period
{
    // 12-bit value = 3-bit exponent + 9-bit mantissa
    uint16_t e = 0;
    while (period_us > 0x1FF) { period_us >>= 1; e++; }
    uint16_t v = (e << 9) | period_us;
    uint8_t  crc = (~(v ^ (v >> 4) ^ (v >> 8))) & 0x0F;
    uint16_t packet = (v << 4) | crc;

    uint32_t gcr = 0;
    for (int i = 12; i >= 0; i -= 4)
        gcr = (gcr << 5) | gcr_map[(packet >> i) & 0xF];

    // Convert to NRZI transitions (21 bits) for output DMA
    uint32_t out = 0, last = 0;
    for (int i = 19; i >= 0; i--) {
        if (gcr & (1u << i)) last ^= 1;
        out = (out << 1) | last;
    }
    return out;   // sent at 5/4 of the DShot bitrate (1.5 Mbit/s for DShot1200)
}
```

### 3.6 Command Handling

| Command | Meaning | Requirement |
|---|---|---|
| 1–5 | Beeps | — |
| 7 / 8 | Spin direction normal / reversed | Sent 6 times |
| 9 / 10 | 3D mode off / on | Sent 6 times |
| 12 | Save settings | Sent 6 times |
| 20 / 21 | Spin direction normal / reversed (alt) | — |

```c
void process_dshot_command(void)
{
    static uint8_t last = 0, repeat = 0;
    if (!dshot_command) return;
    repeat = (dshot_command == last) ? repeat + 1 : 1;
    last = dshot_command;

    if (motor_running) { dshot_command = 0; return; }   // only when stopped

    switch (dshot_command) {
        case 1 ... 5: play_beacon(dshot_command); break;
        case 7:  if (repeat >= 6) settings.reversed = 0; break;
        case 8:  if (repeat >= 6) settings.reversed = 1; break;
        case 9:  if (repeat >= 6) settings.bidirectional = 0; break;
        case 10: if (repeat >= 6) settings.bidirectional = 1; break;
        case 12: if (repeat >= 6) save_settings_to_flash(); break;
        case 20: settings.reversed = 0; break;
        case 21: settings.reversed = 1; break;
    }
    dshot_command = 0;
}
```

---

## 4. Motor Timing: Commutation Engine

### 4.1 Six-Step Table

Each step drives one phase with PWM high, holds another low, and floats the third, which is sensed by the comparator.

```c
typedef struct {
    uint8_t pwm_phase;   // A=0,B=1,C=2
    uint8_t low_phase;
    uint8_t float_phase;
    uint8_t rising;      // expected BEMF slope on float phase
} comm_step_t;

static const comm_step_t comm_table[6] = {
    {0, 1, 2, 0},   // AB, C falling
    {0, 2, 1, 1},   // AC, B rising
    {1, 2, 0, 0},   // BC, A falling
    {1, 0, 2, 1},   // BA, C rising
    {2, 0, 1, 0},   // CA, B falling
    {2, 1, 0, 1},   // CB, A rising
};

volatile uint8_t  step = 0;
vol