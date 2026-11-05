# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

**Note:** This is a reference implementation that follows AM32's architecture (comparator-based BEMF zero-cross detection, timer-capture DShot decoding, and complementary-PWM "damped light" mode). It is not a verbatim copy of the AM32 repository. It targets an STM32G071 or AT32F421-class MCU running a 64 MHz timer clock. Pin and peripheral names are abstracted behind small HAL macros so you can map them to a specific target.

I could not identify an established "Cycle 28" convention. I have treated it as a revision label and tagged the files accordingly.

---

## 1. Configuration (`config.h`)

```c
#ifndef CONFIG_H
#define CONFIG_H
#include <stdint.h>
#include <stdbool.h>

#define FW_CYCLE                28
#define CPU_FREQ_HZ             64000000UL

/* ---- PWM ---- */
#define PWM_FREQ_HZ             24000
#define PWM_ARR                 ((CPU_FREQ_HZ / PWM_FREQ_HZ) - 1)   /* 2665 */
#define DEAD_TIME_TICKS         40          /* ~625 ns @ 64 MHz, tune per FET */
#define MIN_DUTY                (PWM_ARR / 40)
#define MAX_DUTY                PWM_ARR

/* ---- Commutation timer: 1 tick = 0.5 us (prescaler 31) ---- */
#define COM_TIMER_PSC           31
#define TICKS_PER_US            2

/* ---- DShot ---- */
#define DSHOT_BITS              16
#define DSHOT_EDGES             (DSHOT_BITS * 2)
#define DSHOT_MIN_FRAME_TICKS   (12 * 64)   /* DShot1200 frame ~13.3 us   */
#define DSHOT_MAX_FRAME_TICKS   (120 * 64)  /* allows DShot150..1200      */

/* ---- User settings (EEPROM-backed in a real build) ---- */
typedef struct {
    uint8_t  timing_advance;     /* 0..3 -> 0, 7.5, 15, 22.5 deg     */
    bool     comp_pwm;           /* damped light / sync rectification */
    bool     brake_on_stop;
    uint8_t  brake_strength;     /* 0..10                             */
    uint8_t  drag_brake_strength;/* 0..10, applied on throttle drop   */
    bool     bidirectional;      /* 3D mode                           */
    bool     bidir_dshot;        /* inverted DShot + eRPM telemetry   */
    uint8_t  motor_poles;
} esc_settings_t;

extern esc_settings_t settings;
#endif
```

---

## 2. High-Resolution DShot1200 Decoding (`dshot.c`)

### Signal timing at 64 MHz

| Protocol  | Bit period | T1H (1) | T0H (0) | Bit ticks | T1H ticks | T0H ticks |
|-----------|-----------|---------|---------|-----------|-----------|-----------|
| DShot1200 | 0.833 µs  | 0.625 µs| 0.313 µs| 53        | 40        | 20        |
| DShot600  | 1.667 µs  | 1.25 µs | 0.625 µs| 107       | 80        | 40        |
| DShot300  | 3.333 µs  | 2.5 µs  | 1.25 µs | 213       | 160       | 80        |

### Design choices

- **Capture method.** The input timer runs **unprescaled** (15.6 ns resolution) in both-edge capture mode. DMA writes 32 timestamps per frame into a buffer.
- **Bit decision.** Each bit is decided by **comparing its high time against its low time** (`high > low → 1`), not against a fixed threshold. This makes decoding protocol-rate agnostic and tolerant of clock drift. It works because a "1" has a 75% duty cycle and a "0" has 37.5%.
- **Frame validation.** A frame is accepted only if its total length is plausible, the CRC matches, and the frame arrives inside the expected window.

```c
#include "config.h"
#include "dshot.h"

static volatile uint32_t dma_buf[DSHOT_EDGES];
volatile uint16_t dshot_throttle   = 0;    /* 0..2047 raw            */
volatile bool     dshot_armed      = false;
volatile uint32_t dshot_good_frames, dshot_bad_frames;
volatile uint8_t  dshot_cmd_pending = 0;
volatile bool     telemetry_requested = false;

/* Called from DMA transfer-complete ISR (or after IC timeout) */
void dshot_decode_frame(void)
{
    uint32_t t0 = dma_buf[0];
    uint32_t frame_len = dma_buf[DSHOT_EDGES - 1] - t0;

    if (frame_len < DSHOT_MIN_FRAME_TICKS || frame_len > DSHOT_MAX_FRAME_TICKS) {
        dshot_bad_frames++;
        dshot_rearm_capture();
        return;
    }

    uint16_t value = 0;
    for (int i = 0; i < DSHOT_BITS; i++) {
        uint32_t rise = dma_buf[2 * i];
        uint32_t fall = dma_buf[2 * i + 1];
        /* Next rising edge; last bit uses the average bit period */
        uint32_t next = (i < DSHOT_BITS - 1) ? dma_buf[2 * i + 2]
                                             : rise + frame_len / (DSHOT_BITS - 1);
        uint32_t high = fall - rise;
        uint32_t low  = next - fall;
        value = (value << 1) | (high > low ? 1u : 0u);
    }

    /* For inverted (bidirectional) DShot the edges are swapped by
       capturing falling-first; logic above stays identical. */

    uint16_t payload = value >> 4;
    uint8_t  crc_rx  = value & 0x0F;
    uint8_t  crc     = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0F;
    if (settings.bidir_dshot) crc = (~crc) & 0x0F;

    if (crc != crc_rx) {
        dshot_bad_frames++;
        dshot_rearm_capture();
        return;
    }
    dshot_good_frames++;

    uint16_t throttle = payload >> 1;
    telemetry_requested = payload & 1;

    if (throttle == 0) {
        dshot_throttle = 0;
        dshot_armed = true;           /* arm after N zero frames in main */
    } else if (throttle < 48) {
        dshot_cmd_pending = (uint8_t)throttle;   /* special commands */
        dshot_throttle = 0;
    } else {
        dshot_throttle = throttle;    /* 48..2047 */
    }

    if (settings.bidir_dshot)
        dshot_schedule_erpm_reply();  /* switch pin to output ~30 us later */

    dshot_rearm_capture();
}
```

### Throttle mapping, including 3D mode

```c
/* Returns signed demand -2000..+2000 (0 = stop) */
int16_t dshot_to_demand(uint16_t t)
{
    if (t == 0) return 0;
    if (!settings.bidirectional)
        return (int16_t)(t - 47);                    /* 1..2000 */
    if (t <= 1047) return -(int16_t)((t - 47) * 2);  /* reverse */
    return (int16_t)((t - 1047) * 2);                /* forward */
}
```

### Bidirectional eRPM reply (GCR encoded)

```c
static const uint8_t gcr_tbl[16] = {
 0x19,0x1B,0x12,0x13,0x1D,0x15,0x16,0x17,0x1A,0x09,0x0A,0x0B,0x1E,0x0D,0x0E,0x0F };

uint32_t build_erpm_frame(uint32_t period_us)
{
    /* period_us = electrical period; encode as 3-bit exponent + 9-bit mantissa */
    if (period_us > 65408) period_us = 65408;
    uint8_t shift = 0;
    while (period_us > 511) { period_us >>= 1; shift++; }
    uint16_t v = (shift << 9) | period_us;
    uint8_t crc = (~(v ^ (v >> 4) ^ (v >> 8))) & 0x0F;
    uint16_t pkt = (v << 4) | crc;

    uint32_t gcr = 0;
    for (int n = 3; n >= 0; n--) gcr = (gcr << 5) | gcr_tbl[(pkt >> (n * 4)) & 0xF];
    /* NRZ->NRZI: transition on each 1 */
    uint32_t out = 0, level = 0;
    for (int b = 19; b >= 0; b--) {
        if (gcr & (1u << b)) level ^= 1;
        out = (out << 1) | level;
    }
    return out;  /* 21 bits incl. leading 0, clocked out at 5/4 of input bitrate */
}
```

---

## 3. Motor Timing: BEMF Zero-Cross and Commutation (`commutation.c`)

### Timing model

One commutation step is 60° electrical. The zero crossing occurs 30° into the step, so with no advance the next commutation should happen 30° after the zero crossing:

```
interval  = averaged time between zero crossings (= 60° electrical)
advance   = timing_advance × 7.5°
wait_time = interval/2 − interval × advance/60°
          = (interval >> 1) − (interval × timing_advance) / 8
```

### Implementation

```c
#include "config.h"
#include "phase.h"

volatile uint8_t  step = 1;              /* 1..6 */
volatile bool     forward = true;
volatile uint32_t commutation_interval = 0xFFFF;
volatile uint32_t zc_timestamp, last_zc_timestamp;
volatile uint16_t commutation_intervals[6];
volatile bool     running = false, zc_found = false;
volatile uint8_t  bemf_counter = 0;
volatile uint8_t  filter_level = 5;

/* Comparator input selection and expected edge per step */
static const struct { uint8_t comp_phase; bool rising; } zc_map[7] = {
  {0,0}, {PH_C,false},{PH_B,true},{PH_A,false},{PH_C,true},{PH_B,false},{PH_A,true}
};

static inline uint32_t compute_wait(uint32_t interval)
{
    uint32_t adv = (interval * settings.timing_advance) >> 3;  /* 7.5 deg steps */
    uint32_t half = interval >> 1;
    return (half > adv) ? half - adv : 1;
}

/* --- Comparator EXTI 