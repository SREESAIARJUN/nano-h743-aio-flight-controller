# AM32-Style ESC Firmware: Commutation Timing, DShot1200 Decoding, and Active Braking

This is a reference implementation of the three subsystems, written in the structure and conventions of AM32. It is **not a verbatim copy of the AM32 repository**. Register-level details target an STM32F051/G071-class MCU (48–64 MHz, TIM1 for 3-phase PWM, a comparator for back-EMF). Port the HAL macros to your target before use.

---

## 1. System Architecture

```
            ┌─────────────── DShot input (TIMx CH + DMA, edge capture)
            ▼
   [DShot decoder] ──► throttle (0..2047), commands, telemetry request
            │
            ▼
   [Throttle / ramp / brake logic] ──► duty_cycle, braking mode
            │
            ▼
   [TIM1 complementary PWM, 3 phases, dead-time]  ◄── step table (6-step)
            ▲
            │ commutate()
   [Comparator ZC ISR] ──► [Commutation timer (TIM14)] ── advance-delayed commutation
```

**Interrupt priorities, from highest to lowest:**

| Priority | Interrupt | Reason |
|---|---|---|
| 0 | Comparator ZC / commutation timer | Timing-critical |
| 1 | DShot DMA transfer-complete | Decode deferred to main loop if needed |
| 2 | 20 kHz control loop (TIM6) | Ramping, braking, timeouts |

---

## 2. Motor Timing (Sensorless 6-Step Commutation)

### 2.1 Theory

- One electrical revolution has 6 steps of 60°.
- The back-EMF zero-crossing (ZC) on the floating phase occurs **30° before** the ideal commutation point.
- Commutation is therefore scheduled `(30° − advance)` after the ZC.
- 60° equals one `commutation_interval`, so the delay is `interval/2 − interval·advance/60`.
- AM32 expresses advance in steps of 7.5° (0–3 steps maps to 0°, 7.5°, 15°, 22.5°, and so on), so the delay is `interval/2 − (interval/8)·advance_level`.

### 2.2 Step Table

```c
// Phase states: HIGH = PWM on high side (complementary), LOW = low side on, FLOAT = off
typedef enum { PH_FLOAT, PH_PWM, PH_LOW } phase_state_t;

typedef struct {
    phase_state_t a, b, c;
    uint8_t  cmp_input;     // which phase the comparator watches
    uint8_t  rising;        // expected ZC polarity on floating phase
} comm_step_t;

static const comm_step_t step_table[6] = {
    // A       B        C        sense  rising
    { PH_PWM,  PH_LOW,  PH_FLOAT, PHASE_C, 0 },  // step 1: A->B, C floats
    { PH_PWM,  PH_FLOAT,PH_LOW,   PHASE_B, 1 },  // step 2: A->C
    { PH_FLOAT,PH_PWM,  PH_LOW,   PHASE_A, 0 },  // step 3: B->C
    { PH_LOW,  PH_PWM,  PH_FLOAT, PHASE_C, 1 },  // step 4: B->A
    { PH_LOW,  PH_FLOAT,PH_PWM,   PHASE_B, 0 },  // step 5: C->A
    { PH_FLOAT,PH_LOW,  PH_PWM,   PHASE_A, 1 },  // step 6: C->B
};
```

### 2.3 Timing State

```c
#define TIMER_TICK_US        0.5f      // commutation timer: 2 MHz
#define MIN_INTERVAL_TICKS   60        // ~ max eRPM guard
#define BLANK_FRACTION_SHIFT 2         // blank 1/4 of interval after commutation

volatile uint8_t  step = 0;
volatile uint8_t  forward = 1;
volatile uint16_t commutation_interval = 20000;   // ticks per 60°
volatile uint16_t last_zc_time;
volatile uint16_t thiszctime;
volatile uint16_t waitTime;
volatile uint8_t  advance_level = 2;               // 2 * 7.5° = 15°
volatile uint8_t  zc_found_count = 0;
volatile uint8_t  running = 0;
volatile uint8_t  bemf_timeout_count = 0;
uint16_t e_com_time;                                // full electrical period
```

### 2.4 Zero-Cross ISR

```c
void COMP_IRQHandler(void)
{
    COMP_CLEAR_FLAG();

    uint16_t now = COMM_TIMER->CNT;

    // Blanking: ignore demag/switching ringing right after commutation
    if (now < (commutation_interval >> BLANK_FRACTION_SHIFT))
        return;

    // Digital filter: re-sample the comparator N times (N shrinks with speed)
    uint8_t filter = (commutation_interval > 2000) ? 8 :
                     (commutation_interval > 500)  ? 4 : 2;
    for (uint8_t i = 0; i < filter; i++) {
        if (COMP_OUTPUT() != step_table[step].rising ^ !forward)
            return;                       // false ZC; wait for next edge
    }

    COMP_DISABLE_IRQ();                   // one ZC per step
    thiszctime = now;

    // IIR average: interval = (3*old + new)/4 -> smooth but responsive
    commutation_interval = (3 * commutation_interval + thiszctime) >> 2;
    if (commutation_interval < MIN_INTERVAL_TICKS)
        commutation_interval = MIN_INTERVAL_TICKS;

    // Delay to commutation: 30° minus advance (7.5° units)
    uint16_t advance = (commutation_interval >> 3) * advance_level;
    waitTime = (commutation_interval >> 1) - advance;

    // Schedule commutation via one-shot compare timer
    INTERVAL_TIMER->CNT  = 0;
    INTERVAL_TIMER->ARR  = waitTime;
    INTERVAL_TIMER->SR   = 0;
    INTERVAL_TIMER->DIER |= TIM_DIER_UIE;

    if (zc_found_count < 20) zc_found_count++;
    bemf_timeout_count = 0;
}
```

### 2.5 Commutation ISR

```c
void INTERVAL_TIMER_IRQHandler(void)
{
    INTERVAL_TIMER->SR = 0;
    INTERVAL_TIMER->DIER &= ~TIM_DIER_UIE;
    commutate();
}

void commutate(void)
{
    step = forward ? (step + 1) % 6 : (step + 5) % 6;
    apply_step(step);                 // sets TIM1 CCER/OCxM, see §4
    comp_select_input(step_table[step].cmp_input,
                      step_table[step].rising ^ !forward);

    e_com_time = commutation_interval * 6;
    COMM_TIMER->CNT = 0;              // timestamp origin for next ZC
    COMP_CLEAR_FLAG();
    COMP_ENABLE_IRQ();
}
```

### 2.6 Startup and Desync Handling (Control Loop, 20 kHz)

```c
void control_loop_20khz(void)
{
    // BEMF timeout: no ZC for > 2 intervals -> force commutation (open loop)
    if (running && COMM_TIMER->CNT > (commutation_interval << 1)) {
        commutate();
        if (++bemf_timeout_count > 100) {   // desync: stop and restart
            running = 0; zc_found_count = 0;
            all_phases_float();
        }
    }

    // Adaptive timing: more advance at high eRPM / high load (optional)
    if (auto_advance)
        advance_level = (commutation_interval < 300) ? 3 :
                        (commutation_interval < 1500) ? 2 : 1;

    update_throttle_and_braking();   // §4
}
```

**Open-loop startup:**

1. Align the rotor at low duty.
2. Run forced commutations with a decreasing interval.
3. Hand over to closed loop once `zc_found_count >= 20`.

---

## 3. High-Resolution DShot1200 Decoding

### 3.1 Signal Parameters

| Rate | Bit time | T1H (bit "1") | T0H (bit "0") | Frame (16 bit) |
|---|---|---|---|---|
| DShot300 | 3.33 µs | 2.50 µs | 1.25 µs | 53 µs |
| DShot600 | 1.67 µs | 1.25 µs | 0.625 µs | 27 µs |
| **DShot1200** | **0.833 µs** | **0.625 µs** | **0.3125 µs** | **13.3 µs** |

**Frame layout:** `[11-bit throttle][1-bit telemetry req][4-bit CRC]`, sent MSB first.

- **Normal CRC:** `(v ^ v>>4 ^ v>>8) & 0xF`
- **Bidirectional (inverted) CRC:** `~(...) & 0xF`

At 48 MHz with no prescaler, one DShot1200 bit is about 40 ticks:

| Pulse | Width (ticks) |
|---|---|
| "1" | ~30 |
| "0" | ~15 |

A threshold of about 22 ticks gives roughly ±7 ticks (±146 ns) of margin. That is adequate, but the input-capture filter must be short (≤ 4 samples) or it will eat into the margin.

### 3.2 Capture Strategy

- Configure the input timer in **input-capture, both edges**.
- Let DMA write timer CNT into a buffer for every edge: 32 edges per frame (one rise and one fall per bit).
- Use pulse **widths** (fall minus rise), not absolute times. This makes decoding immune to frame start jitter.
- Auto-detect the rate from the total frame length, so DShot300/600/1200 all work without configuration.

```c
#define DSHOT_EDGES 32
volatile uint16_t dma_buffer[DSHOT_EDGES];
volatile uint16_t newinput;
volatile uint8_t  dshot_frame_ready = 0;
volatile uint8_t  bidir_dshot = 0;
volatile uint16_t dshot_throttle = 0;
volatile uint8_t  telemetry_request = 0;
uint32_t dshot_good = 0, dshot_bad = 0;

void dshot_capture_start(void)
{
    IC_TIMER->CNT = 0;
    IC_TIMER->PSC = 0;                       // full resolution (48/64 MHz)
    IC_TIMER->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP; // both edges
    IC_TIMER->CCMR1 = TIM_CCMR1_CC1S_0 | (2 << TIM_CCMR1_IC1F_Pos); // light filter
    DMA_CH->CNDTR = DSHOT_EDGES;
    DMA_CH->CMAR  = (uint32_t)dma_buffer;
    DMA_CH->CCR  |= DMA_CCR_EN | DMA_CCR_TCIE;
    IC_TIMER->DIER |= TIM_DIER_CC1DE;
    IC_TIMER->CR1  |= TIM_CR1_CEN;
}

void DMA_IRQHandler(void)                     // 32 edges captured
{
    DMA_CLEAR_TC();
    DMA_CH->CCR &= ~DMA_CCR_EN;
    dshot_decode();
    dshot_capture_start();                    // re-arm (or switch to TX for bidir)
}
```

### 3.3 Decoder

```c
static inline uint8_t dshot_crc(uint16_t v, uint8_t inverted)
{
    uint8_t c = (v ^ (v >> 4) ^ (v >> 8)) & 0x0F;
    return inverted ? (~c & 0x0F) : c;
}

void dshot_decode(void)
{
    // Frame length sanity / rate auto-detect
    uint16_t frame_ticks = dma_buffer[31] - dma_buffer[0];
    // DShot1200 @48MHz: ~31*20 = ~620 ticks; DShot600 ~1240; DShot300 ~2480
    if (frame_ticks < 400 || frame_ticks > 4000) { dshot_bad++; return; }

    // Adaptive threshold: bit period = frame / 15.5 periods (~31 half-edges)
    // Threshold = 0.5625 * bit period ≈ midway between T0H (0.375) and T1H (0.75)
    uint16_t bit_period = (frame_ticks * 2) / 31;
    uint16_t threshold  = (bit_period * 9) >> 4;

    uint16_t value = 0;
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t high = dma_buffer[2*i + 1] - dma_buffer[2*i];   // uint16 wraps safely
        value <<= 1;
        if (high > threshold) value |= 1;
    }

    uint16_t payload = value >> 4;
    uint8_t  crc_rx  = value & 0x0F;

    if (crc_rx != dshot_crc(payload, bidir_dshot)) { dshot_bad++; return; }
    dshot_good++;

    telemetry_request = payload & 0x01;
    uint16_t cmd      = payload >> 1;          // 0..2047

    if (cmd == 0) {
        dshot_throttle = 0;                    // disarmed / stop
    } else if (cmd < 48) {
        if (dshot_throttle == 0) handle_dshot_command(cmd); // only when stopped
    } else {
        dshot_throttle = cmd - 47;             // 1..2000
    }
    signal_timeout = 0;                        // reset failsafe counter
}
```

### 3.4 High-Resolution Notes for 1200

**Timer resolution and capture filter**
- Run the timer at the full core clock with `PSC = 0`.
- Keep the capture filter at `IC1F ≤ 2`. Larger filters add delay of ≥ 8 clocks on each edge. The delay cancels for widths, but noise near the threshold doesn't.

**ISR and DMA latency**
- Keep the DMA ISR priority above the control loop.
- DShot frames can arrive every ~20 µs, and the decode takes about 2 µs at 48 MHz.

**Commands 1–47**
- Commands such as beacon, direction, 3D, and save settings require repetition (≥ 6 identical frames) before acting.
- They are only honored at zero throttle.

**Bidirectional DShot**
- The line idles low (inverted signal), so swap the capture polarity.
- After a valid frame, wait about 30 µs and reconfigure the pin as a timer output.
- Transmit the 21-bit GCR-encoded eRPM period at 5/4 of the bit rate, then return to input capture.

**Failsafe**
- Count `signal_timeout` in the 20 kHz loop.
- After about 250 ms without valid frames, set throttle to 0 and disarm.

---

## 4. PWM