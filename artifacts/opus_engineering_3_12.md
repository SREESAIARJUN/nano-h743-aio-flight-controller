# AM32 ESC Firmware: Commutation Timing, DShot1200 Decoding, and Active Braking

This is a consolidated implementation of three AM32 subsystems:

- Sensorless 6-step commutation timing with advance and demag handling
- DShot1200 input-capture decoding (normal and bidirectional with eRPM telemetry)
- Complementary-PWM "damped light" active braking with drag-brake and brake-on-stop

The code targets an STM32G071 or STM32F051-class MCU. The structure follows AM32's architecture (comparator zero-cross, interval timer, TIM1 6-channel PWM, a single input-capture timer with DMA). It is written to be readable and adaptable, so check pin and peripheral mapping against your target's `targets.h` before flashing.

---

## 1. System Architecture

```
            ┌──────────────┐  DMA (32 edges)  ┌─────────────────┐
 Signal ───►│ TIM3 CH1 IC  │─────────────────►│ dshot_decode()  │──► newinput (0..2047)
            └──────────────┘                  └─────────────────┘        │
                                                                          ▼
 ┌────────────┐ ZC irq ┌──────────────┐ TIM14 (one-shot) ┌───────────┐  duty / brake
 │ COMP1/2    │───────►│ zc_handler() │─────────────────►│ commutate │◄──────────────
 │ (BEMF mux) │        │ interval calc│  advance delay   │ 6-step tbl│
 └────────────┘        └──────────────┘                  └───────────┘
                               ▲                                │
                       TIM2 free-running (1 MHz) ─ interval     ▼
                                                     TIM1 CH1-3 + CH1N-3N (comp PWM)
```

### Timer Allocation

| Timer | Role | Clock |
|---|---|---|
| TIM1 | 3-phase complementary PWM, center-aligned, dead-time | 64 MHz, ARR ≈ 2666 (24 kHz) |
| TIM2 | Free-running commutation interval timer | 1 MHz tick (PSC = 63) |
| TIM14 | One-shot "time-to-commutate" after ZC | 1 MHz |
| TIM3 | DShot input capture, both edges, DMA | 64 MHz (PSC = 0) |
| TIM16/17 | Bidirectional telemetry output (GPIO bit-bang via DMA) | 64 MHz |

---

## 2. Motor Commutation Timing

### 2.1 Principle

In 6-step trapezoidal drive, one electrical cycle has 6 steps of 60° each. The back-EMF zero-cross (ZC) of the floating phase occurs 30° after the previous commutation. The ideal next commutation therefore falls 30° after ZC.

With timing advance *A*, the delay after ZC becomes:

```
t_delay = (30° − A) / 60° × commutation_interval
        = interval/2 − interval × A/60
```

AM32 expresses advance in 7.5° units (`advance_level` 0–3, extended to 0–4+ in newer builds), so:

```
advance  = (interval × advance_level) >> 3     // interval/8 per 7.5°
waitTime = (interval >> 1) − advance
```

### 2.2 Commutation Table

```c
// Phase drive states: HIGH = PWM, LOW = low-side on, FLOAT = off (BEMF sense)
typedef enum { PH_FLOAT, PH_PWM, PH_LOW } phase_state_t;

typedef struct {
    phase_state_t a, b, c;
    uint8_t  comp_input;   // which phase the comparator watches
    uint8_t  rising;       // expected BEMF edge polarity
} comm_step_t;

static const comm_step_t comm_table[6] = {
    /* step 1 */ { PH_PWM,   PH_LOW,   PH_FLOAT, PHASE_C, 1 },
    /* step 2 */ { PH_PWM,   PH_FLOAT, PH_LOW,   PHASE_B, 0 },
    /* step 3 */ { PH_FLOAT, PH_PWM,   PH_LOW,   PHASE_A, 1 },
    /* step 4 */ { PH_LOW,   PH_PWM,   PH_FLOAT, PHASE_C, 0 },
    /* step 5 */ { PH_LOW,   PH_FLOAT, PH_PWM,   PHASE_B, 1 },
    /* step 6 */ { PH_FLOAT, PH_LOW,   PH_PWM,   PHASE_A, 0 },
};
```

### 2.3 State Variables

```c
volatile uint8_t  step = 1;
volatile uint8_t  forward = 1;              // direction (reversible via 3D / turtle)
volatile uint32_t thiszctime, lastzctime;
volatile uint32_t commutation_interval = 12500;   // µs per 60°
volatile uint32_t e_com_time;               // full e-rev time (6 steps), for telemetry
volatile uint32_t commutation_intervals[6];
volatile uint16_t advance_level = 2;        // 2 × 7.5° = 15°
volatile uint16_t waitTime, advance;
volatile uint8_t  zcfound, bemfcounter, bad_count;
volatile uint8_t  filter_level = 5;         // consecutive confirmations needed
volatile uint8_t  running, old_routine = 1; // old_routine = polling mode at low rpm
volatile uint8_t  desync_check, demag_time_pct = 0;
```

### 2.4 Zero-Cross Interrupt and Timing Calculation

```c
void comparator_isr(void)            // COMP1/2 EXTI line
{
    // ---- Blanking: ignore ZC too soon after commutation (switching ringing / demag)
    if (TIM2->CNT < (commutation_interval >> 1)) {   // inside first half of step
        return;
    }

    // ---- Glitch filter: re-sample comparator output N times
    for (uint8_t i = 0; i < filter_level; i++) {
        if (getCompOutputLevel() == comm_table[step - 1].rising) {
            return;  // signal bounced back → not a real ZC
        }
    }

    thiszctime = TIM2->CNT;
    disableCompInterrupts();

    // ---- Interval: rolling average weighting new sample 1/2, clamped to reject outliers
    uint32_t raw = thiszctime;               // TIM2 is reset at each commutation
    if (raw > (commutation_interval << 1) && running) {
        bad_count++;                          // desync hint
        if (bad_count > 6) { desync_handler(); return; }
    } else {
        bad_count = 0;
    }
    commutation_interval = (commutation_interval + (lastzctime + raw)) >> 1;
    // (lastzctime = ZC position of previous step relative to its commutation)

    // ---- Advance and wait
    advance  = (commutation_interval >> 3) * advance_level;   // 7.5° units
    waitTime = (commutation_interval >> 1) - advance;
    if (waitTime < 2) waitTime = 2;

    // ---- Arm one-shot commutation timer
    TIM14->CNT = 0;
    TIM14->ARR = waitTime;
    TIM14->SR  = 0;
    TIM14->DIER |= TIM_DIER_UIE;
    TIM14->CR1  |= TIM_CR1_CEN;

    lastzctime = commutation_interval - thiszctime;  // carry remainder
    zcfound = 1;
}

void TIM14_IRQHandler(void)          // time to commutate
{
    TIM14->SR = 0;
    TIM14->CR1 &= ~TIM_CR1_CEN;
    TIM14->DIER &= ~TIM_DIER_UIE;
    commutate();
    TIM2->CNT = 0;                    // restart interval measurement
    enableCompInterrupts();           // re-armed after blanking check in ISR
}
```

### 2.5 Commutation

```c
void commutate(void)
{
    commutation_intervals[step - 1] = commutation_interval;
    e_com_time = commutation_intervals[0] + commutation_intervals[1] +
                 commutation_intervals[2] + commutation_intervals[3] +
                 commutation_intervals[4] + commutation_intervals[5];

    if (forward) { if (++step > 6) step = 1; }
    else         { if (--step < 1) step = 6; }

    apply_phase_states(&comm_table[step - 1]);
    changeCompInput(comm_table[step - 1].comp_input,
                    comm_table[step - 1].rising ^ !forward);
    bemfcounter = 0;
    zcfound = 0;
}

static inline void set_phase(uint8_t ph, phase_state_t s)
{
    switch (s) {
    case PH_PWM:
        // comp_pwm: CHx + CHxN both enabled → synchronous rectification
        if (eepromBuffer.comp_pwm) tim1_set_ch_mode(ph, PWM_COMPLEMENTARY);
        else                        tim1_set_ch_mode(ph, PWM_HIGH_ONLY);
        break;
    case PH_LOW:   tim1_set_ch_mode(ph, FORCE_LOW_ON);  break;
    case PH_FLOAT: tim1_set_ch_mode(ph, ALL_OFF);       break;
    }
}

void apply_phase_states(const comm_step_t *s)
{
    // Float first to avoid shoot-through between table states
    if (s->a == PH_FLOAT) set_phase(PHASE_A, PH_FLOAT);
    if (s->b == PH_FLOAT) set_phase(PHASE_B, PH_FLOAT);
    if (s->c == PH_FLOAT) set_phase(PHASE_C, PH_FLOAT);
    set_phase(PHASE_A, s->a);
    set_phase(PHASE_B, s->b);
    set_phase(PHASE_C, s->c);
    TIM1->EGR = TIM_EGR_COMG;   // preload commit (CCPC=1) → atomic update
}
```

Using `CCPC` with `COMG` makes all six output-enable bits switch on the same clock edge. This avoids transient states where two phases are both driven high.

### 2.6 Low-RPM Polling Mode and Demag Compensation

Below roughly 2,000 eRPM, comparator interrupts are unreliable because BEMF is small and PWM noise dominates. AM32 switches to a polled routine sampled in the PWM ISR (`old_routine`):

```c
void poll_zero_cross(void)           // called from TIM1 update ISR when old_routine
{
    if (TIM2->CNT < (commutation_interval >> 1)) return;   // blanking
    if (getCompOutputLevel() != comm_table[step-1].rising) {
        if (++bemfcounter > min_bemf_counts) {
            // Low speed: no advance, commutate immediately at 30° nominal estimate
            thiszctime = TIM2->CNT;
            commutation_interval = (commutation_interval + thiszctime*2) >> 1;
            waitTime = commutation_interval >> 1;
            arm_tim14(waitTime);
        }
    } else {
        bemfcounter = 0;
    }
}

void update_routine_mode(void)
{
    if (commutation_interval > 12000)      old_routine = 1;  // < ~1400 eRPM per step
    else if (commutation_interval < 9000)  old_routine = 0;  // hysteresis
    min_bemf_counts = (commutation_interval > 6000) ? 6 : 3;
    filter_level    = (commutation_interval < 300)  ? 2 :
                      (commutation_interval < 1000) ? 4 : 8;
}
```

Demag compensation addresses a specific failure. At high load, the freewheel current on the floating phase takes time to decay. During that time, the comparator sees a clamped diode voltage that can look like a premature ZC.

The fix has two parts:

1. Extend blanking proportionally to duty: `blank = interval/2 + interval*duty*demag_pct/(100*maxduty)`.
2. If ZC repeatedly lands right after blanking, treat it as a demag event and cut duty for a few commutations.

```c
uint16_t blanking_time(void)
{
    uint32_t base = commutation_interval >> 1;
    uint32_t extra = (commutation_interval * duty_cycle * demag_time_pct)
                     / (100UL * tim1_arr);
    return base + extra;
}
```

### 2.7 eRPM Computation

```c
// e_com_time: µs per electrical revolution (6 steps)
uint16_t get_erpm_div100(void)
{
    if (!running || e_com_time == 0) return 0;
    return (uint16_t)(600000UL / e_com_time);    // 60e6/e_com_time/100
}
```

---

## 3. High-Resolution DShot1200 Decoding

### 3.1 Protocol Timing

| | DShot300 | DShot600 | DShot1200 |
|---|---|---|---|
| Bit period | 3.33 µs | 1.67 µs | **0.833 µs** |
| T1H | 2.50 µs | 1.25 µs | **0.625 µs** |
| T0H | 1.25 µs | 0.625 µs | **0.3125 µs** |
| Frame (16 bit) | 53 µs | 26.7 µs | **13.3 µs** |

At 64 MHz (15.6 ns/tick), DShot1200 gives:

- T0H ≈ 20 ticks
- T1H ≈ 40 ticks
- bit ≈ 53 ticks

These counts leave enough resolution for robust classification.

**Frame layout (MSB first):** `[11-bit throttle][1-bit telemetry req][4-bit CRC]`

- Normal CRC: `(v ^ v>>4 ^ v>>8) & 0xF`
- Bidirectional CRC: inverted (`~crc`), and the line idles **low** (inverted signal).

### 3.2 Capture Strategy

TIM3 CH1 captures **both edges** (`CC1P | CC1NP`) into a 32-entry DMA buffer. That gives 16 rising and 16 falling timestamps.

Pulse width for bit *i* is `buf[2i+1] − buf[2i]`. Classification uses a self-calibrating threshold: half the measured bit period, derived from rising-to-rising distance. This makes decoding speed-agnostic. DShot 150/300/600/1200 are auto-detected with the same code.

```c
#define DSHOT_EDGES 32
volatile uint32_t dma_buffer[DSHOT_EDGES];
volatile uint8_t  dshot_frame_ready, inverted_signal, dshot_telemetry;
volatile uint16_t newinput;
volatile uint16_t dshot_badcounts, dshot_goodcounts;
volatile uint8_t  dshotcommand, send_telemetry;

void dshot_ic_init(void)
{
    TIM3->PSC   = 0;                 // 64 MHz full resolution
    TIM3->ARR   = 0xFFFF;
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0   // IC1 → TI1
                | (0x0 << TIM_CCMR1_IC1F_Pos);  // no digital filter at 1200 (would eat 20-t