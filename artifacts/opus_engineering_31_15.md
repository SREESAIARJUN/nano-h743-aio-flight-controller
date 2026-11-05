# AM32-Style ESC Firmware Reference: Motor Timing, DShot1200 Decoding, and Active Braking

**Note on scope.** This is a reference implementation written in the style and architecture of AM32. It is not a verbatim copy of the AM32 repository. Register names assume an STM32F0/G0/G4-class MCU with:

- TIM1 for complementary 3-phase PWM
- a GPIO input-capture timer with DMA for DShot
- a comparator for back-EMF zero-cross detection

"Cycle 31" is treated as a revision label. Adapt pin and peripheral mappings to your target's `targets.h`.

---

## 1. System Architecture

```
               ┌────────────────────────────────────────────────┐
DShot pin ───► │ TIMx IC + DMA (32 edges) ──► dshot_decode()    │
               │      │  (EXTI/DMA-TC ISR, highest prio)        │
               │      ▼                                         │
               │  newinput (0..2047) ─► throttle_map()          │
               │      │                                         │
               │      ▼                                         │
Comparator ──► │ zc_isr() ──► interval filter ──► COM_TIMER     │
               │                    │              (advance)    │
               │                    ▼                           │
               │              commutate() ──► TIM1 CCER/OCxM    │
               │                                                │
               │ 20kHz control loop (tenKhzRoutine):            │
               │   duty ramp, brake logic, damped-light, RPM    │
               └────────────────────────────────────────────────┘
```

**Interrupt priorities (lower number = higher priority):**

| Priority | Source | Reason |
|---|---|---|
| 0 | DShot DMA transfer-complete | Must not miss frames at 1.2 Mbit |
| 1 | Commutation timer | Timing jitter becomes torque ripple |
| 1 | Comparator zero-cross | Same reason as commutation |
| 2 | 20 kHz control loop | Can tolerate small delays |
| 3 | Telemetry UART / USB | Least time-critical |

---

## 2. Global State and Configuration

```c
// --- config (EEPROM-backed) ---
typedef struct {
    uint8_t  advance_level;      // 0..42  -> advance in 0.9375° steps? (see below)
    uint8_t  brake_on_stop;      // 0=off, 1=on
    uint8_t  drag_brake_strength;// 1..10 (10 = weakest)
    uint8_t  comp_pwm;           // complementary PWM (damped light) enable
    uint8_t  bi_direction;
    uint8_t  dir_reversed;
    uint16_t pwm_freq_khz;       // 24..48
    uint8_t  motor_poles;
    uint8_t  variable_pwm;
} eeprom_cfg_t;

extern eeprom_cfg_t cfg;

// --- runtime ---
volatile uint16_t commutation_interval = 12500; // timer ticks per 60 elec. deg
volatile uint16_t zero_crosses;
volatile uint8_t  step = 1;           // 1..6
volatile uint8_t  running, armed;
volatile uint16_t newinput;           // 0..2047 raw dshot value
volatile uint16_t adjusted_input;     // 0..2000 throttle
volatile uint16_t duty_cycle, duty_cycle_setpoint;
volatile uint8_t  rising;             // expected BEMF edge polarity
volatile uint16_t advance;            // ticks
volatile uint16_t waitTime;
volatile uint8_t  old_routine = 1;    // 1 = polled startup, 0 = interrupt ZC
volatile uint32_t e_com_time;         // full electrical rev, for eRPM telemetry
volatile uint8_t  dshot_telemetry;    // bidirectional dshot active
volatile uint8_t  prop_brake_active;
```

**Timer base.** `COM_TIMER` (e.g. TIM14 or TIM17) and `INTERVAL_TIMER` (e.g. TIM2) run at 1 MHz / 2 MHz. This version uses **2 MHz (0.5 µs/tick)**: at 16 bits that covers about 32 ms per 60° step, which is enough for very low startup speeds.

---

## 3. Motor Timing: Zero-Cross Detection and Commutation

### 3.1 Theory

- One commutation step spans **60 electrical degrees**.
- In an ideal trapezoidal drive, the BEMF zero-crossing occurs **30° after the previous commutation**.
- The next commutation is therefore ideally scheduled **30° after the zero-cross**.
- Timing advance shortens this delay. This compensates for winding inductance, because phase current lags phase voltage:

```
wait = interval/2 − advance
advance = interval × (advance_deg / 60)
```

AM32 encodes advance in steps of 1/64 of 60°, which is 0.9375° per step. The useful range is 0 to about 26.25°: `advance_level` 0..28.

### 3.2 Phase Table (6-step)

| Step | High-side PWM | Low-side ON | Floating (BEMF sense) | ZC polarity |
|------|---------------|-------------|-----------------------|-------------|
| 1 | A | B | C | rising |
| 2 | C | B | A | falling |
| 3 | C | A | B | rising |
| 4 | B | A | C | falling |
| 5 | B | C | A | rising |
| 6 | A | C | B | falling |

### 3.3 Phase Drive Primitives (Complementary-Aware)

```c
// Each phase can be: PWM (comp or not), LOW (lowside fully on), FLOAT (both off)
static inline void phaseA_PWM(void) {
    if (cfg.comp_pwm) {             // damped light: synchronous rectification
        TIM1->CCMR1 = (TIM1->CCMR1 & ~TIM_CCMR1_OC1M) | (6 << 4); // PWM1
        TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
    } else {
        TIM1->CCMR1 = (TIM1->CCMR1 & ~TIM_CCMR1_OC1M) | (6 << 4);
        TIM1->CCER = (TIM1->CCER | TIM_CCER_CC1E) & ~TIM_CCER_CC1NE;
        LL_GPIO_ResetOutputPin(PHASE_A_LOW_PORT, PHASE_A_LOW_PIN); // if GPIO low side
    }
}
static inline void phaseA_LOW(void) {
    // force OC1 inactive, OC1N active -> lowside on 100%
    TIM1->CCMR1 = (TIM1->CCMR1 & ~TIM_CCMR1_OC1M) | (4 << 4);   // force inactive
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
}
static inline void phaseA_FLOAT(void) {
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
}
// phaseB_*, phaseC_* identical on CH2/CH3
```

### 3.4 Commutation

```c
void comStep(uint8_t s) {
    switch (s) {
    case 1: phaseC_FLOAT(); phaseB_LOW(); phaseA_PWM(); break;
    case 2: phaseA_FLOAT(); phaseB_LOW(); phaseC_PWM(); break;
    case 3: phaseB_FLOAT(); phaseA_LOW(); phaseC_PWM(); break;
    case 4: phaseC_FLOAT(); phaseA_LOW(); phaseB_PWM(); break;
    case 5: phaseA_FLOAT(); phaseC_LOW(); phaseB_PWM(); break;
    case 6: phaseB_FLOAT(); phaseC_LOW(); phaseA_PWM(); break;
    }
    TIM1->EGR = TIM_EGR_COMG;   // latch preloaded CCER/OCxM atomically (if CCPC set)
}

static const uint8_t comp_input_for_step[7] = {0, PH_C, PH_A, PH_B, PH_C, PH_A, PH_B};

void commutate(void) {
    if (forward) { if (++step > 6) { step = 1; desync_check = 1; } rising = step & 1; }
    else         { if (--step < 1) { step = 6; desync_check = 1; } rising = !(step & 1); }

    if (!prop_brake_active) comStep(step);

    changeCompInput();           // mux comparator to floating phase, set edge polarity
    if (average_interval > 1700) // low speed: blank longer
        old_routine = old_routine; // (keep polling mode until stable)
    bemfcounter = 0;
    zcfound = 0;
}

void changeCompInput(void) {
    COMP_SelectInput(comp_input_for_step[step]);
    EXTI->RTSR = rising ? COMP_EXTI_LINE : 0;
    EXTI->FTSR = rising ? 0 : COMP_EXTI_LINE;
}
```

### 3.5 Zero-Cross Interrupt and Interval Filter

```c
#define FILTER_LEVEL_FAST   2
#define FILTER_LEVEL_SLOW   12

void interruptRoutine(void) {           // Comparator EXTI ISR
    // Software filter: comparator must hold for 'filter_level' reads.
    // Rejects PWM-switching noise coupled into the floating phase.
    for (uint8_t i = 0; i < filter_level; i++) {
        if (rising ? !COMP_OUT() : COMP_OUT()) return;   // bounce -> ignore
    }

    __disable_irq();
    maskPhaseInterrupts();                // no more ZC until next commutation
    thiszctime = INTERVAL_TIMER->CNT;
    INTERVAL_TIMER->CNT = 0;

    // IIR-ish: weight new measurement 1:1 with previous (AM32 uses (old+new)/2)
    commutation_interval = (commutation_interval + thiszctime) >> 1;

    // advance: (interval / 64) * level  -> 0.9375 deg per level
    advance  = (commutation_interval >> 6) * cfg.advance_level;
    waitTime = (commutation_interval >> 1) - advance;
    if (waitTime < 2) waitTime = 2;

    COM_TIMER->CNT = 0;
    COM_TIMER->ARR = waitTime;
    COM_TIMER->SR  = 0;
    COM_TIMER->DIER |= TIM_DIER_UIE;     // fire commutation after waitTime
    __enable_irq();
}

void PeriodElapsedCallback(void) {      // COM_TIMER update ISR
    COM_TIMER->DIER &= ~TIM_DIER_UIE;
    commutate();
    e_com_time = ((commutation_interval * 6) + e_com_time * 3) >> 2; // smoothed
    if (zero_crosses < 10000) zero_crosses++;
    enableCompInterrupts();              // after blanking (see below)
}
```

### 3.6 Blanking and Filter Scheduling

After commutation, the de-energised phase rings through its freewheel diode. **Demag time** is the interval during which the floating phase is clamped to a rail by that freewheel diode. It scales with current and with interval.

```c
// called in 20kHz loop
void updateFilterAndBlanking(void) {
    if (zero_crosses < 30 || commutation_interval > 4000) filter_level = FILTER_LEVEL_SLOW;
    else filter_level = FILTER_LEVEL_FAST + (commutation_interval > 1000);

    // Blank ~1/4 of interval before ZC interrupt is re-armed (enableCompInterrupts
    // is called from a one-shot after blank_ticks in production code)
    blank_ticks = commutation_interval >> 2;
}
```

**Demag compensation (optional, AM32 "stuck rotor/desync protection").** If a zero-cross arrives in less than about 1/8 of the expected interval, it is treated as a demag artefact:

- ignore it, and
- increment `bad_count`.

If `bad_count` exceeds a threshold, cut duty and restart.

### 3.7 Startup (Polled Mode)

```c
void startMotor(void) {
    if (!running) {
        commutate();
        commutation_interval = 10000;
        INTERVAL_TIMER->CNT = 5000;
        running = 1;
        old_routine = 1;
    }
    enableCompInterrupts();
}

// In main loop while old_routine: poll comparator, switch to interrupt
// mode once zero_crosses > 20 and interval stable.
void getBemfState(void) {
    uint8_t c = COMP_OUT();
    if (rising ? c : !c) bemfcounter++; else bad_count++;
    if (bemfcounter > min_bemf_counts_up) { zcfoundroutine(); }
}
```

---

## 4. High-Resolution DShot1200 Decoding

### 4.1 Signal Timing

| Protocol | Bit time | T1H ("1" high) | T0H ("0" high) | Frame (16 bits) |
|----------|----------|----------------|----------------|-----------------|
| DShot300 | 3.333 µs | 2.500 µs | 1.250 µs | 53.3 µs |
| DShot600 | 1.667 µs | 1.250 µs | 0.625 µs | 26.7 µs |
| DShot1200 | 0.833 µs | 0.625 µs | 0.3125 µs | 13.3 µs |

**Frame format:** `[11 bit value][1 bit telemetry request][4 bit CRC]`, MSB first.

**Resolution budget at 64 MHz timer clock (PSC = 0):**

| Quantity | Ticks |
|---|---|
| Bit period | 53.3 |
| T1H | 40 |
| T0H | 20 |

The decision margin is ±10 ticks, which comfortably absorbs jitter. Because DShot1200 is too fast for per-edge ISRs, **capture all edges with DMA**.

### 4.2 Capture Setup

```c
#define DSHOT_EDGES 32
volatile uint32_t dma_buffer[64];   // room for bidirectional / glitch tolerance

void dshot_ic_init(void) {
    IC_TIMER->PSC  = 0;                 // full speed (e.g. 64 MHz)
    IC_TIMER->ARR  = 0xFFFF;
    // CH1 input capture on both edges, filter = 0 (DShot1200 needs it unfiltered,
    // or N=2 at fCK_INT for slight noise rejection)
    IC_TIMER->CCMR1 = TIM_CCMR1_CC1S_0;              // IC1 -> TI1
    IC_TIMER->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP; // both edges
    IC_TIMER->DIER  = TIM_DIER_CC1DE;                // DMA on capture

    DMA_CH->CPAR  = (uint32_t)&IC_TIMER->CCR1;
    DMA_CH->CMAR  = (uint32_t)dma_buffer;
    DMA_CH->CNDTR = DS