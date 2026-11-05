# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

**Scope note:** This is an original reference implementation that follows the AM32 architecture: STM32 MCU, comparator-based BEMF sensing, timer-driven commutation, and DMA-captured DShot. It is **not** the AM32 source code. For production work, use the official AM32 repository (GPL-3.0) and treat this as a design and verification companion.

Target assumptions:
- **MCU:** STM32G071 at 64 MHz. F051 and G431 ports differ only in peripheral names.
- **TIM1:** 3-phase complementary PWM.
- **TIM3:** DShot input capture via DMA.
- **TIM14:** commutation scheduler.
- **TIM17:** free-running interval timer.
- **COMP2:** BEMF zero-cross detection.

---

## 1. Timing Budget

| Item | Value |
|---|---|
| CPU clock | 64 MHz (15.625 ns/tick) |
| DShot1200 bit period | 0.833 µs = 53.3 ticks |
| T1H / T0H | 0.625 µs (40 ticks) / 0.3125 µs (20 ticks) |
| Decision threshold | ~30 ticks (midpoint) |
| Frame length | 16 bits ≈ 13.3 µs |
| PWM frequency | 24–48 kHz (ARR = 1333–2666 at 64 MHz) |
| Max eRPM target | ~250k eRPM, giving a commutation interval of ~40 µs |

**Interrupt priorities (lowest number = highest priority):**
- 0: COMP / zero-cross
- 0: commutation timer
- 1: DShot DMA complete
- 2: 1 kHz control loop
- 3: telemetry

---

## 2. Phase Control HAL

```c
// phase.h
typedef enum { PH_FLOAT, PH_PWM, PH_LOW } phase_mode_t;

#define PHASE_A 0
#define PHASE_B 1
#define PHASE_C 2

extern volatile uint8_t comp_damped_mode;   // 1 = complementary (damped light)

static inline void phase_set(uint8_t ph, phase_mode_t m)
{
    // TIM1 CCMR modes: 0b110 = PWM1, 0b100 = force inactive, 0b101 = force active
    volatile uint32_t *ccmr = (ph < 2) ? &TIM1->CCMR1 : &TIM1->CCMR2;
    uint32_t shift = (ph & 1) ? 12 : 4;
    uint32_t ccer_en  = TIM_CCER_CC1E  << (ph * 4);
    uint32_t ccer_nen = TIM_CCER_CC1NE << (ph * 4);

    switch (m) {
    case PH_PWM:
        *ccmr = (*ccmr & ~(7u << shift)) | (6u << shift);           // PWM1
        TIM1->CCER |= ccer_en;
        if (comp_damped_mode) TIM1->CCER |= ccer_nen;               // sync rectification
        else                