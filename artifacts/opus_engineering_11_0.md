# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

This is an original reference implementation written in the style of AM32's architecture. It is not the upstream AM32 source. It targets an STM32G071 at 64 MHz with:

- TIM1 producing 3-phase complementary PWM
- COMP1/2/3 doing back-EMF zero-cross detection
- TIM3 doing DShot input capture via DMA
- TIM14 as the commutation timer
- TIM17 as the interval timer

The "Cycle 11" label has no specific meaning in AM32. This writeup treats it as an iteration of the three subsystems you named.

---

## 1. Configuration (`config.h`)

```c
#pragma once
#include <stdint.h>

#define CPU_HZ                 64000000UL
#define PWM_FREQ_HZ            24000
#define PWM_ARR                (CPU_HZ / PWM_FREQ_HZ)      // 2666
#define DEADTIME_NS            300

// Motor timing
#define TIMING_ADVANCE_DEG     15      // 0..30 electrical degrees (default 15 deg is typical)
#define MIN_COMM_INTERVAL_US   20      // ~ 500k eRPM ceiling
#define MAX_COMM_INTERVAL_US   20000   // below this speed -> desync / restart
#define ZC_FILTER_MIN          2       // comparator confirmations at high speed
#define ZC_FILTER_MAX          12      // at low speed
#define BLANKING_PCT           25      // % of interval to ignore after commutation

// DShot1200: bit = 0.833 us; T0H = 0.3125 us; T1H = 0.625 us
#define DSHOT_TIMER_HZ         CPU_HZ
#define DSHOT1200_BIT_TICKS    53      // 0.833 us * 64 MHz
#define DSHOT1200_THRESH_TICKS 30      // midpoint between T0H(20) and T1H(40)
#define DSHOT_FRAME_EDGES      32      // 16 bits x (rise+fall)

// Braking
#define DRAG_BRAKE_STRENGTH    10      // 1..10, applied when throttle == 0
#define BRAKE_ON_STOP          1
#define DAMPED_LIGHT           1       // complementary PWM (synchronous rectification)
```

---

## 2. High-Resolution DShot1200 Decoding (`dshot.c`)

The decoder captures both edges with TIM3 CH1 at the full 64 MHz (PSC=0). DMA stores 32 timestamps per frame. Each bit is classified by its high time, so the decoder needs no knowledge of the protocol rate beyond the threshold.

```c
#include "config.h"
#include "stm32g0xx.h"

static volatile uint16_t dma_buf[DSHOT_FRAME_EDGES + 1];
volatile uint16_t dshot_throttle;      // 0..2047
volatile uint8_t  dshot_telem_req;
volatile uint8_t  dshot_bidir;         // inverted signal => bidirectional
volatile uint32_t dshot_good, dshot_bad;

void dshot_init(void) {
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    TIM3->PSC  = 0;
    TIM3->ARR  = 0xFFFF;
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | (0x2 << TIM_CCMR1_IC1F_Pos); // IC1 on TI1, light filter
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;  // both edges
    TIM3->DIER  = TIM_DIER_CC1DE;

    DMA1_Channel1->CPAR  = (uint32_t)&TIM3->CCR1;
    DMA1_Channel1->CMAR  = (uint32_t)dma_buf;
    DMA1_Channel1->CNDTR = DSHOT_FRAME_EDGES;
    DMA1_Channel1->CCR   = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_TCIE;
    DMAMUX1_Channel0->CCR = 32; // TIM3_CH1 request
    NVIC_SetPriority(DMA1_Channel1_IRQn, 0);   // highest: frame timing is critical
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    TIM3->CR1 |= TIM_CR1_CEN;
}

static inline void dshot_rearm(void) {
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CNDTR = DSHOT_FRAME_EDGES;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}

static void dshot_decode(void) {
    uint32_t frame = 0;

    // Detect polarity: in bidirectional (inverted) mode the first captured edge is falling.
    // The pin level after the frame tells us the idle state.
    dshot_bidir = !(