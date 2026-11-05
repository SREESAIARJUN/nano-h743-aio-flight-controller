# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking

I don't have the earlier "cycles" of this work, so this is a self-contained design. It follows AM32's architecture and naming but is written fresh. It is not a verbatim copy of AM32 source, which is GPL-3.0; if you merge it into AM32, the result falls under that license.

**Target:** STM32G071 at 64 MHz. The same design applies to AT32F421 and STM32F051 by changing the clock constants. Peripheral access goes through thin macros so you can map it onto LL drivers or direct register access.

---

## 1. System Architecture

| Function | Peripheral | Priority | Notes |
|---|---|---|---|
| DShot input capture | TIM3 CH1 + DMA, both edges | 0 (highest) | 32 edge timestamps per frame |
| Bidirectional telemetry TX | Same pin, switched to output, TIM + DMA | 0 | GCR at 5/4 of the DShot bit rate |
| Phase PWM | TIM1 CH1–3 + CH1N–3N, complementary, dead time | n/a | 24–48 kHz, center-aligned |
| Commutation timer | TIM14 or TIM17 at 1 MHz (0.5 µs/tick optional) | 1 | One-shot commutation scheduling |
| Interval timer | TIM2, free-running at 1 MHz | n/a | Zero-cross timestamps |
| BEMF comparator | COMP1/2 with input mux per phase | 1 | EXTI interrupt at high speed, polled at low speed |
| Control loop | TIM6 at 20 kHz | 2 | Throttle ramp, braking, telemetry scheduling |
| Signal-loss watchdog | TIM16 or SysTick | 3 | Failsafe |

The rule throughout: **DShot capture must preempt commutation**, so a frame is never torn. Decoding itself is deferred out of the DMA interrupt, but the edge capture is DMA-driven and therefore never delayed.

---

## 2. High-Resolution DShot1200 Decoding

### 2.1 Signal Timing

| Protocol | Bit period | "1" high time | "0" high time | Frame (16 bits) | Ticks/bit at 64 MHz |
|---|---|---|---|---|---|
| DShot300 | 3.333 µs | 2.500 µs | 1.250 µs | 53.3 µs | 213 |
| DShot600 | 1.667 µs | 1.250 µs | 0.625 µs | 26.7 µs | 107 |
| **DShot1200** | **0.833 µs** | **0.625 µs** | **0.3125 µs** | **13.3 µs** | **53** |

At 64 MHz with no prescaler, a DShot1200 "1" is about 40 ticks and a "0" is about 20 ticks. That leaves roughly ±10 ticks of margin around the decision threshold, which is plenty.

**Frame layout (MSB first):**
- Bits 15–5: throttle (11 bits).
- Bit 4: telemetry request.
- Bits 3–0: CRC = `(v ^ v>>4 ^ v>>8) & 0xF`, where `v` is the upper 12 bits.
- In bidirectional mode, the line idles low (inverted) and the CRC is also