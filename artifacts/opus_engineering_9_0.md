# AM32-Style ESC Firmware: Motor Timing, DShot1200 Decoding, and Active Braking (Cycle 9)

This is a **reference implementation in the style of AM32**. It is not a verbatim copy of the upstream GPL source. The target is an STM32G071 at 64 MHz, with an STM32F051 at 48 MHz noted where it differs. Register-level calls use LL-style names so you can port them to AT32 or GD32.

---

## 1. System Architecture

```
            ┌──────────────┐  DMA (edges)   ┌──────────────────┐
 Signal ───►│ TIM3 IC CH1  │───────────────►│ dshot_buffer[32] │
            └──────────────┘                └────────┬─────────┘
                                         DMA TC IRQ  │ (prio 0)
                                                     ▼
                                          computeDshotDMA()
                                                     │ newinput
            ┌──────────────┐                         ▼
 BEMF ─────►│ COMP1/2 IRQ  │──► interruptRoutine() ──► zcfound
            └──────────────┘          (prio 1)        │
                                                      ▼
            ┌──────────────┐      ┌──────────────────────────┐
            │ TIM14 (