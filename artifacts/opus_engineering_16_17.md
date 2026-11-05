# ESP32-S3 Mainboard: 3× TMC6300 FOC, 3× AS5600, 2× ICM-42688-P

I don't have access to earlier "cycles" of this project, so this is a complete, self-consistent design built from scratch. It is a strong starting point, but it is **not production-ready until you run ERC/DRC and check it against the datasheets.** Section 6 lists the specific items you must confirm, including two architectural issues you need to decide on before layout.

---

## 1. System Architecture

```
 2S LiPo (6.0–8.4 V) ─┬─ Reverse-pol PMOS ─ VMOT ──┬── TMC6300 #1 ── Motor 1 (U/V/W)
                      │                            ├── TMC6300 #2 ── Motor 2
 USB-C VBUS (5 V) ──┐ │                            └── TMC6300 #3 ── Motor 3
                    └─┴─ Schottky OR ─ AP63203 (3.3 V / 2 A) ─ +3V3
                                                     │
            ┌────────────────────────────────────────┴───────────────┐
            │                ESP32-S3-WROOM-1-N16R8                  │
            │  MCPWM0 → TMC#1 (6 PWM)   MCPWM1 → TMC#2 (6 PWM)       │
            │  LEDC   → TMC#3 (6 PWM)   SPI2   → 2× ICM-42688-P      │
            │  I2C0   → TCA9548A → 3× AS5600 (JST-SH, off-board)     │
            │         → PCA9536 (DIAG readback)                      │
            │  USB-OTG (GPIO19/20) → USB-C                           │
            └────────────────────────────────────────────────────────┘
```

**Key design decisions**

- **AS5600 address conflict.** The AS5600 has a fixed I²C address (0x36), so three on one bus will collide. A TCA9548A mux (0x70) gives each encoder its own channel.
- **Encoders are off-board.** Each AS5600 must sit on the motor axis. The mainboard provides 4-pin JST-SH connectors (3V3, GND, SDA, SCL).
-