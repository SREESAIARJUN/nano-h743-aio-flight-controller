# ESP32-S3 3-Axis FOC Mainboard: KiCad 8 Netlist and Schematic Package

I don't have your earlier "Cycle 29" files or design decisions, so this design is built from scratch to your component list. Treat it as a **thorough starting design, not a verified production release.** No netlist is production-ready until it passes ERC/DRC, footprint-to-datasheet pin verification, and a prototype bring-up. Where I'm less certain of a pin detail, I mark it **⚠ VERIFY**.

---

## 1. Architecture Problems Solved Up Front

These constraints would break a naive design, so they are resolved before any wiring.

| Problem | Resolution |
|---|---|
| All AS5600s have a **fixed I²C address (0x36)**, so three cannot share one bus. | Add a **TCA9548A I²C mux** (channels 0–2). An alternative is AS5600L parts, which have a programmable address. |
| 3× TMC6300 in 6-PWM mode needs **18 PWM pins**. ESP32-S3 MCPWM provides only 12 outputs (2 units × 6). | Motors 1–2 use MCPWM0/MCPWM1. Motor 3 uses **LEDC** (6 channels, no hardware dead-time). The TMC6300 has internal break-before-make. Confirm that is acceptable for your firmware (e.g., SimpleFOC). |
| GPIO budget: 18 PWM + 5 SPI + 2 IMU INT + 2 I²C = **27 pins**. | Use **ESP32-S3-WROOM-1-N16 (no octal PSRAM)**. This frees IO35/36/37. Do **not** use the N8R8/N16R8 variants. |
| The TMC6300 VS absolute max is about 11–12 V. | Motor supply limited to **2S Li-ion (6.0–8.4 V)**. |
| ICM-42688-P bandwidth for dual-IMU fusion. | Both IMUs on a shared **SPI bus** with separate CS lines and separate INT lines. |

---

## 2. GPIO Map (ESP32-S3-WROOM-1-N16)

| Function | GPIO | Module Pin | Peripheral |
|---|---|---|---|
| M1_UH / UL / VH / VL / WH / WL | 4 / 5 / 6 / 7 / 15 / 16 | 4 / 5 / 6 / 7 / 8 / 9 | MCPWM0 |
| M2_UH / UL / VH / VL / WH / WL | 17 / 18 / 8 / 9 / 10 / 11 | 10 / 11 / 12 / 17 / 18 / 19 | MCPWM1 |
| M3_UH / UL / VH / VL / WH / WL | 12 / 13 / 14 / 21 / 47 / 48 | 20 / 21 / 22 / 23 / 24 / 25 | LEDC ch0–5 |
| SPI_SCK / MOSI / MISO | 36 / 35 / 37 | 29 / 28 / 30 | SPI2 (GPIO matrix) |
| IMU1_CS / IMU2_CS | 38 / 39 | 31 / 32 | — |
| IMU1_INT / IMU2_INT | 40 / 41 | 33 / 34 | — |
| I2C_SDA / I2C_SCL | 1 / 2 | 39 / 38 | I2C0 |
| DRV_DIAG (wired-OR, ⚠ VERIFY DIAG output type) | 42 | 35 | — |
| USB D− / D+ | 19 / 20 | 13 / 14 | Native USB |
| BOOT / EN | 0 / EN | 27 / 3 | — |
| Strapping pins left unconnected | 3, 45, 46 | 15, 26, 16 | — |
| UART0 TX/RX (debug header) | 43 / 44 | 37 / 36 | — |

---

## 3. Bill of Materials (Reference Designators)

| Ref | Part | Footprint (KiCad 8 lib) |