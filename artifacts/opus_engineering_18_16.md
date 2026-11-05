# ESP32-S3 Triple-FOC Mainboard: Netlist and Schematic Reference

## Before you use this

- **I have no access to Cycles 1–17.** I can't see your earlier pin assignments, power topology, or design decisions. This design is built from scratch from the part list. Reconcile it against your previous revision before merging.
- **This is not "production-ready" until it passes your own review.** Production readiness comes from ERC/DRC, datasheet cross-checks, bring-up of prototype boards, and thermal and EMC testing. Text output can't substitute for that. Pin numbers I'm confident in are given numerically. Pins you **must verify** against the datasheet and your library symbol are marked `VERIFY`.
- **A hard constraint drives the architecture.** Three TMC6300s in 6-PWM mode need **18 PWM pins**. The ESP32-S3's MCPWM peripheral provides only **12 hardware-complementary outputs** (2 units × 3 operators × 2). This design handles it as described in section 2.

---

## 1. System architecture

```
 VIN (2S LiPo 6.0–8.4 V, TMC6300 abs max 11 V)
  │
  ├─► Reverse-polarity P-FET + TVS (SMBJ10A) + 100 µF bulk ──► VMOT
  │         │
  │         ├──► TMC6300 #1 ─► Motor 1 (U/V/W)
  │         ├──► TMC6300 #2 ─► Motor 2
  │         └──► TMC6300 #3 ─► Motor 3
  │
  └─► Buck TPS62160 (3–17 V in, 1 A) ──► +3V3
                                          ├─ ESP32-S3-WROOM-1-N16R8
                                          ├─ TCA9548A ─┬─ AS5600 #1 (ch0)
                                          │            ├─ AS5600 #2 (ch1)
                                          │            └─ AS5600 #3 (ch2)
                                          ├─ ICM-42688-P #1 (SPI, CS0)
                                          ├─ ICM-42688-P #2 (SPI, CS1)
                                          └─ TCA9535 GPIO expander (DIAG/VIO-enable)
 USB-C (native USB D-/D+ on GPIO19/20) ──► programming/debug, 5 V via Schottky OR
```

**Key decisions:**

| Issue | Decision | Rationale |
|---|---|---|
| AS5600 has a fixed I²C address (0x36) | TCA9548A mux on I2C0 | Three identical addresses need a mux. Use 1 MHz Fast-mode Plus if wiring permits. |
| IMU bandwidth | Both ICM-42688-P on SPI2 (FSPI), separate CS | Up to 24 MHz SPI, deterministic timing, with INT1 to the MCU for each. |
| GPIO shortage | TMC6300 DIAG + enable via TCA9535 on I2C0 | These signals are not timing-critical. |
| Octal PSRAM (N16R8) | GPIO35/36/37 **unused** | They are consumed by the OPI PSRAM. |
| Strapping pins | GPIO0 = BOOT button only; GPIO3, 45, 46 unused | Avoids boot-mode failures. |
| Current sensing | Not on MCU ADC (no pins left); optional ADS7038 on SPI | See section 6. |

---

## 2. GPIO allocation (ESP32-S3-WROOM-1-N16R8)

Usable pins after excluding flash/PSRAM (26–32, 35–37) and straps (0, 3, 45, 46): **1, 2, 4–18, 21, 38–44, 47, 48** gives **27 pins**. UART0 (43/44) is reclaimed because programming goes through native USB.

| Function | GPIO | Module pin | Peripheral |
|---|---|---|---|
| M1_UH / M1_UL | 4 / 5 | 4 / 5 | MCPWM0 OP0 A/B |
| M1_VH / M1_VL | 6 / 7 | 6 / 7 | MCPWM0 OP1 A/B |
| M1_WH / M1_WL | 15 / 16 | 8 / 9 | MCPWM0 OP2 A/B |
| M2_UH / M2_UL | 17 / 18 | 10 / 11 | MCPWM1 OP0 A/B |
| M2_VH / M2_VL | 8 / 9 | 12 / 17 | MCPWM1 OP1 A/B |
| M2_WH / M2_WL | 10 / 11 | 18 / 19 | MCPWM1 OP2 A/B |
| M3_UH / M3_UL | 12 / 13 | 20 / 21 | LEDC ch0/ch1 (hpoint dead-time) |
| M3_VH / M3_VL | 14 / 21 | 22 / 23 | LEDC ch2/ch3 |
| M3_WH / M3_WL | 47 / 48 | 24 / 25 | LEDC ch4/ch5 |
| SPI_SCK | 39 | 32 | FSPI (GPIO matrix) |
| SPI_MOSI | 40 | 33 | FSPI |
| SPI_MISO | 41 | 34 | FSPI |
| IMU1_CS | 42 | 35 | GPIO |
| IMU2_CS | 38 | 31 | GPIO |
| IMU1_INT1 | 1 | 39 | GPIO IRQ |
| IMU2_INT1 | 2 | 38 | GPIO IRQ |
| I2C0_SDA | 43 | 37 | I2C0 |
| I2C0_SCL | 44 | 36 | I2C0 |
| USB_D− / USB_D+ | 19 / 20 | 13 / 14 | USB-Serial-JTAG |
| BOOT | 0 | 27 | Button to GND, 10 k pull-up |
| EN | — | 3 | 10 k pull-up + 1 µF RC + reset button |

**Motor 3 on LEDC:** the LEDC peripheral produces center-unaligned PWM, with dead time emulated via `hpoint` offsets. Before committing:

1. Verify that the TMC6300's internal break-before-make protection is sufficient for your LEDC timing (`VERIFY` in the datasheet).
2. Or run motor 3 in 3-PWM mode with an external complementary/dead-time generator.
3. Or move to a two-MCU or co-processor arrangement.

Note that SimpleFOC's ESP32 6-PWM driver only supports MCPWM, so you'll need custom driver code for motor 3.

**Note on GPIO43/44:** these output UART0 boot logs at power-up. Glitches on I2C at boot are harmless, but make sure the I²C devices tolerate them. Alternatively, swap I2C to GPIO3/46 only if you understand those strap constraints.

---

## 3. Per-block schematic details

### 3.1 Power
- **J1** VIN (XT30 or 2-pin JST-VH) → **Q1** P-MOSFET reverse protection (e.g., DMG2305UX; gate to GND via 10 k, 10 V Zener G-S) → **D1** SMBJ10A TVS → **C1** 100 µF/25 V polymer + **C2** 10 µF/25 V X7R → net `VMOT`.
- **U2** TPS62160DGK: VIN = VMOT, 2.2 µH inductor (XFL4020-222), 22 µF output, FB divider for 3.3 V (consult datasheet table), EN tied to VIN through 100 k → `+3V3`.
- USB-C VBUS → **D2** Schottky → optional 5 V-to-3V3 path via OR-ing diode into the buck input. Its 3 V minimum input allows USB-only bring-up with motors off. CC1/CC2 each get 5.1 k to GND.

### 3.2 TMC6300 (×3, identical; designators U3/U4/U5)
- **VS**: VMOT, 10 µF + 100 nF placed directly at the pin.
- **Charge pump**: 22 nF CPI–CPO, 100 nF VCP–VS (`VERIFY` values in the datasheet's application circuit).
- **VIO**: driven from a TCA9535 output through 1 k, with 100 nF. VIO low = standby, which gives per-driver enable.
- **BRUV / BRW**: to GND via sense resistors (e.g., 0.1 Ω 