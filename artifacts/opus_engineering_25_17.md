# ESP32-S3 3-Axis FOC Mainboard: Schematic Design Package

## Read this first

- **I don't have your "Cycle 25" context.** I don't have your earlier revisions, constraints, or design history. This package is a complete, self-consistent design built from your stated parts. Reconcile it against your previous cycle.
- **Treat this as a design starting point, not a production-ready file.** "Production-ready" requires ERC/DRC in your KiCad libraries, a datasheet pin-number check, and a bring-up spin. I haven't run it through a simulator or fabricated it.
- **Connections are given as pin names, not numbers.** A KiCad `.net` file keys on pin numbers, which depend on your exact symbol and footprint. Copying pin numbers from memory is the most common way generated netlists end up silently wrong. Enter the connection tables below in Eeschema and export the netlist (File → Export → Netlist). That gives you a netlist that is guaranteed consistent with your libraries.
- **Pins to verify against current datasheets:** TMC6300 (QFN20), ICM-42688-P (LGA-14), AS5600 (SOIC-8), TCA9548A (TSSOP-24).

---

## 1. Architecture and key decisions

| Decision | Choice | Reason |
|---|---|---|
| MCU | ESP32-S3-WROOM-1-**N16R8** | Octal PSRAM uses GPIO35–37, so those pins are excluded below |
| Motor supply (VM) | 2S LiPo, 6.0–8.4 V | TMC6300 VS operating range is 2–11 V. **Do not use 3S.** |
| PWM mode | 6-PWM per driver (18 pins) | TMC6300 has separate HS/LS inputs |
| PWM peripherals | M1 on MCPWM0, M2 on MCPWM1, M3 on LEDC | The S3 has only 12 MCPWM outputs (see note below) |
| Encoders | 3× AS5600 behind a TCA9548A mux | All AS5600s share the fixed address 0x36 |
| IMUs | 2× ICM-42688-P on shared SPI2, separate CS | Fast and deterministic. Gives redundancy or a two-location fusion setup. |
| Current sense | **None** (voltage-mode FOC) | GPIO budget is exhausted. Fine for gimbal-class motors. |
| USB | Native USB-Serial/JTAG on GPIO19/20 | Frees the UART0 pins for I²C |

**Motor 3 PWM caveat:** LEDC has no hardware complementary/dead-time pairing. Motor 3 relies on the TMC6300's internal break-before-make logic plus software phase alignment. If you need hardware dead-time on all three axes, use one of these options:
- Move to a second MCU.
- Use a GPIO expander for STBY/DIAG and free pins.
- Use a driver with 3-PWM input (e.g., DRV8313).

---

## 2. ESP32-S3 GPIO allocation (final)

| GPIO | Net | Notes |
|---|---|---|
| 0 | BOOT_BTN | 10k pull-up, button to GND |
| 1 | VM_SENSE | ADC1_CH0, divider 100k/22k → 8.4 V maps to 1.51 V |
| 2 | IMU2_INT1 | |
| 3 | IMU1_INT1 | Strap (JTAG source). Harmless unless the eFuse is burned. |
| 4 / 5 | M1_UH / M1_UL | MCPWM0 op0 |
| 6 / 7 | M1_VH / M1_VL | MCPWM0 op1 |
| 15 / 16 | M1_WH / M1_WL | MCPWM0 op2 |
| 17 / 18 | M2_UH / M2_UL | MCPWM1 op0 |
| 8 / 9 | M2_VH / M2_VL | MCPWM1 op1 |
| 10 / 11 | M2_WH / M2_WL | MCPWM1 op2 |
| 38 / 39 | M3_UH / M3_UL | LEDC |
| 40 / 41 | M3_VH / M3_VL | LEDC (JTAG pins; use USB-JTAG instead) |
| 42 / 47 | M3_WH / M3_WL | LEDC |
| 12 | SPI_SCK | SPI2 via GPIO matrix |
| 13 | SPI_MOSI | |
| 14 | SPI_MISO | |
| 21 | IMU1_CS | 10k pull-up |
| 48 | IMU2_CS | 10k pull-up |
| 43 | I2C_SDA | Formerly U0TXD; console moves to USB-CDC |
| 44 | I2C_SCL | Formerly U0RXD |
| 45 | DRV_FAULT | Strap: must be LOW at boot. 10k pull-down; diode-OR of the DIAG pins. |
| 46 | DRV_STBY_N | Strap: must be LOW for download mode. 10k pull-down keeps drivers in standby at boot. |
| 19 / 20 | USB_D− / USB_D+ | Native USB |
| EN | ESP_EN | 10k pull-up + 1 µF to GND, RESET button to GND |

All 31 usable GPIOs are consumed. Any feature addition requires an I/O expander (e.g., a PCA9535 on the existing I²C bus at 0x20).

**Check the STBY polarity** against the TMC6300 datasheet. The intent is drivers disabled while GPIO46 is low at boot. If STBY is active-high = standby, add an inverter (74LVC1G04) or move the net.

---

## 3. Schematic sheets and connections

### Sheet 1: Power

**Input and protection**
- **J1** (XT