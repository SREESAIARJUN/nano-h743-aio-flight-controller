# ESP32-S3 3-Axis FOC Mainboard: Schematic Design Package

## Scope and Verification Notice

I don't have context from earlier design cycles ("Cycle 24"), so this design starts fresh from your stated requirements. I can't honestly hand you a file that is production-ready without review. No netlist written outside an ERC/DRC-checked KiCad project should go to fab unverified.

This package gives you:

1. A complete, consistent architecture and GPIO map.
2. A net-by-net connection table that you can capture directly in Eeschema.
3. A KiCad-format netlist for the subsystems whose pinouts I'm confident in.
4. A BOM, layout rules, and a verification checklist.

Pins marked **⚠VERIFY** must be checked against the current manufacturer datasheet before capture.

---

## 1. Critical Design Decisions (Read First)

| Issue | Problem | Resolution in this design |
|---|---|---|
| **AS5600 address** | All AS5600s have a fixed I²C address (0x36). Three cannot share one bus. | A TCA9548A I²C mux (0x70) puts each encoder on its own channel. |
| **PWM count** | 3× TMC6300 in 6-PWM mode needs 18 PWM outputs. The ESP32-S3 MCPWM peripheral has only 12. | 3-PWM mode: each xH input is driven directly, and each xL input gets the inverted signal from a 74LVC3G04. That needs 9 PWM pins, all fitting on MCPWM0/1. The TMC6300's internal break-before-make provides dead time. |
| **TMC6300 voltage** | The absolute maximum is 12 V and the operating range is 2–11 V. | Motor input is limited to 2S Li-ion (≤ 8.4 V). Use large bulk capacitance near the bridges. A standard TVS clamps too high to protect a 12 V part, so don't rely on one. |
| **Octal PSRAM** | The N16R8/N8R8 modules use GPIO35–37 internally. | Those pins are unused here. |
| **Strapping pins** | GPIO0, 3, 45, and 46 affect boot. | Only GPIO0 (BOOT button) is used. The others are left unconnected. |
| **IMU bus** | Two ICM-42688-P on I²C would compete with the encoders. | Both IMUs sit on SPI2 (FSPI) at up to 24 MHz, each with its own chip select. |

---

## 2. Block Diagram

```
 VIN (6–8.4V, XT30) ──F1──Q1(rev-pol PMOS)──┬── VM rail ──► 3× TMC6300 VS (+ bulk caps)
                                             │
                                             └─D1─┐
 USB-C VBUS(5V) ──────────────F2──────────────D2─┴─► VBUCK ─► U2 AP63203 ─► +3V3
                                                                         │
 USB-C D+/D- ── ESD(U9) ── ESP32-S3 GPIO19/20 (native USB-JTAG/CDC)      │
                                                                         ▼
 ┌───────────── ESP32-S3-WROOM-1 (U1) ─────────────┐   +3V3 → U1, U3–U8, U10–U12, encoders
 │ MCPWM ×9 ──► U10/U11/U12 74LVC3G04 ──► U3/U4/U5 TMC6300 ──► M1/M2/M3 (J3/J4/J5)
 │ I2C0 ──► U6 TCA9548A ──ch0/1/2──► J6/J7/J8 → AS5600 boards (or on-board U13–U15)
 │ SPI2 ──► U7 ICM-42688-P (CS0), U8 ICM-42688-P (CS1)
 │ ADC1_CH8 ◄── VM divider
 └──────────────────────────────────────────────────┘
```

---

## 3. GPIO Allocation (ESP32-S3-WROOM-1)

| Module pin | GPIO | Net | Function |
|---|---|---|---|
| 1, 40, 41 | — | GND | Ground and exposed pad |
| 2 | — | +3V3 | Supply |
| 3 | EN | ESP_EN | 10 k pull-up, 1 µF to GND, RESET button |
| 4 | IO4 | M1_PWM_A | MCPWM0 |
| 5 | IO5 | M1_PWM_B | MCPWM0 |
| 6 | IO6 | M1_PWM_C | MCPWM0 |
| 7 | IO7 | M2_PWM_A | MCPWM0 |
| 8 | IO15 | M2_PWM_B | MCPWM0 |
| 9 | IO16 | M2_PWM_C | MCPWM0 |
| 10 | IO17 | M3_PWM_A | MCPWM1 |
| 11 | IO18 | M3_PWM_B | MCPWM1 |
| 12 | IO8 | M3_PWM_C | MCPWM1 |
| 13 | IO19 | USB_DN | Native USB |
| 14 | IO20 | USB_DP | Native USB |
| 15 | IO3 | NC | Strapping pin, leave unconnected |
| 16 | IO46 | NC | Strapping pin, leave unconnected |
| 17 | IO9 | VM_SENSE | ADC1_CH8, through a 100 k / 22 k divider |
| 18 | IO10 | IMU0_CS | SPI2 chip select |
| 19 | IO11 | SPI_MOSI | SPI2 |
| 20 | IO12 | SPI_SCK | SPI2 |
| 21 | IO13 | SPI_MISO | SPI2 |
| 22 | IO14 | IMU1_CS | SPI2 chip select |
| 23 | IO21 | IMU0_INT1 | Input |
| 24 | IO47 | IMU1_INT1 | Input |
| 25 | IO48 | LED_STATUS | 1 k resistor to LED |
| 26 | IO45 | NC | Strapping pin |
| 27 | IO0 | BOOT | 10 k pull-up, BOOT button to GND |
| 28–30 | IO35–37 | NC | Used by octal PSRAM |
| 31 | IO38 | M1_DIAG | 10 k pull-up |
| 32 | IO39 | M2_DIAG | 10 k pull-up |
| 33 | IO40 | M3_DIAG | 10 k pull-up |
| 34 | IO41 | M1_EN | Drives TMC6300 VIO/STANDBY |
| 35 | IO42 | M2_EN | Drives TMC6300 VIO/STANDBY |
| 36 | RXD0 | UART0_RX | Debug header J9 |
| 37 | TXD0 | UART0_