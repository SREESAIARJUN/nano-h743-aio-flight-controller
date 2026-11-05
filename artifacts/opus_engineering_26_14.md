# ESP32-S3 Motion Mainboard: 3× TMC6300, 3× AS5600, 2× ICM-42688-P

## Read this first

**I can't call this "production-ready".** No hand-written netlist is, including mine. Before fabrication, it needs the following:

1. Capture it as a KiCad schematic. The netlist below is for review, not for direct import. In KiCad, the schematic is the source of truth, and the netlist is exported from it.
2. Run ERC and DRC.
3. Check every pin against the current datasheets. I'm confident about the ESP32-S3-WROOM-1, AS5600, ICM-42688-P and TCA9548A pinouts. **I am not confident about the TMC6300 QFN pin numbers**, so its netlist entries use pin names and are marked `VERIFY`.
4. Bring it up on a prototype.

**I also don't have the history from "Cycles 1–25".** I've made the design decisions below from first principles. If earlier cycles fixed different choices (supply voltage, connectors, current sensing, GPIO map), tell me and I'll reconcile.

---

## 1. Design problems this architecture solves

| Problem | Resolution |
|---|---|
| The AS5600 has a **fixed I²C address (0x36)**, so three can't share a bus. | Add a TCA9548A I²C mux at 0x70, with one encoder per channel. Alternative: use the AS5600 analog OUT into ADC1, which costs 3 GPIOs. |
| 3× TMC6300 in 6-PWM mode needs **18 PWM pins**. | ESP32-S3 MCPWM provides 2 units × 6 outputs = 12, which covers M1 and M2 with hardware dead-time and sync. M3 uses LEDC, so it has no hardware complementary dead-time. The TMC6300's internal cross-conduction protection must be relied on there (**verify in the datasheet**). |
| GPIO budget on the WROOM-1-N16R8 (octal PSRAM consumes IO35–37). | The full allocation is below. UART0 (IO43/44) is repurposed, and the console runs over native USB-CDC. |
| No pins remain for **phase current sensing**. | This revision uses voltage-mode FOC (SimpleFOC `TorqueControlType::voltage`). If current-mode FOC is needed, add an external ADC (e.g. an ADS7953 on the shared SPI bus) or move to a 3-PWM driver. |
| TMC6300 VM is limited to **11 V max**. | Input range is 5–10 V nominal (2S LiPo or a 5–9 V supply). A TVS clamps at about 11 V. Do not connect 3S. |

---

## 2. Block diagram

```
VIN (5–10V, XT30) ──► Reverse-pol P-FET ──► TVS ──► VM rail ──┬──► TMC6300 ×3 (VM)
                                                              └──► AP63203 buck ──► +3V3
USB-C (VBUS) ──► Schottky ──► AP63203 VIN (OR-ed with VM via Schottky)

+3V3 ──► ESP32-S3-WROOM-1, TMC6300 VIO, ICM-42688-P ×2, TCA9548A, encoder connectors

ESP32-S3
 ├─ MCPWM0  ──► U3 TMC6300 (M1) ──► J_M1 (U/V/W)
 ├─ MCPWM1  ──► U4 TMC6300 (M2) ──► J_M2
 ├─ LEDC    ──► U5 TMC6300 (M3) ──► J_M3
 ├─ SPI2    ──► U6 ICM-42688-P (IMU1), U7 ICM-42688-P (IMU2)
 ├─ I2C0    ──► U8 TCA9548A ──► CH0/1/2 ──► J_ENC1/2/3 (AS5600 on motor PCBs)
 └─ USB D+/D- ──► USB-C
```

---

## 3. GPIO allocation (ESP32-S3-WROOM-1, module pin numbers)

| Net | GPIO | Module pin | Notes |
|---|---|---|---|
| M1_UH / M1_UL | IO4 / IO5 | 4 / 5 | MCPWM0 op0 |
| M1_VH / M1_VL | IO6 / IO7 | 6 / 7 | MCPWM0 op1 |
| M1_WH / M1_WL | IO15 / IO16 | 8 / 9 | MCPWM0 op2 |
| M2_UH / M2_UL | IO17 / IO18 | 10 / 11 | MCPWM1 op0 |
| M2_VH / M2_VL | IO8 / IO9 | 12 / 17 | MCPWM1 op1 |
| M2_WH / M2_WL | IO10 / IO11 | 18 / 19 | MCPWM1 op2 |
| M3_UH / M3_UL | IO12 / IO13 | 20 / 21 | LEDC |
| M3_VH / M3_VL | IO14 / IO21 | 22 / 23 | LEDC |
| M3_WH / M3_WL | IO47 / IO48 | 24 / 25 | LEDC |
| SPI_SCK | IO40 | 33 | |
| SPI_MOSI | IO41 | 34 | |
| SPI_MISO | IO42 | 35 | |
| IMU1_CS | IO39 | 32 | 10k pull-up |
| IMU2_CS | IO38 | 31 | 10k pull-up |
| I2C_SDA | IO1 | 39 | 4.7k pull-up |
| I2C_SCL | IO2 | 38 | 4.7k pull-up |
| IMU1_INT1 | IO3 | 15 | Strapping (JTAG source). OK because the INT pin is high-Z/low at boot. |
| IMU2_INT1 | IO46 | 16 | Strapping, must be low at boot. Add a 10k pull-down. |
| DRV_STBY (all 3) | IO43 | 37 | Shared. 10k pull-down keeps the drivers in standby at boot. |
| DRV_DIAG (wired-OR) | IO44 | 36 | 3× BAT54 diode-OR plus 10k pull-down |
| USB_D− / USB_D+ | IO19 / IO20 | 13 / 14 | 22 Ω series resistors |
| BOOT | IO0 