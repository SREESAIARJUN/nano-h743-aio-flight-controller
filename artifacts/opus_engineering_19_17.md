# ESP32-S3 Mainboard: 3× TMC6300 FOC, 3× AS5600, 2× ICM-42688-P

## Scope and Limits

- **No prior context.** I don't have access to your earlier "Cycles," so this is a clean design. Paste any constraints from Cycle 18 (supply voltage, board outline, connector choices, pin assignments) and I'll reconcile them.
- **Not production-ready yet.** No schematic is production-ready until it passes datasheet cross-checks, ERC, DRC, and a bring-up spin. The connectivity and architecture below are complete and deliberate.
- **Pin numbers to verify:**
  - TMC6300 (QFN-20) and ICM-42688-P (LGA-14) are not in the stock KiCad 8 libraries.
  - Nets below are defined by **pin name**. Bind them to symbols you build from the current datasheets.
  - Numbers I'm confident of are given. Anything marked ⚠ must be checked.
- **Why no hand-written .net file.** A full hand-typed netlist for about 140 parts is a common source of silent errors. KiCad generates it from the schematic, and PCB import works from the schematic directly. I give you exact connectivity tables (the actual source of truth) plus a representative `.net` excerpt in KiCad 8 format.

---

## 1. Architecture Decisions

| Issue | Decision | Reason |
|---|---|---|
| **All AS5600s share fixed I²C address 0x36** | TCA9548A I²C mux (0x70) on the main bus | Three AS5600s cannot share one bus. |
| PWM count: 3 × 6 = 18 inputs | Motor 1 on MCPWM0, Motor 2 on MCPWM1, Motor 3 on LEDC (6 ch) | The S3 has only 2 MCPWM units (6 outputs each). TMC6300 has internal break-before-make, so LEDC without hardware dead time is acceptable. |
| GPIO budget is tight | Native USB-CDC for console, which frees GPIO43/44. STANDBY and DIAG lines go on a TCA9534 I/O expander (0x20). | Avoids strapping pins 0, 45, 46. |
| TMC6300 VS max ≈ 11 V | Input is **2S LiPo (6.0–8.4 V)** only | 3S (12.6 V) would exceed the abs-max rating. Add a TVS diode. |
| Octal PSRAM (N16R8) | GPIO35/36/37 are **not** used | Reserved by PSRAM. |
| FOC current sensing | BR pins go to GND through 0 Ω links, with an optional shunt footprint | Voltage-mode FOC to start. Adding inline current sensing later needs amplifiers and ADC1 pins. |

---

## 2. Bill of Materials (core)

| Ref | Part | Footprint |
|---|---|---|
| U1 | ESP32-S3-WROOM-1-N16R8 | RF_Module:ESP32-S3-WROOM-1 |
| U2–U4 | TMC6300-LA | QFN-20-1EP_3x3mm_P0.4mm ⚠ verify |
| U5, U6 | ICM-42688-P | LGA-14_2.5x3mm ⚠ custom |
| U7 | TCA9548APWR | TSSOP-24 |
| U8 | TCA9534P