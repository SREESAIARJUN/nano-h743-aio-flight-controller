# ESP32-S3 Mainboard: 3× TMC6300 + 3× AS5600 + 2× ICM-42688-P

## Before you use this

- **I don't have your Cycle 1–3 files.** This is a self-consistent reference design, not a continuation of your earlier revisions. Diff it against your previous cycle before merging.
- **A hand-typed netlist is not how KiCad 8 is normally used.** Pcbnew pulls connectivity from the schematic ("Update PCB from Schematic"). What makes this usable is the **net-by-net connection spec (§4)**. Enter it into Eeschema, run ERC, then export. The S-expression in §6 shows the format and covers the MCU-side nets.
- **Check every pin number against current datasheets.** This applies especially to the **TMC6300**: pins below are given **by name** because symbol/footprint numbering varies by library. Also check the ICM-42688-P LGA-14 numbering. "Production-ready" requires your own ERC/DRC and a prototype spin. No design should go to fab on the basis of a chat answer alone.

---

## 1. Architecture

```
VBAT 7–11 V ──► Reverse-pol P-FET ──► Fuse ──► VMOT rail ──► 3× TMC6300 (VS)
                                         │
                              BAT54C OR ─┴─ USB VBUS 5 V
                                         ▼
                                 AP63203 buck → +3V3 (2 A)
                                         ▼
          ESP32-S3-WROOM-1-N16R8 ── 18× PWM ──► TMC6300 ×3
             │ I2C0 ── TCA9548A mux ──► AS5600 ×3 (all fixed at 0x36)
             │        └─ PCA9555 I/O expander (STANDBY, DIAG×3, LEDs)
             │ SPI2 (FSPI) ──► ICM-42688-P ×2 (CS1/CS2, INT1 each)
             └ USB-C native (GPIO19/20) – CDC + JTAG debug
```

Key decisions:

| Problem | Solution |
|---|---|
| All three AS5600 parts are fixed at I²C address 0x36, and the S3 has only 2 I²C controllers | TCA9548A mux (0x70), channels 0–2 |
| 18 PWM outputs are needed | MCPWM0 + MCPWM1 give 12 outputs (M1, M2). LEDC gives 6 (M3). See note in §7. |
| GPIO budget is tight on N16R8, where GPIO26–32 (flash) and 35–37 (octal PSRAM) are unusable | Slow signals (STANDBY, DIAG, LEDs) go on a PCA9555 (0x20) |
| GPIO45 strap (VDD_SPI voltage) must be low at boot | Left unconnected (10 k pulldown only) |
| UART0 pins are needed for IMU interrupts | Debug runs over native USB-CDC/JTAG. GPIO43/44 are reused. |

---

## 2. ESP32-S3-WROOM-1 GPIO Allocation (module pin numbers)

| Mod pin | GPIO | Net | Notes |
|---|---|---|---|
| 1, 40, 41 | — | GND | EPAD needs a via array |
| 2 | — | +3V3 | 10 µF + 100 nF |
| 3 | EN | /EN | 10 k pull-up + 1 µF to GND, RESET button |
| 4 | IO4 | M1_UH | MCPWM0 op0 A |
| 5 | IO5 | M1_UL | MCPWM0 op0 B |
| 6 | IO6 | M1_VH | MCPWM0 op1 A |
| 7 | IO7 | M1_VL | MCPWM0 op1 B |
| 8 | IO15 | M1_WH | MCPWM0 op2 A |
| 9 | IO16 | M1_WL | MCPWM0 op2 B |
| 10 | IO17 | M2_UH | MCPWM1 op0 A |
| 11 | IO18 | M2_UL | MCPWM1 op0 B |
| 12 | IO8 | M2_VH | MCPWM1 op1 A |
| 13 | IO19 | USB_D− | Native USB |
| 14 | IO20 | USB_D+ | Native USB |
| 15 | IO3 | VBAT_SEN