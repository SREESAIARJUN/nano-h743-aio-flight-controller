# ESP32-S3 Triple-FOC Mainboard: Schematic Design Package and KiCad 8 Netlist

## Limits of This Package

I don't have the earlier "Cycles 1–9," so this is a self-contained design. It is a strong starting point, not a release-ready board:

1. **KiCad netlists are generated from schematics, not the other way around.** KiCad cannot rebuild an `.kicad_sch` from a `.net` file. The netlist below can be imported into Pcbnew, or used as an exact wiring reference while you draw the schematic.
2. **Pin numbers I'm confident about** (ESP32-S3-WROOM-1, AS5600, TCA9548A, ICM-42688-P, INA181, AP63203) are given numerically.
3. **TMC6300 pins are given by name only.** Map them to numbers from the current Trinamic/ADI datasheet. Do not trust any QFN pin numbering from memory, including mine.
4. "Production-ready" still requires ERC, DRC, datasheet cross-checks, a thermal review and a prototype spin. The checklist at the end lists what to verify.

---

## 1. Architecture & Key Design Decisions

```
 VBAT 2S Li-ion (6.0–8.4 V) ─► Fuse ─► Rev-pol PFET ─► INA226 shunt ─► VM rail
                                                                    ├─► 3× TMC6300 (VS)
                                                                    └─► AP63203 buck ─► +3V3
 ESP32-S3-WROOM-1-N16R8
   ├─ MCPWM0 (6 ch) ──► TMC6300 #1
   ├─ MCPWM1 (6 ch) ──► TMC6300 #2
   ├─ LEDC   (6 ch) ──► TMC6300 #3
   ├─ ADC1 ×3 ◄── INA181A1 ◄── shunt (BRx) per driver
   ├─ SPI2 ──► ICM-42688-P #1 (CS0), #2 (CS1), with INT1 each
   ├─ I2C0 ──► TCA9548A (0x70) ─► AS5600 ×3 (all fixed at 0x36)
   │         ├─ TCA9534 (0x20): STBY×3, DIAG×3, LED, spare
   │         └─ INA226 (0x40): battery V/I
   └─ USB-C native (GPIO19/20): flashing + CDC console
```

**Why it is structured this