# ESP32-S3 3-Axis FOC Mainboard: Reference Design (KiCad 8)

I don't have your "Cycle 31" project files or earlier design decisions, so this is a complete standalone reference design. The architecture, pin map, power tree and netlist are worked out. However, no AI-generated netlist is production-ready until it has passed ERC/DRC, been checked line by line against current datasheets, and been reviewed by a person. Items I'm less certain of are marked **⚠ VERIFY**.

---

## 1. Architecture and Key Design Decisions

```
 2S LiPo (6.0–8.4 V) ─XT30─► Reverse-pol PFET ─► TVS ─► VM rail ──┬─► TMC6300 #1 ─► Motor 1 ◄─ AS5600 #1 (motor-side PCB)
                                                                 ├─► TMC6300 #2 ─► Motor 2 ◄─ AS5600 #2
 USB-C (5 V) ─► Schottky OR ─────────────────────────┐           ├─► TMC6300 #3 ─► Motor 3 ◄─ AS5600 #3
                                                     ▼           │
                                      TPS62130A buck ─► +3V3 ◄───┘ (diode-OR'd input)
                                                     │
          ESP32-S3-WROOM-1-N16R8 ◄──SPI (FSPI)