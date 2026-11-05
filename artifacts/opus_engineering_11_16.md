# ESP32-S3 3-Axis FOC Mainboard: Schematic Design and KiCad 8 Netlist

## Scope and Limits

- **No prior context.** I don't have previous cycles of this design, so this is a complete standalone design. If Cycle 10 used different pin assignments, power architecture, or parts, give me the deltas and I'll reconcile.
- **Not production-ready yet.** No netlist is production-ready until someone has verified every pin number against the datasheets and the KiCad symbols and footprints you actually use, run ERC/DRC, and reviewed the design.
- **Netlist strategy.** A hand-written netlist that disagrees with your symbol pinouts is worse than none. So I give you:
  1. A **net-by-net connection table**, keyed by pin *name*. This is the source of truth for drawing the schematic.
  2. A **KiCad 8 `.net` file**, using pin numbers I'm confident in. For the TMC6300, pin numbers are marked **VERIFY**. Re-export the netlist from Eeschema after drawing; don't hand-import it into a PCB.

---

## 1. System Architecture

```
 VBAT 6–11 V (2S LiPo) ──► Reverse-pol PFET ──► VM rail ──┬─► TMC6300 ×3 (VS)
                                                          │
 USB-C 5 V ──► Schottky ──────────────────────────────────┴─► TPS62162 buck ─► +3V3
                                                                               │
  ┌─────────────────────────── +3V3 ───────────────────────────────────────────┤
  │                                                                            │
  ESP32-S3-WROOM-1-N16R8                                                       │
   ├─ MCPWM0 (6ch) ─► TMC6300 #1 ─► Motor 1                                    │
   ├─ MCPWM1 (6ch) ─► TMC6300 #2 ─► Motor 2                                    │
   ├─ LEDC   (6ch) ─► TMC6300 #3 ─► Motor 3                                    │
   ├─ SPI2 (FSPI) ──► ICM-42688-P #1 (CS1), ICM-42688-P #2 (CS2)               │
   ├─ I2C0 ─► TCA9548A ─► ch0 AS5600 #1, ch1 AS5600 #2, ch2 AS5600 #3          │
   └─ USB native (GPIO19/20) ─► USB-C                                          │
```

### Key Design Decisions

| Issue | Decision |
|---|---|
| All AS5600s have the fixed I²C address 0x36 | TCA9548A mux at 0x70. The alternative is the analog OUT pin into ADC1, but there are no spare pins. |
| 18 PWM lines needed, but MCPWM provides only 12 | Motors 1–2 use MCPWM with hardware dead-time. Motor 3 uses LEDC. Use LEDC complementary phase offset in firmware, or run motor 3 in 3-PWM mode (see §6). |
| Octal PSRAM (N16R8) occupies GPIO35–37 | Not used. |
| Strapping pins GPIO0/3/45/46 | GPIO45 is **never** pulled up (doing so selects 1.8 V flash and bricks boot). GPIO46 serves as driver enable with a pulldown, so drivers are off at boot. |
| UART0 (GPIO43/44) | Reused for I²C. Console and flashing go over native USB-CDC/JTAG. |
| TMC6300 VS limit is 11 V | 2S LiPo (8.4 V max). A TVS clamps regen spikes. **Do not use 3S.** |

---

## 2. GPIO Map

| GPIO | Net | Function | Notes |
|---|---|---|---|
| 0 | BOOT | Boot button | 10k pull-up, SW to GND |
| 1 | IMU1_CS | SPI CS | 10k pull-up |
| 2 | IMU2_CS | SPI CS | 10k pull-up |
| 3 | DRV_DIAG | Diode-OR of 3× DIAG | 10k pulldown (strap-safe) |
| 4 | M1_UH | MCPWM0 | |
| 5 | M1_UL | MCPWM0 | |
| 6 | M1_VH | MCPWM0 | |
| 7 | M1_VL | MCPWM0 | |
| 15 | M1_WH | MCPWM0 | |
| 16 | M1_WL | MCPWM0 | |
| 8 | M2_UH | MCPWM1 | |
| 9 | M2_UL | MCPWM1 | |
| 10 | M2_VH | MCPWM1 | |
| 11 | M2_VL | MCPWM1 | |
| 12 | M2_WH | MCPWM1 | |
| 13 | M2_WL | MCPWM1 | |
| 14 | M3_UH | LEDC | |
| 17 | M3_UL | LEDC | |
| 18 | M3_VH | LEDC | |
| 21 | M3_VL | LEDC | |
| 38 | M3_WH | LEDC | |
| 47 | M3_WL | LEDC | |
| 39 | IMU1_INT1 | Input | Gives up USB-JTAG pad-JTAG (built-in USB-JTAG still works) |
| 48 | IMU2_INT1 | Input | |
| 40 | SPI_SCK | FSPI via GPIO matrix | |
| 41 | SPI_MOSI | | |
| 42 | SPI_MISO | | |
| 43 | I2C_SDA | | 2.2k pull-up |
| 44 | I2C_SCL | | 2.2k pull-up |
| 46 | DRV_EN | Drives the TMC6300 VIO/enable pins | 10k pulldown |
| 19/20 | USB_D− / USB_D+ | Native USB | 22 Ω series (optional), ESD |
| 45 | NC | | Leave floating or pulled **down** |
| EN | ESP_EN | Reset | 10k pull-up + 1 µF to GND, RST button |

---

## 3. Net-by-Net Connection Table (Source of Truth)

### 3.1 Power

| Net | Connections |
|---|---|
| VBAT_IN | J1 (XT30) +; Q1 PFET drain (body diode orientation for reverse protection) |
| VM | Q1 source; D2 TVS SMBJ12A cathode; C1 470 µF/16 V polymer; U4/U5/U6 VS; C (per driver) 10 µF + 100 nF at VS; D1 cathode (USB OR); U2 VIN |
| Q1 gate | R1 100k to GND; Z1 10 V zener to source |
| VBUS | J2 USB-C VBUS; D1 anode (SS14); C 10 µF |
| +3V3 | U2 VOUT through L1 2.2 µH; C 22 µF ×2; U1 3V3; all logic VDD |
| GND | Everything. Use a single solid plane, with a star point at C1 for motor return. |
| U2 TPS62162 | VIN=VM, EN=VM via 100k, VOS=+3V3, SW→L1, PG→NC, AGND/PGND=GND |

### 3.2 ESP32-S3-WROOM-1 (U1)

- **Power:** 3V3 to +3V3, with 22 µF + 100 nF at the pin. GND pins and the EPAD go to GND (via array).
- **Reset:** EN connects to R 10k to +3V3, C 1 µF to GND, and SW_RST to GND.
- **GPIO:** All other GPIO per §2.

### 3.3 TMC6300 ×3 (U4, U5, U6), shown for Motor *n*

| TMC6300 pin (name) | Net |
|---|---|
| VS (×2) | VM |
| VCP | 100 nF to VS |
| CPI / CPO | 22 nF flying cap between |
| VIO | DRV_EN (GPIO46). This pin is the standby input: VIO low = standby. |
| UH, UL, VH, VL, WH, WL | Mn_UH … Mn_WL |
| DIAG | Mn_DIAG → BAT54 anode, cathode → DRV_DIAG |
| U, V, W | Mn_U/V/W → motor connector J(3+n) |
| BRU, BRV, BRW (low-side source) | Mn_BR → 0.12 Ω 0805 shunt → GND |
| VDD/5VOUT (internal LDO output) | 1 µF to GND |
| GND / EP | GND |

> **VERIFY:** The TMC6300 (QFN20, 3×3 mm) pin numbers and the exact names of the LDO and bridge pins must be checked against the Trinam