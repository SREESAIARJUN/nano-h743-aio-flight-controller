I don't have access to the `nano-h743-aio-flight-controller` repository, its schematics, or the earlier "cycles," so I can't know its actual pin assignments, regulator ratings, or thermal test results. Inventing those would be risky: a wrong UART, timer group, or current rating in a flight controller README can cost hardware or a vehicle.

The README below is complete in structure. Everything generic to STM32H743 and ArduPilot (build system, DFU flashing, bootloader, parameter mapping) is filled in. Board-specific values are marked **`⟪TBD⟫`** so you can fill them from your `hwdef.dat` and schematic. If you paste those files, I can populate every table.

---

````markdown
# Nano H743 AIO Flight Controller

> Compact all-in-one STM32H743 flight controller with integrated ESC, OSD, and BEC, running ArduPilot.

![Board top view](docs/img/top.png) ![Board bottom view](docs/img/bottom.png)

| | |
|---|---|
| **Status** | ⟪TBD: Prototype / Production rev X.Y⟫ |
| **Firmware** | ArduPilot (Copter / Plane / Rover / Sub-compatible targets) |
| **ArduPilot board name** | `NanoH743AIO` ⟪confirm against hwdef directory name⟫ |
| **License** | Hardware: ⟪TBD, e.g. CERN-OHL-S-2.0⟫ · Firmware: GPLv3 (ArduPilot) |

---

## Table of Contents

1. [Specifications](#specifications)
2. [Pinout](#pinout)
3. [UART / Serial Mapping](#uart--serial-mapping)
4. [PWM / Motor Output Mapping](#pwm--motor-output-mapping)
5. [Power, Battery Monitoring & BEC](#power-battery-monitoring--bec)
6. [Thermal Specifications](#thermal-specifications)
7. [Building ArduPilot](#building-arduplot)
8. [Flashing](#flashing)
9. [First-Time Setup](#first-time-setup)
10. [Troubleshooting](#troubleshooting)
11. [Repository Layout](#repository-layout)
12. [Contributing](#contributing)
13. [Safety Notice](#safety-notice)

---

## Specifications

### Processor & Memory
| Item | Value |
|---|---|
| MCU | STM32H743⟪VIT6 / VIH6 — confirm package & temp grade⟫ |
| Core | ARM Cortex-M7 @ 480 MHz, double-precision FPU |
| Flash (internal) | 2 MB |
| RAM | 1 MB |
| Data logging | ⟪TBD: microSD / onboard SPI flash, part number & size⟫ |

### Sensors
| Function | Part | Bus |
|---|---|---|
| IMU 1 | ⟪TBD⟫ | SPI⟪n⟫ |
| IMU 2 (if fitted) | ⟪TBD⟫ | SPI⟪n⟫ |
| Barometer | ⟪TBD⟫ | I2C⟪n⟫ / SPI⟪n⟫ |
| OSD | ⟪TBD, e.g. AT7456E⟫ | SPI⟪n⟫ |
| Magnetometer | External only, via I2C⟪n⟫ | — |

### Integrated ESC
| Item | Value |
|---|---|
| Channels | ⟪4⟫ |
| Continuous current per channel | ⟪TBD⟫ A *(see [Thermal Specifications](#thermal-specifications))* |
| Burst current per channel | ⟪TBD⟫ A for ⟪TBD⟫ s |
| ESC firmware | ⟪BLHeli_S / Bluejay / AM32 — TBD⟫ |
| Protocol | DShot⟪150/300/600⟫, bidirectional DShot ⟪supported? Y/N⟫ |
| Input voltage | ⟪TBD⟫S LiPo (⟪min⟫ to ⟪max⟫ V) |

### Physical
| Item | Value |
|---|---|
| Mounting pattern | ⟪25.5 × 25.5 mm / 20 × 20 mm, M2⟫ |
| Dimensions | ⟪TBD⟫ mm |
| Weight | ⟪TBD⟫ g |

---

## Pinout

> Source of truth: `hwdef/NanoH743AIO/hwdef.dat` and `hardware/schematic.pdf`.
> If this table disagrees with `hwdef.dat`, **`hwdef.dat` wins**. Please open an issue.

### Solder Pad Diagram

```
          ┌───────────────────────────────────────┐
   M1 ●   │  ⟪insert pad diagram or link image⟫   │   ● M2
          │                                       │
          │   [USB-C]                [BOOT]       │
          │                                       │
   M4 ●   │                                       │   ● M3
          └───────────────────────────────────────┘
               BAT+  BAT-      (main power pads)
```

![Detailed pinout](docs/img/pinout.png)

### Pad Reference
| Pad label | MCU pin | Function | Voltage | Notes |
|---|---|---|---|---|
| 5V | — | BEC 5 V out | 5.0 V | ⟪max current⟫ |
| 9V / 10V | — | VTX BEC out | ⟪TBD⟫ | ⟪switchable via GPIO? Y/N⟫ |
| 3V3 | — | LDO out | 3.3 V | Low-current peripherals only |
| GND | — | Ground | — | |
| TX1 / RX1 | ⟪P??⟫ / ⟪P??⟫ | USART1 | 3.3 V | |
| TX2 / RX2 | ⟪P??⟫ / ⟪P??⟫ | USART2 | 3.3 V | |
| … | … | … | … | ⟪complete for all UARTs⟫ |
| SCL / SDA | ⟪P??⟫ / ⟪P??⟫ | I2C⟪n⟫ | 3.3 V | External compass / airspeed |
| LED | ⟪P??⟫ | WS2812 data | 3.3 V | |
| BZ+ / BZ- | ⟪P??⟫ | Buzzer | 5 V | |
| CURR | ⟪P??⟫ | Current sense ADC | 0–3.3 V | |
| VBAT | ⟪P??⟫ | Battery voltage ADC | divided | |
| RSSI | ⟪P??⟫ | Analog RSSI | 0–3.3 V | |
| CAM | ⟪P??⟫ | Camera video in | — | |
| VTX | ⟪P??⟫ | Video out | — | |

---

## UART / Serial Mapping

ArduPilot `SERIALn` order is defined by the `SERIAL_ORDER` line in `hwdef.dat`.

| ArduPilot port | Hardware | Default protocol | Default `SERIALn_PROTOCOL` | DMA | Notes |
|---|---|---|---|---|---|
| SERIAL0 | USB (OTG1) | MAVLink2 | 2 | — | |
| SERIAL1 | ⟪USARTx⟫ | MAVLink2 | 2 | ⟪Y/N⟫ | Telemetry |
| SERIAL2 | ⟪USARTx⟫ | ⟪TBD⟫ | ⟪TBD⟫ | ⟪Y/N⟫ | |
| SERIAL3 | ⟪USARTx⟫ | GPS | 5 | ⟪Y/N⟫ | |
| SERIAL4 | ⟪USARTx⟫ | ⟪TBD⟫ | ⟪TBD⟫ | ⟪Y/N⟫ | |
| SERIAL5 | ⟪USARTx⟫ | ⟪TBD⟫ | ⟪TBD⟫ | ⟪Y/N⟫ | |
| SERIAL6 | ⟪USARTx⟫ | RC input | 23 | ⟪Y/N⟫ | CRSF / ELRS / SBUS |
| SERIAL7 | ⟪USARTx⟫ | ESC telemetry | 16 | ⟪Y/N⟫ | |

**Common protocol values:** `1/2` MAVLink1/2 · `5` GPS · `10` FrSky SPort · `16` ESC Telemetry · `23` RCIN · `28` Scripting · `33` DJI FPV / MSP DisplayPort (`42`).

**RC receiver setup (CRSF / ELRS example):**
```
SERIAL6_PROTOCOL = 23
SERIAL6_BAUD     = 115     # 420000 handled automatically for CRSF
RSSI_TYPE        = 3
```

---

## PWM / Motor Output Mapping

On STM32, outputs that share a hardware timer **must use the same protocol and rate**. For example, you cannot mix DShot and PWM within one group.

| Output | MCU pin | Timer / channel | Group | Default function | Bidir DShot |
|---|---|---|---|---|---|
| M1 / SERVO1 | ⟪P??⟫ | TIM⟪x⟫_CH⟪y⟫ | 1 | Motor 1 | ⟪Y/N⟫ |
| M2 / SERVO2 | ⟪P??⟫ | TIM⟪x⟫_CH⟪y⟫ | 1 | Motor 2 | ⟪Y/N⟫ |
| M3 / SERVO3 | ⟪P??⟫ | TIM⟪x⟫_CH⟪y⟫ | 2 | Motor 3 | ⟪Y/N⟫ |
| M4 / SERVO4 | ⟪P??⟫ | TIM⟪x⟫_CH⟪y⟫ | 2 | Motor 4 | ⟪Y/N⟫ |
| SERVO5 (pad) | ⟪P??⟫ | TIM⟪x⟫_CH⟪y⟫ | 3 | ⟪TBD⟫ | — |
| LED | ⟪P??⟫ | TIM⟪x⟫_CH⟪y⟫ | ⟪n⟫ | NeoPixel (`SERVOn_FUNCTION=120`) | — |

**Recommended DShot configuration (Copter):**
```
MOT_PWM_TYPE      = 6      # DShot600 (5 = DShot300)
SERVO_BLH_AUTO    = 1      # BLHeli passthrough
SERVO_BLH_BDMASK  = 15     # Bidirectional DShot on outputs 1–4 (if supported)
INS_HNTCH_MODE    = 3      # ESC-telemetry RPM-based harmonic notch
```

---

## Power, Battery Monitoring & BEC

| Rail | Source | Voltage | Max continuous current |
|---|---|---|---|
| Main input | Battery pads | ⟪min⟫–⟪max⟫ V (⟪n⟫S) | Limited by ESC rating |
| 5 V BEC | ⟪regulator part⟫ | 5.0 V | ⟪TBD⟫ A |
| VTX BEC | ⟪regulator part⟫ | ⟪TBD⟫ V | ⟪TBD⟫ A |
| 3.3 V | ⟪LDO part⟫ | 3.3 V | ⟪TBD⟫ mA |

**Default battery monitor parameters** (set in `hwdef.dat`; **calibrate per board**):
```
BATT_MONITOR     = 4          # Analog voltage + current
BATT_VOLT_PIN    = ⟪TBD⟫
BATT_CURR_PIN    = ⟪TBD⟫
BATT_VOLT_MULT   = ⟪TBD⟫
BATT_AMP_PERVLT  = ⟪TBD⟫
BATT_AMP_OFFSET  = ⟪TBD⟫
```

To calibrate, measure pack voltage with a multimeter, then adjust `BATT_VOLT_MULT` until the reported value matches. Calibrate current against a known load or a charged-mAh comparison.

---

## Thermal Specifications

> ⚠️ **All values below must come from measured test data.** Do not ship this section with estimates.
> Record your test method in `docs/thermal-test-report.md`.

### Component Limits
| Component | Rated range | Source |
|---|---|---|
| STM32H743 | ⟪-40 to +85 °C ambient (suffix 6) / -40 to +125 °C (suffix 3) — confirm part suffix⟫ | ST datasheet |
| IMU | ⟪TBD⟫ | Datasheet |
| ESC MOSFETs | T<sub>J</sub> max ⟪TBD⟫ °C | Datasheet |
| BEC regulator | T<sub>J</sub> max ⟪TBD⟫ °C, thermal shutdown ⟪TBD⟫ °C | Datasheet |
| Board operating range (validated) | ⟪TBD⟫ to ⟪TBD⟫ °C ambient | Test report |

### ESC Current vs. Cooling (measured)
| Condition | Continuous / channel | Burst / channel | Peak MOSFET temp | Test duration |
|---|---|---|---|---|
| Still air, 25 °C ambient | ⟪TBD⟫ A | ⟪TBD⟫ A / ⟪t⟫ s | ⟪TBD⟫ °C | ⟪TBD⟫ |
| Prop-wash airflow (≈⟪TBD⟫ m/s) | ⟪TBD⟫ A | ⟪TBD⟫ A / ⟪t⟫ s | ⟪TBD⟫ °C | ⟪TBD⟫ |
| Enclosed / ducted frame | ⟪TBD⟫ A | — | ⟪TBD⟫ °C | ⟪TBD⟫ |

### BEC Derating
| Ambient | 5 V max current | VTX rail max current |
|---|---|---|
| 25 °C | ⟪TBD⟫ | ⟪TBD⟫ |
| 50 °C | ⟪TBD⟫ | ⟪TBD⟫ |
| 70 °C | ⟪TBD⟫ | ⟪TBD⟫ |

### Thermal Design Guidance
- **Never run the VTX at high power on the bench without airflow.** On AIO boards, VTX and ESC heat can push the board above its limits within minutes.
- Keep IMU temperature stable to reduce drift. If supported, enable heater/temperature calibration (`BRD_HEAT_TARG`, `INS_TCALn_*`).
- Avoid conformal-coating the MOSFET thermal pads unless the coating has been validated for thermal conductivity.
- Monitor in flight with the `POWR` and `ESC` log messages and the OSD MCU temperature (`OSD1_*` items).

---

## Building ArduPilot

### 1. Prerequisites (Ubuntu 22.04 / 24.04 recommended)
```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile          # or log out and back in
```
For macOS, use `