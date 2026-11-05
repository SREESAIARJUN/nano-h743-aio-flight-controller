I don't have access to your repository, schematic, or `hwdef.dat`, or to what changed in earlier cycles, so I can't know this board's actual pin assignments, regulator ratings, or thermal test results. Inventing them would be dangerous: a wrong UART, ADC scale, or current rating in a flight controller README can destroy hardware or cause a flyaway.

The README below is complete and production-structured. Generic facts (STM32H743 specs, ArduPilot build and flash workflow) are filled in. Every **board-specific value is marked `⟨VERIFY⟩`** for you to replace from your schematic and test data. If you share your `hwdef.dat` and schematic notes, I can fill these in exactly.

---

````markdown
# Nano H743 AIO Flight Controller

![Status](https://img.shields.io/badge/status-⟨VERIFY:beta|stable⟩-blue)
![MCU](https://img.shields.io/badge/MCU-STM32H743-informational)
![Firmware](https://img.shields.io/badge/firmware-ArduPilot-green)
![License](https://img.shields.io/badge/license-⟨VERIFY⟩-lightgrey)

A compact all-in-one (AIO) flight controller built on the STM32H743. It has an integrated
⟨VERIFY: 4-in-1 ESC / no ESC⟩, OSD, barometer, and blackbox storage.
It is designed for ⟨VERIFY: 2.5–5"⟩ multirotors and small fixed-wing aircraft running ArduPilot.

> **Revision:** Hardware rev ⟨VERIFY⟩ · Document cycle 8 · Last updated ⟨VERIFY: YYYY-MM-DD⟩

---

## Table of Contents
1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Power & Analog Sensing](#power--analog-sensing)
7. [Thermal Specifications](#thermal-specifications)
8. [Building ArduPilot](#building-ardupilot)
9. [Flashing](#flashing)
10. [First-Time Setup](#first-time-setup)
11. [Troubleshooting](#troubleshooting)
12. [Repository Structure](#repository-structure)
13. [Contributing](#contributing)
14. [License](#license)

---

## Specifications

| Category | Specification |
|---|---|
| MCU | STM32H743VIT6, Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM |
| IMU | ⟨VERIFY: e.g. ICM-42688-P / BMI270⟩ (SPI) |
| Barometer | ⟨VERIFY: e.g. DPS310 / BMP280⟩ (I²C) |
| OSD | ⟨VERIFY: AT7456E (analog) / MSP DisplayPort only⟩ |
| Blackbox | ⟨VERIFY: microSD / 128 Mbit SPI flash⟩ |
| Input voltage | ⟨VERIFY: 2–6S (7.4–25.2 V)⟩ |
| BEC outputs | ⟨VERIFY: 5 V @ x A, 9/10 V @ x A (switchable?)⟩, 3.3 V @ ⟨VERIFY⟩ |
| Integrated ESC | ⟨VERIFY: 4× xx A continuous / xx A burst, BLHeli_S / AM32 / Bluejay⟩ |
| UARTs | ⟨VERIFY: count⟩ |
| PWM outputs | ⟨VERIFY: count⟩ (DShot capable on ⟨VERIFY⟩) |
| I²C | ⟨VERIFY⟩ (compass / airspeed / external sensors) |
| CAN | ⟨VERIFY: yes/no, transceiver part⟩ |
| USB | ⟨VERIFY: USB-C⟩, USB 2.0 Full Speed |
| Mounting | ⟨VERIFY: 20×20 mm, M2 / M3 grommets⟩ |
| Dimensions | ⟨VERIFY: L × W × H mm⟩ |
| Weight | ⟨VERIFY: g⟩ |

---

## Board Layout

```
          ┌──────────────────────────────────┐
          │  [USB-C]          [BOOT]  [LED]  │
          │                                  │
          │   M1 ○                    ○ M2   │
          │            ┌────────┐            │
          │            │ H743   │            │
          │            └────────┘            │
          │   M4 ○                    ○ M3   │
          │                                  │
          │  [VBAT+] [VBAT-]    [UART/RC pads]│
          └──────────────────────────────────┘
          ⟨VERIFY: replace with real top/bottom render in docs/img/⟩
```

Photos:
- `docs/img/top.png`: ⟨VERIFY: add⟩
- `docs/img/bottom.png`: ⟨VERIFY: add⟩

---

## Pinout

> ⚠️ All MCU pin assignments must match `hwdef/NanoH743AIO/hwdef.dat`.
> If this table and the hwdef disagree, **the hwdef is authoritative**.

| Pad label | Function | MCU pin | Voltage level | Notes |
|---|---|---|---|---|
| 5V | BEC output | n/a | 5 V | ⟨VERIFY⟩ A max |
| 9V / 10V | VTX BEC | n/a | ⟨VERIFY⟩ | ⟨VERIFY: switchable via GPIO Pxx?⟩ |
| 3V3 | LDO output | n/a | 3.3 V | Low current, sensors only |
| GND | Ground | n/a | n/a | n/a |
| T1 / R1 | UART1 TX/RX | ⟨VERIFY: PA9 / PA10⟩ | 3.3 V | ⟨VERIFY: role⟩ |
| T2 / R2 | UART2 TX/RX | ⟨VERIFY⟩ | 3.3 V | |
| T3 / R3 | UART3 TX/RX | ⟨VERIFY⟩ | 3.3 V | |
| T4 / R4 | UART4 TX/RX | ⟨VERIFY⟩ | 3.3 V | |
| SDA / SCL | I²C | ⟨VERIFY⟩ | 3.3 V | Onboard pull-ups: ⟨VERIFY⟩ |
| LED | WS2812 data | ⟨VERIFY⟩ | 3.3 V | |
| BZ+ / BZ− | Buzzer | ⟨VERIFY⟩ | 5 V | |
| VTX | Analog video out | n/a | n/a | |
| CAM | Analog video in | n/a | n/a | |
| CURR | Current sense | ⟨VERIFY⟩ | 0–3.3 V | ADC input |
| RSSI | Analog RSSI | ⟨VERIFY⟩ | 0–3.3 V | |

**All signal pads are 3.3 V logic.** Check each pin's five-volt tolerance in the
STM32H743 datasheet before connecting any 5 V signal. Never apply 5 V to analog (ADC) pins.

---

## UART Mapping

ArduPilot `SERIALn` → physical UART (must match `SERIAL_ORDER` in hwdef):

| ArduPilot | Hardware | Default protocol | Default baud | Pads |
|---|---|---|---|---|
| SERIAL0 | USB (OTG1) | MAVLink2 | 115200 | USB-C |
| SERIAL1 | ⟨VERIFY: USART1⟩ | MAVLink2 (telem) | 57600 | T1/R1 |
| SERIAL2 | ⟨VERIFY⟩ | ⟨VERIFY: DisplayPort / VTX⟩ | ⟨VERIFY⟩ | |
| SERIAL3 | ⟨VERIFY⟩ | GPS | 230400 | |
| SERIAL4 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | | |
| SERIAL6 | ⟨VERIFY⟩ | RCIN (CRSF / ELRS) | 420000 | |
| SERIAL7 | ⟨VERIFY⟩ | ESC telemetry | 115200 | |

Example: ELRS receiver on SERIAL6:
```
SERIAL6_PROTOCOL = 23   # RCIN
RSSI_TYPE        = 3    # Receiver-reported
```

---

## PWM / Motor Outputs

| Output | MCU pin | Timer | DShot | BDShot | Group |
|---|---|---|---|---|---|
| M1 | ⟨VERIFY⟩ | ⟨VERIFY: TIMx_CHy⟩ | ✓ | ⟨VERIFY⟩ | 1 |
| M2 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | ✓ | ⟨VERIFY⟩ | 1 |
| M3 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | ✓ | ⟨VERIFY⟩ | 2 |
| M4 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | ✓ | ⟨VERIFY⟩ | 2 |

Outputs in the same group share a timer, so they **must use the same protocol and rate**.

Recommended for the integrated ESC:
```
MOT_PWM_TYPE      = 6     # DShot600
SERVO_BLH_AUTO    = 1
SERVO_BLH_BDMASK  = 15    # Bidirectional DShot on M1–M4 (if ESC firmware supports it)
INS_HNTCH_MODE    = 3     # ESC-telemetry RPM harmonic notch
```

---

## Power & Analog Sensing

| Parameter | Value |
|---|---|
| `BATT_MONITOR` | 4 (Analog voltage + current) |
| `BATT_VOLT_PIN` | ⟨VERIFY⟩ |
| `BATT_CURR_PIN` | ⟨VERIFY⟩ |
| `BATT_VOLT_MULT` | ⟨VERIFY: from divider ratio, e.g. 11.0⟩ |
| `BATT_AMP_PERVLT` | ⟨VERIFY: from shunt/amp calibration⟩ |
| `BATT_AMP_OFFSET` | ⟨VERIFY⟩ |

Calibrate against a reference meter. Divider tolerances make the default values only approximate.

---

## Thermal Specifications

### Silicon limits (from manufacturer datasheets)

| Component | Parameter | Limit | Source |
|---|---|---|---|
| STM32H743 | Max junction temperature (Tj) | 125 °C (⟨VERIFY: temp-range suffix of fitted part⟩) | ST DS12110 |
| STM32H743 | Ambient operating range | ⟨VERIFY: −40 to +85 °C or +105 °C per suffix⟩ | ST DS12110 |
| IMU | Operating range | ⟨VERIFY from IMU datasheet⟩ | |
| ESC MOSFETs | Max Tj | ⟨VERIFY: typ. 150–175 °C⟩ | ⟨VERIFY⟩ |
| BEC regulator | Thermal shutdown | ⟨VERIFY⟩ | ⟨VERIFY⟩ |

### Measured board performance

> Replace this table with **your own measured data**. Record the test conditions
> (ambient temperature, airflow, load, duration, sensor placement) so results are reproducible.

| Condition | MCU temp (`SCALED_PRESSURE`/`POWR`) | ESC FET hotspot | BEC hotspot |
|---|---|---|---|
| Bench idle, USB only, 25 °C, no airflow | ⟨VERIFY⟩ | n/a | ⟨VERIFY⟩ |
| Bench, VBAT 4S, VTX 800 mW, no airflow, 10 min | ⟨VERIFY⟩ | n/a | ⟨VERIFY⟩ |
| Static thrust, ⟨VERIFY⟩ A/motor continuous, 60 s | ⟨VERIFY⟩ | ⟨VERIFY⟩ | ⟨VERIFY⟩ |
| Typical flight, 3–5 min | ⟨VERIFY⟩ | ⟨VERIFY⟩ | ⟨VERIFY⟩ |

### Thermal guidance
- **Do not run high-power VTX or ESC loads on the bench without airflow.** AIO boards rely on prop wash for cooling.
- Burst current ratings assume ⟨VERIFY: ≤ 10 s⟩ with airflow. Continuous ratings assume ⟨VERIFY⟩.
- IMU heating: ⟨VERIFY: if the board has an IMU heater, document `BRD_HEAT_TARG`. Otherwise state "not supported"⟩.
- Monitor the MCU temperature in logs. Investigate if it exceeds ⟨VERIFY: e.g. 85 °C⟩ in flight.
- Conformal coating ⟨VERIFY: yes/no⟩. Coating affects thermal dissipation only marginally.

---

## Building ArduPilot

### 1. Prerequisites (Ubuntu 22.04 / 24.04)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

On Windows, use WSL2. On macOS, use `Tools/environment_install/install-prereqs-mac.sh`.

### 2. Add the board definition

If the board is not yet upstream, copy the hwdef from this repo:

```bash
cp -r <this-repo>/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

The directory must contain:
- `hwdef.dat`: main firmware pin and peripheral definition
- `hwdef-bl.dat`: bootloader definition
- `README.md`: upstream board docs (optional but required for upstreaming)
- `defaults.parm`: default parameters (optional)

Use a unique `APJ_BOARD_ID`. Request an official ID via a PR to
`Tools/AP_Bootloader/board_types.txt`. Do not reuse another board's ID.

### 3. Build the bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

### 4. Build the firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # multirotor
# ./waf plane       # fixed wing
# ./waf rover
```

Outputs in `build/NanoH743AIO/bin/`:

| File | Use |
|---|---|
| `arducopter.apj` | Upload via Mission Planner / QGC / `uploader.py` (bootloader already present