# nano-h743-aio-flight-controller README

I don't have access to your repository, schematic, or `hwdef.dat`, so the pin assignments, current ratings, and thermal figures below are **placeholders based on typical STM32H743 AIO designs**. They are marked `⚠️ VERIFY` or `TBD`. Replace them with values from your schematic and test data before publishing. Shipping a flight controller README with wrong pinouts can destroy hardware.

---

```markdown
# Nano H743 AIO Flight Controller

![Revision](https://img.shields.io/badge/hardware-Cycle%2029-blue)
![MCU](https://img.shields.io/badge/MCU-STM32H743-informational)
![Firmware](https://img.shields.io/badge/firmware-ArduPilot-success)
![License](https://img.shields.io/badge/license-TBD-lightgrey)

A compact all-in-one (AIO) flight controller built on the STM32H743 (480 MHz Cortex-M7),
designed for small multirotors, fixed-wing, and VTOL platforms running ArduPilot.

> **Hardware revision:** Cycle 29
> **Status:** ⚠️ VERIFY (e.g. Engineering Validation / Production)

---

## Table of Contents

1. [Features](#features)
2. [Specifications](#specifications)
3. [Board Layout](#board-layout)
4. [Pinout](#pinout)
5. [UART Mapping](#uart-mapping)
6. [PWM / Motor Outputs](#pwm--motor-outputs)
7. [Power](#power)
8. [Thermal Specifications](#thermal-specifications)
9. [Building ArduPilot](#building-ardupilot)
10. [Flashing](#flashing)
11. [First-Time Setup](#first-time-setup)
12. [Troubleshooting](#troubleshooting)
13. [Revision History](#revision-history)
14. [Safety](#safety)
15. [License](#license)

---

## Features

- STM32H743VIT6 MCU, 480 MHz, 2 MB flash, 1 MB RAM
- Dual IMU (⚠️ VERIFY: e.g. ICM-42688-P + BMI270)
- Barometer (⚠️ VERIFY: e.g. DPS310 / BMP280)
- Onboard OSD (AT7456E) (⚠️ VERIFY)
- MicroSD card slot for logging
- Integrated 4-in-1 ESC (AIO) (⚠️ VERIFY current rating)
- 8× PWM / DShot outputs with bidirectional DShot support
- 6–7 UARTs, I²C, CAN (⚠️ VERIFY)
- Onboard 5 V and 10 V BECs
- Current and voltage sensing

---

## Specifications

| Parameter              | Value                              |
|------------------------|------------------------------------|
| MCU                    | STM32H743VIT6                      |
| Input voltage          | 2S–6S LiPo (7.4–25.2 V) ⚠️ VERIFY  |
| BEC 5 V                | 5 V @ 2 A ⚠️ VERIFY                |
| BEC 10 V (VTX)         | 10 V @ 2 A ⚠️ VERIFY               |
| ESC continuous current | TBD A per channel                  |
| ESC burst current      | TBD A (≤10 s)                      |
| Mounting pattern       | 25.5 × 25.5 mm, M2 ⚠️ VERIFY       |
| Dimensions             | TBD × TBD × TBD mm                 |
| Weight                 | TBD g                              |
| Firmware               | ArduPilot (target: `NanoH743AIO`)  |

---

## Board Layout

<!-- Replace with actual rendered images from your PCB tool -->
![Top view](docs/images/top.png)
![Bottom view](docs/images/bottom.png)

---

## Pinout

> ⚠️ **All pin assignments below are placeholders.** The authoritative source is
> `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`. Keep this table in sync with it.

### MCU Peripheral Map

| Function        | MCU Pin | Peripheral    | Notes                    |
|-----------------|---------|---------------|--------------------------|
| IMU1 SCK        | PA5     | SPI1          | ⚠️ VERIFY                |
| IMU1 MISO       | PA6     | SPI1          |                          |
| IMU1 MOSI       | PD7     | SPI1          |                          |
| IMU1 CS         | PC15    | GPIO          |                          |
| IMU2 CS         | PE11    | GPIO          | Second IMU on SPI4       |
| OSD CS          | PB12    | SPI2          | AT7456E                  |
| SD card         | PC8–PC12, PD2 | SDMMC1  | 4-bit mode               |
| Baro SDA        | PB7     | I2C1          |                          |
| Baro SCL        | PB6     | I2C1          |                          |
| VBAT sense      | PC0     | ADC1          | Divider ratio: TBD       |
| Current sense   | PC1     | ADC1          | Scale: TBD A/V           |
| RSSI (analog)   | PC5     | ADC1          | Optional                 |
| LED (status)    | PE3     | GPIO          |                          |
| Buzzer          | PA15    | TIM2          | Active-high              |

### External Pads / Connectors

| Pad      | Function                    | Voltage level |
|----------|-----------------------------|---------------|
| `BAT+`   | Battery input               | 2S–6S         |
| `GND`    | Ground                      | —             |
| `5V`     | 5 V BEC output              | 5 V           |
| `10V`    | 10 V BEC (VTX)              | 10 V          |
| `4V5`    | USB-powered rail (RX/GPS)   | 4.5 V         |
| `SDA/SCL`| I²C (compass)               | 3.3 V         |
| `CANH/L` | DroneCAN                    | Differential  |
| `LED`    | WS2812 data                 | 3.3 V         |
| `BZ+/BZ-`| Buzzer                      | 5 V           |

---

## UART Mapping

| ArduPilot | MCU UART | TX   | RX   | Default protocol      | `SERIALx_PROTOCOL` |
|-----------|----------|------|------|-----------------------|--------------------|
| SERIAL0   | USB      | —    | —    | MAVLink2              | 2                  |
| SERIAL1   | USART1   | PA9  | PA10 | MAVLink2 (telem)      | 2                  |
| SERIAL2   | USART2   | PD5  | PD6  | RC input              | 23                 |
| SERIAL3   | USART3   | PD8  | PD9  | GPS                   | 5                  |
| SERIAL4   | UART4    | PB9  | PB8  | GPS2 / spare          | -1                 |
| SERIAL5   | UART5    | PB13 | PB5  | DisplayPort / VTX     | 42                 |
| SERIAL6   | USART6   | PC6  | PC7  | ESC telemetry         | 16                 |
| SERIAL7   | UART7    | PE8  | PE7  | Spare                 | -1                 |

⚠️ VERIFY all pins against `hwdef.dat`.

---

## PWM / Motor Outputs

| Output | MCU Pin | Timer    | DShot | BiDir DShot | Group |
|--------|---------|----------|-------|-------------|-------|
| M1     | PB0     | TIM3_CH3 | ✅    | ✅          | 1     |
| M2     | PB1     | TIM3_CH4 | ✅    | ✅          | 1     |
| M3     | PA0     | TIM5_CH1 | ✅    | ✅          | 2     |
| M4     | PA1     | TIM5_CH2 | ✅    | ✅          | 2     |
| S5     | PA2     | TIM5_CH3 | ✅    | ❌          | 2     |
| S6     | PA3     | TIM5_CH4 | ✅    | ❌          | 2     |
| S7     | PD12    | TIM4_CH1 | ✅    | ❌          | 3     |
| S8     | PD13    | TIM4_CH2 | ✅    | ❌          | 3     |

> Outputs in the same group **must use the same protocol and rate** (e.g. you cannot mix
> PWM and DShot within group 2).

---

## Power

### Voltage and Current Monitoring Defaults

| Parameter         | Default  |
|-------------------|----------|
| `BATT_MONITOR`    | 4        |
| `BATT_VOLT_PIN`   | TBD      |
| `BATT_CURR_PIN`   | TBD      |
| `BATT_VOLT_MULT`  | TBD      |
| `BATT_AMP_PERVLT` | TBD      |

Calibrate `BATT_AMP_PERVLT` against a known load before relying on mAh-consumed failsafes.

### Power Rules

- Never power the board from USB with ESC outputs connected to spinning props.
- Use a low-ESR capacitor (≥ 35 V, ≥ 470 µF for 6S) across `BAT+`/`GND` as close to the pads as possible.

---

## Thermal Specifications

> ⚠️ Values marked TBD must come from your own thermal testing (thermocouple or IR
> camera, still air and in-flight airflow). Component limits below are from datasheets.

### Component Limits

| Component            | Absolute limit                         | Source                     |
|----------------------|----------------------------------------|----------------------------|
| STM32H743 (–6 grade) | T<sub>J</sub> max 125 °C; T<sub>A</sub> –40 to +85 °C | ST datasheet, verify suffix |
| ESC MOSFETs          | T<sub>J</sub> max TBD °C (typ. 150–175 °C) | MOSFET datasheet       |
| IMU                  | Operating –40 to +85 °C                | IMU datasheet              |
| BEC regulators       | Thermal shutdown at TBD °C             | Regulator datasheet        |

### Measured Performance (Cycle 29)

| Condition                         | MCU temp | ESC FET temp | BEC temp | Notes        |
|-----------------------------------|----------|--------------|----------|--------------|
| Idle, USB powered, 25 °C ambient  | TBD      | —            | TBD      | Still air    |
| Bench, 6S, motors disarmed        | TBD      | TBD          | TBD      | Still air    |
| Sustained TBD A/ch, with airflow  | TBD      | TBD          | TBD      | TBD m/s      |
| Burst TBD A/ch for 10 s           | TBD      | TBD          | TBD      |              |

### Design Guidance

- **Airflow is required** at sustained ESC currents above TBD A per channel.
- Do not enclose the board in a sealed canopy without ventilation.
- The IMU heater (if enabled via `BRD_HEAT_TARG`) adds load. Disable it in hot climates.
- Monitor MCU temperature in flight logs (`POWR.MTemp`) and ESC temperature via ESC telemetry.

---

## Building ArduPilot

### Prerequisites

Ubuntu 22.04 / 24.04 (or WSL2):

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Board Definition

Until the board is merged upstream, copy the hwdef from this repo:

```bash
cp -r <this-repo>/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

### Build Firmware

```bash
# Configure for this board
./waf configure --board NanoH743AIO

# Build the vehicle you need
./waf copter      # ArduCopter
./waf plane       # ArduPlane (incl. QuadPlane VTOL)
./waf rover       # Rover
```

Output files:

```
build/NanoH743AIO/bin/arducopter.apj             # Upload via GCS
build/NanoH743AIO/bin/arducopter_with_bl.hex     # Full image incl. bootloader
```

### Build the Bootloader

```bash
./waf configure --board NanoH743AIO --bootloader
./waf bootloader
# Output: build/NanoH743AIO/bin/AP_Bootloader.bin
```

### Clean Rebuild

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

---

## Flashing

### Method 1: First Flash via DFU (factory / blank board)

1. Hold the **BOOT** button and connect USB.
2. Confirm the board enumerates in DFU mode:

   ```bash
   dfu-util -l
   # Expect: [0483:df11] ... "@Internal Flash"
   ```

3. Flash the combined image with **STM32CubeProgrammer** (recommended for `.hex`):

   ```bash
   STM32_Programmer_CLI -c port=USB1 -w arducopter_with_bl.hex -v -rst
   ```

   Or flash the bootloader binary with dfu-util:

   ```bash
   dfu-util -a 0 --dfuse-address 0x08000000:leave -D AP_Bootloader.bin
   ```

4. Unplug and reconnect USB **without** pressing BOOT.

> **Linux:** If you get permission errors, add a udev rule for `0483:df11` or run with `sudo`.
> **Windows:** Use STM32CubeProgrammer, or install the WinUSB driver via Zadig for dfu-util.

### Method 2: Firmware Updates (ArduPilot bootloader installed)

**Mission Planner:** Setup → Install Firmware → *Load custom firmware* → select `.apj`

**QGroundControl:** Vehicle Setup → Firmware → *Advanced* → *Custom firmware file*

**Command line:**

```bash
./waf copter --upload
# or
python3 Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
```

### Verify

After flashing, connect a GCS and confirm the firmware banner shows:

```
ArduCopter V4.x.x
ChibiOS: xxxxxxxx
NanoH743AIO xxxxxxxx xxxxxxxx xxxxxxxx
```

---

## First-Time Setup

1. Set **frame class/type** (`FRAME_CLASS`, `FRAME_TYPE`).
2. Run **accelerometer** and **compass** calibration.
3. Configure **RC input** (`SERIAL2_PROTOCOL=23`, `RSSI_TYPE` if needed).
4. Set the **motor protocol**:

   ```
   MOT_PWM_TYPE = 6          # DShot600
   SERVO_BLH_AUTO = 1
   SERVO_BLH_BDMASK = 15     # Bidirectional DShot on M1–M4
   ```

5. Calibrate **battery voltage and current**.
6. Verify **motor order and direction** with props OFF (Mission Planner → Motor Test).
7. Configure **failsafes**: RC, battery, GCS.

---

## Troubleshooting

| Symptom                           | Likely cause / fix                                            |
|-----------------------------------|---------------------------------------------------------------|
| No USB device detected            | Charge-only cable; try a data cable                           |
| Stuck in DFU mode                 | BOOT pad bridged/stuck; reflash bootloader                    |
| "No IMU detected"                 | Wrong hwdef; check SPI CS pins                                |
| Bidirectional DShot not working   | Check `SERVO_BLH_BDMASK`; ESC firmware must support it        |
| Motors twitch on boot             | Mixed protocols within a timer group                          |
| Current reading wrong             | Recalibrate `BATT_AMP_PERVLT` and `B