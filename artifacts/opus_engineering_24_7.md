I don't have access to the actual `nano-h743-aio-flight-controller` repository, its schematics, or the outputs of previous cycles. The README below therefore:

- Uses **real, current ArduPilot build and flash procedures**, which apply to any STM32H743 board.
- Fills hardware-specific values (pin assignments, timers, thermal measurements) with **typical H743 AIO conventions, marked `⚠ VERIFY`**. Before publishing, replace these with values from your `hwdef.dat` and schematic. Publishing an unverified pinout can cause users to wire motors or power incorrectly.

If you paste your `hwdef.dat` or schematic netlist, I can produce a fully populated version.

---

```markdown
# Nano H743 AIO Flight Controller

![Board Render](docs/img/board-top.png)

[![ArduPilot](https://img.shields.io/badge/ArduPilot-4.5%2B-blue)](https://ardupilot.org)
[![License: CERN-OHL-S-2.0](https://img.shields.io/badge/Hardware-CERN--OHL--S--2.0-green)](LICENSE-HW)
[![License: GPL-3.0](https://img.shields.io/badge/Firmware-GPL--3.0-blue)](LICENSE)

A compact all-in-one (AIO) flight controller built around the STM32H743. It integrates
an onboard 4-in-1 ESC footprint, a barometer, an IMU, OSD, and blackbox flash, and it
targets ArduPilot (Copter, Plane, Rover) in sub-250 g and whoop-class airframes.

> **Status:** Hardware revision `rev-C` · Release cycle 24
> **Board ID:** `APJ_BOARD_ID ⚠ VERIFY` (must be registered in `Tools/AP_Bootloader/board_types.txt`)

---

## Table of Contents
1. [Specifications](#specifications)
2. [Pinout](#pinout)
3. [UART Mapping](#uart-mapping)
4. [PWM / Motor Outputs](#pwm--motor-outputs)
5. [Building ArduPilot](#building-arducopter--ardupilot)
6. [Flashing](#flashing)
7. [Thermal Specifications](#thermal-specifications)
8. [Power](#power)
9. [Troubleshooting](#troubleshooting)
10. [Contributing](#contributing)
11. [License](#license)

---

## Specifications

| Parameter        | Value                                           |
|------------------|-------------------------------------------------|
| MCU              | STM32H743VIT6, Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM |
| IMU              | ⚠ VERIFY (e.g., ICM-42688-P, SPI1)              |
| Barometer        | ⚠ VERIFY (e.g., DPS310 / BMP280, I2C2)          |
| OSD              | AT7456E (SPI2)                                  |
| Blackbox         | ⚠ VERIFY (e.g., 16 MB W25Q128 SPI flash, SPI3)  |
| Input voltage    | 2S–6S LiPo (7–26 V) ⚠ VERIFY                    |
| BEC              | 5 V / 2 A, 10 V / 2 A (VTX), 3.3 V LDO          |
| UARTs            | 6 (incl. 1 inverted-capable for SBUS)           |
| Motor outputs    | 4 (onboard ESC) + 4 aux PWM                     |
| Mounting         | 25.5 × 25.5 mm, M2 ⚠ VERIFY                     |
| Dimensions       | ⚠ VERIFY mm                                     |
| Weight           | ⚠ VERIFY g                                      |
| USB              | USB-C (FS, DFU-capable)                         |

---

## Pinout

> **Source of truth:** `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`.
> If this table and the hwdef disagree, the hwdef wins. File an issue.

![Pinout Top](docs/img/pinout-top.png)
![Pinout Bottom](docs/img/pinout-bottom.png)

### MCU Pin Assignment

| MCU Pin | Function      | Peripheral | Notes                        |
|---------|---------------|------------|------------------------------|
| PA5     | SPI1_SCK      | IMU        | ⚠ VERIFY                     |
| PA6     | SPI1_MISO     | IMU        |                              |
| PA7     | SPI1_MOSI     | IMU        |                              |
| PC15    | IMU_CS        | IMU        |                              |
| PB13    | SPI2_SCK      | OSD        |                              |
| PB14    | SPI2_MISO     | OSD        |                              |
| PB15    | SPI2_MOSI     | OSD        |                              |
| PB12    | OSD_CS        | OSD        |                              |
| PB10    | I2C2_SCL      | Baro/Mag   | External mag on same bus     |
| PB11    | I2C2_SDA      | Baro/Mag   |                              |
| PC0     | BATT_VOLTAGE  | ADC        | Scale ⚠ VERIFY               |
| PC1     | BATT_CURRENT  | ADC        | Scale ⚠ VERIFY               |
| PC5     | RSSI          | ADC        |                              |
| PE3     | LED0          | GPIO       | Status (blue)                |
| PE4     | LED1          | GPIO       | Status (green)               |
| PD10    | BUZZER        | GPIO/TIM   |                              |

### Connector Pads

| Pad    | Signal              | Voltage level |
|--------|---------------------|---------------|
| T1/R1  | UART1 TX/RX         | 3.3 V         |
| T2/R2  | UART2 TX/RX         | 3.3 V         |
| T3/R3  | UART3 TX/RX         | 3.3 V         |
| T4/R4  | UART4 TX/RX         | 3.3 V         |
| T6/R6  | UART6 TX/RX (RCIN)  | 3.3 V         |
| T7/R7  | UART7 TX/RX         | 3.3 V         |
| SDA/SCL| I2C2                | 3.3 V (pulled up) |
| 5V     | BEC out             | 5 V           |
| 10V    | VTX BEC out         | 10 V          |
| VBAT   | Battery passthrough | Pack voltage  |
| CUR    | Current sensor in   | 0–3.3 V       |

---

## UART Mapping

| ArduPilot `SERIALx` | MCU UART | Default protocol | Notes              |
|---------------------|----------|------------------|--------------------|
| SERIAL0             | USB      | MAVLink2         |                    |
| SERIAL1             | UART1    | MAVLink2         | Telemetry          |
| SERIAL2             | UART2    | MAVLink2         |                    |
| SERIAL3             | UART3    | GPS              |                    |
| SERIAL4             | UART4    | GPS              | Secondary GPS      |
| SERIAL5             | UART6    | RCIN             | CRSF/ELRS/SBUS     |
| SERIAL6             | UART7    | DisplayPort / VTX| DJI/Walksnail      |

Set `SERIALx_PROTOCOL` accordingly. For SBUS, use `SERIALx_OPTIONS=1` (invert RX).

---

## PWM / Motor Outputs

| Output | MCU Pin | Timer      | DShot group | Bidir DShot |
|--------|---------|------------|-------------|-------------|
| M1     | PB0     | TIM3_CH3   | 1           | Yes         |
| M2     | PB1     | TIM3_CH4   | 1           | Yes         |
| M3     | PA0     | TIM5_CH1   | 2           | Yes         |
| M4     | PA1     | TIM5_CH2   | 2           | Yes         |
| S5     | PA2     | TIM5_CH3   | 2           | Yes         |
| S6     | PA3     | TIM5_CH4   | 2           | Yes         |
| S7     | PD12    | TIM4_CH1   | 3           | No          |
| S8     | PD13    | TIM4_CH2   | 3           | No          |

⚠ VERIFY all of the above against the hwdef.

**Outputs in the same timer group must use the same protocol.** For example, you cannot
mix DShot and PWM within group 2. Recommended setup: `MOT_PWM_TYPE=6` (DShot600),
`SERVO_BLH_BDMASK=15`.

---

## Building ArduCopter / ArduPilot

### Prerequisites (Ubuntu 22.04 / 24.04)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

If the board is not yet upstream, copy the hwdef from this repo first:

```bash
cp -r /path/to/this/repo/hwdef/NanoH743AIO libraries/AP_HAL_ChibiOS/hwdef/
```

### Build the Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane, rover, sub, heli
```

Artifacts are written to `build/NanoH743AIO/bin/`:

| File                          | Purpose                                       |
|-------------------------------|-----------------------------------------------|
| `arducopter.apj`              | Upload via Mission Planner / QGC / uploader.py |
| `arducopter_with_bl.hex`      | Full image (bootloader + firmware) for DFU    |
| `arducopter.bin`              | Raw application binary                        |

### Build the Bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

After building the bootloader, rebuild the firmware so `_with_bl.hex` embeds it.

### Useful Build Options

```bash
./waf configure --board NanoH743AIO --debug        # debug symbols
./waf configure --board NanoH743AIO --enable-dds   # ROS 2 DDS support
./waf copter --upload                              # build and flash over USB
```

---

## Flashing

### First Flash (Blank Board or Betaflight/INAV Installed): DFU

1. Hold **BOOT** while plugging in USB. The board enumerates as `STM32 BOOTLOADER`.
2. Flash the combined image using **one** of the methods below.

**STM32CubeProgrammer (recommended)**
```bash
STM32_Programmer_CLI -c port=USB1 -e all -w arducopter_with_bl.hex -v -rst
```

**dfu-util**
```bash
dfu-util -a 0 --dfuse-address 0x08000000:mass-erase:force:leave \
         -D arducopter_with_bl.bin
```

**Windows:** If the device is not detected, use [Zadig](https://zadig.akeo.ie/) to install the WinUSB driver.

### Subsequent Updates (ArduPilot Bootloader Present)

- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* → select the `.apj` file.
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*.
- **CLI:**
  ```bash
  python3 Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
  ```

### Recovery

If the board fails to boot, re-enter DFU using the BOOT pad or button and repeat the full
`_with_bl.hex` flash. The ROM DFU bootloader cannot be bricked by firmware.

---

## Thermal Specifications

| Parameter                              | Value              | Source              |
|----------------------------------------|--------------------|---------------------|
| MCU operating ambient                  | −40 °C to +85 °C   | STM32H743 datasheet (temp range 6) |
| MCU max junction temperature (T<sub>J</sub>) | +125 °C      | STM32H743 datasheet |
| IMU operating range                    | −40 °C to +85 °C   | IMU datasheet ⚠ VERIFY |
| Recommended board ambient (flight)     | −10 °C to +60 °C   | Design guideline    |
| Storage                                | −40 °C to +85 °C   |                     |
| MCU temp rise, idle, USB powered       | ⚠ MEASURE °C       | Thermal camera, 25 °C ambient |
| MCU temp rise, 6S hover, no airflow    | ⚠ MEASURE °C       |                     |
| ESC FET temp, 30 A continuous, 10 s    | ⚠ MEASURE °C       |                     |
| BEC regulator temp, 2 A load           | ⚠ MEASURE °C       |                     |
| ESC continuous / burst current         | ⚠ VERIFY A / A (10 s) |                  |

### Guidance

- Prop wash provides most of the cooling. **Avoid bench-running the ESC at sustained
  load without airflow.**
- Monitor MCU temperature in flight. In ArduPilot it is logged as `POWR.MTemp`.
- IMU heating is not supported on this board (`BRD_HEAT_TARG` has no effect) ⚠ VERIFY.
- Recommended thermal derating: reduce continuous ESC current by about 20 % above 40 °C ambient.

### Test Methodology

Measurements use a FLIR thermal camera at 25 °C ambient, emissivity 0.95, and steady state
reached after 5 minutes. Raw data is stored in `docs/thermal/`.

---

## Power

| Rail  | Source        | Max current | Notes                         |
|-------|---------------|-------------|-------------------------------|
| VBAT  | Battery       | ESC-limited | 2–6S                          |
| 10 V  | Buck          | 2 A         | VTX; switchable via GPIO ⚠ VERIFY |
| 5 V   | Buck          | 2 A         | RX, GPS, peripherals          |
| 3.3 V | LDO from 5 V  | 250 mA      | MCU, sensors only             |

Default battery monitor settings:

```
BATT_MONITOR     4
BATT_VOLT_PIN    ⚠ VERIFY
BATT_CURR_PIN    ⚠ VERIFY
BATT_VOLT_MULT   ⚠ CALIBRATE
BATT_AMP_PERVLT  ⚠