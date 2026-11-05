I don't have access to your repository's schematic, hwdef, or test data, so this README uses verified generic details (STM32H743 specs, ArduPilot build system, DFU flashing) and marks **board-specific values as `TBD` placeholders**. Pinouts and thermal figures are hardware facts, and invented numbers in a flight-controller README could cause miswiring or overheating, so fill those from your schematic and bench measurements. If you paste your `hwdef.dat`, schematic netlist, or thermal logs, I can populate the tables exactly.

---

````markdown
# Nano H743 AIO Flight Controller

![Status](https://img.shields.io/badge/status-TBD-lightgrey)
![MCU](https://img.shields.io/badge/MCU-STM32H743-blue)
![Firmware](https://img.shields.io/badge/firmware-ArduPilot-green)
![License](https://img.shields.io/badge/license-TBD-lightgrey)

A compact all-in-one (AIO) flight controller based on the STMicroelectronics
STM32H743 MCU, designed for ArduPilot (Copter / Plane / Rover / Sub).

> **Safety notice:** Always remove propellers before connecting power, flashing
> firmware, or changing configuration. Verify every pin assignment against the
> schematic in `/hardware` before wiring.

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Power, Battery Monitoring & BECs](#power-battery-monitoring--becs)
7. [Building ArduPilot](#building-arduplot)
8. [Flashing](#flashing)
9. [First-Time Configuration](#first-time-configuration)
10. [Thermal Specifications](#thermal-specifications)
11. [Electrical Ratings](#electrical-ratings)
12. [Troubleshooting](#troubleshooting)
13. [Repository Structure](#repository-structure)
14. [Contributing](#contributing)
15. [License](#license)

---

## Specifications

| Parameter            | Value                                             |
|----------------------|---------------------------------------------------|
| MCU                  | STM32H743VIT6 (Arm Cortex-M7, up to 480 MHz)      |
| Flash / RAM          | 2 MB / 1 MB                                       |
| IMU                  | `TBD` (e.g., ICM-42688-P / BMI270)                |
| Barometer            | `TBD` (e.g., DPS310 / BMP280)                     |
| OSD                  | `TBD` (e.g., AT7456E analog OSD)                  |
| Blackbox / Logging   | `TBD` (microSD / onboard flash, size)             |
| Integrated ESC       | `TBD` A continuous / `TBD` A burst, `TBD`S LiPo   |
| ESC protocol         | DShot 150/300/600, bidirectional DShot (`TBD`)    |
| Input voltage        | `TBD`S–`TBD`S LiPo (`TBD` V–`TBD` V)              |
| BEC outputs          | 5 V @ `TBD` A, `TBD` V @ `TBD` A                  |
| UARTs                | `TBD`                                             |
| I²C                  | `TBD` (compass / external sensors)                |
| CAN                  | `TBD` (FDCAN, yes/no)                             |
| USB                  | USB-C, full speed                                 |
| Mounting             | `TBD` × `TBD` mm, M`TBD`                          |
| Dimensions           | `TBD` × `TBD` × `TBD` mm                          |
| Weight               | `TBD` g                                           |

---

## Board Layout

```
        ┌───────────────────────────────┐
        │  [USB-C]         [BOOT btn]   │
        │                               │
        │   M1 ●                 ● M2   │
        │          ┌─────────┐          │
        │          │  H743   │          │
        │          └─────────┘          │
        │   M4 ●                 ● M3   │
        │                               │
        │  [VBAT+] [GND]   [Pads: TBD]  │
        └───────────────────────────────┘
          (Replace with photo/render: docs/img/top.png)
```

Images: `docs/img/top.png`, `docs/img/bottom.png`

---

## Pinout

> ⚠️ All MCU pin assignments must match `hwdef/NanoH743AIO/hwdef.dat`.
> The table below is the single source of truth for wiring documentation —
> keep it synchronized with the hwdef.

### Solder Pads

| Pad label | Function            | MCU pin | Voltage | Notes                     |
|-----------|---------------------|---------|---------|---------------------------|
| `VBAT`    | Battery input       | —       | `TBD`   | Main power                |
| `GND`     | Ground              | —       | 0 V     |                           |
| `5V`      | BEC output          | —       | 5 V     | Max `TBD` A               |
| `9V/VTX`  | VTX power           | —       | `TBD`   | Switchable? `TBD`         |
| `TX1`     | UART1 TX            | `TBD`   | 3.3 V   |                           |
| `RX1`     | UART1 RX            | `TBD`   | 3.3 V   |                           |
| `SDA`     | I²C SDA             | `TBD`   | 3.3 V   | Compass                   |
| `SCL`     | I²C SCL             | `TBD`   | 3.3 V   | Compass                   |
| `LED`     | WS2812 LED strip    | `TBD`   | 3.3 V   |                           |
| `BZ+/BZ-` | Buzzer              | `TBD`   | 5 V     |                           |
| `CURR`    | Current sense       | `TBD`   | 3.3 V   | ADC                       |
| `RSSI`    | Analog RSSI         | `TBD`   | 3.3 V   | ADC                       |
| `VTX`     | Video out           | —       | —       | Via OSD                   |
| `CAM`     | Video in            | —       | —       |                           |

### Internal Peripheral Map

| Peripheral | Bus   | MCU pins (SCK/MISO/MOSI/CS or SCL/SDA) | Notes       |
|------------|-------|----------------------------------------|-------------|
| IMU        | SPI`TBD` | `TBD`                               | DRDY: `TBD` |
| OSD        | SPI`TBD` | `TBD`                               |             |
| Flash/SD   | `TBD` | `TBD`                                  |             |
| Barometer  | I²C`TBD` | `TBD`                               |             |

---

## UART Mapping

| ArduPilot `SERIALn` | STM32 UART | Default protocol   | Pads        | DMA |
|---------------------|------------|--------------------|-------------|-----|
| SERIAL0             | USB (OTG1) | MAVLink2           | USB-C       | —   |
| SERIAL1             | `TBD`      | MAVLink2 (telem)   | `TBD`       | `TBD` |
| SERIAL2             | `TBD`      | GPS                | `TBD`       | `TBD` |
| SERIAL3             | `TBD`      | RCIN               | `TBD`       | `TBD` |
| SERIAL4             | `TBD`      | ESC telemetry      | `TBD`       | `TBD` |
| SERIAL5             | `TBD`      | DisplayPort / VTX  | `TBD`       | `TBD` |

Set the protocol with `SERIALn_PROTOCOL` and baud with `SERIALn_BAUD`.

---

## PWM / Motor Outputs

| Output | MCU pin | Timer/Channel | Group | Default function |
|--------|---------|---------------|-------|------------------|
| PWM1   | `TBD`   | `TBD`         | 1     | Motor 1          |
| PWM2   | `TBD`   | `TBD`         | 1     | Motor 2          |
| PWM3   | `TBD`   | `TBD`         | 2     | Motor 3          |
| PWM4   | `TBD`   | `TBD`         | 2     | Motor 4          |
| PWM5   | `TBD`   | `TBD`         | 3     | LED / Servo      |

> Outputs sharing a timer group **must** use the same protocol/rate
> (e.g., all DShot600 or all PWM at the same frequency).

Bidirectional DShot requires the bdshot firmware variant if your hwdef defines
`define HAL_WITH_BIDIR_DSHOT` / uses the `-bdshot` board target (`TBD`).

---

## Power, Battery Monitoring & BECs

Default battery-monitor parameters (verify against your shunt and divider):

| Parameter          | Value   |
|--------------------|---------|
| `BATT_MONITOR`     | 4 (Analog voltage and current) |
| `BATT_VOLT_PIN`    | `TBD`   |
| `BATT_CURR_PIN`    | `TBD`   |
| `BATT_VOLT_MULT`   | `TBD`   |
| `BATT_AMP_PERVLT`  | `TBD`   |
| `BATT_AMP_OFFSET`  | `TBD`   |

Calibrate `BATT_AMP_PERVLT` against a reference meter. Do not rely on defaults
for capacity-based failsafes.

---

## Building ArduPilot

### 1. Set up the environment (Ubuntu / WSL2)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

For macOS, use `Tools/environment_install/install-prereqs-mac.sh`.

### 2. Add the board definition

Copy the hwdef from this repository into the ArduPilot tree:

```bash
cp -r /path/to/nano-h743-aio-flight-controller/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

Expected contents:

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat          # main firmware definition
├── hwdef-bl.dat       # bootloader definition
├── defaults.parm      # default parameters (optional)
└── README.md          # board doc for upstream submission
```

Ensure the `APJ_BOARD_ID` in `hwdef.dat` is unique. Request an official ID
by adding it to `Tools/AP_Bootloader/board_types.txt` in an upstream PR.

### 3. Build the bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
```

Output: `Tools/bootloaders/NanoH743AIO_bl.bin` (and `.hex`)

### 4. Build the firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane, rover, sub, heli
```

Outputs in `build/NanoH743AIO/bin/`:

| File                         | Use                                      |
|------------------------------|------------------------------------------|
| `arducopter.apj`             | Upload via Mission Planner / QGC / USB   |
| `arducopter_with_bl.hex`     | First-time flash via DFU (includes BL)   |
| `arducopter.bin`             | Raw application binary                   |

### 5. Clean rebuild

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

---

## Flashing

### Method A — First flash via DFU (blank board or bootloader recovery)

1. Hold the **BOOT** button while connecting USB.
2. Confirm the device is detected:
   ```bash
   dfu-util -l
   # Look for: [0483:df11] ... "@Internal Flash"
   ```
3. Flash the combined bootloader + firmware.

   **Using STM32CubeProgrammer (recommended for `.hex`):**
   ```bash
   STM32_Programmer_CLI -c port=USB1 -e all \
       -w build/NanoH743AIO/bin/arducopter_with_bl.hex -v -rst
   ```

   **Using dfu-util (requires `.bin`):**
   ```bash
   dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave \
       -D Tools/bootloaders/NanoH743AIO_bl.bin
   ```
   Then upload the application with Method B.

4. Unplug and reconnect USB (without holding BOOT).

### Method B — Updates via the ArduPilot bootloader

```bash
./waf copter --upload
```

Or load `arducopter.apj` via **Mission Planner → Install Firmware → Load custom firmware**
or **QGroundControl → Firmware → Advanced → Custom firmware file**.

### Linux USB permissions

```bash
sudo usermod -aG dialout $USER
# log out and back in
```

---

## First-Time Configuration

1. Set frame: `FRAME_CLASS`, `FRAME_TYPE`.
2. Configure motor protocol: `MOT_PWM_TYPE` (e.g., `6` = DShot600).
3. Map outputs: `SERVO1_FUNCTION`…`SERVO4_FUNCTION` = 33–36.
4. Configure receiver UART: `SERIALn_PROTOCOL = 23` (RCIN).
5. Calibrate accelerometer and compass (if fitted).
6. Calibrate battery monitor (see above).
7. Verify motor order and direction with **Motor Test** (props off).
8. Configure failsafes: `FS_THR_ENABLE`, `BATT_FS_LOW_ACT`, `BATT_LOW_VOLT`.

---

## Thermal Specifications

> ⚠️ Board-level values must be measured on production hardware. Component
> limits below come from manufacturer datasheets. Board values are `TBD`
> until characterized.

### Component limits (datasheet)

| Component              | Parameter                    | Limit                 |
|------------------------|------------------------------|-----------------------|
| STM32H743VIT6          | Ambient operating (suffix 6) | −40 °C to +85 °C      |
| STM32H743              | Max junction temperature     | +125 °C               |
| IMU (`TBD`)            | Operating range              | `TBD`                 |
| ESC MOSFETs (`TBD`)    | Max junction temperature     | `TBD` (typ. 150–175 °C) |
| BEC regulator (`TBD`)  | Thermal shutdown             | `TBD`                 |

### Board-level characterization (to be measured)

| Condition                             | MCU temp | ESC FET temp | BEC temp | Notes |
|---------------------------------------|----------|--------------|----------|-------|
| Idle, USB powered, 25 °C ambient      | `TBD`    | —            | `TBD`    |       |
| Idle, battery, 25 °C ambient          | `TBD`    | `TBD`        | `TBD`    |       |
| `TBD` A continuous, no airflow, 60 s  | `TBD`    | `TBD`        | `TBD`    |       |
| `TBD` A continuous, prop-wash airflow | `TBD`    | `TBD`        | `TBD`    |       |
| `TBD` A burst, 10 s                   | `TBD`    | `