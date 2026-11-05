I don't have access to your repository's schematics, hwdef files, or test data, so I can't know your board's actual pin assignments or measured thermal figures. This README uses real ArduPilot tooling and STM32H743 datasheet values. Every board-specific value is marked `⚠️ VERIFY` or `TBD` and must be replaced with data from your schematic and `hwdef.dat` before publishing. Publishing an invented pinout could cause someone to wire an ESC to the wrong pad.

---

```markdown
# Nano H743 AIO Flight Controller

![Status](https://img.shields.io/badge/status-pre--release-orange)
![MCU](https://img.shields.io/badge/MCU-STM32H743-blue)
![Firmware](https://img.shields.io/badge/firmware-ArduPilot-green)
![License](https://img.shields.io/badge/license-TBD-lightgrey)

An all-in-one (AIO) flight controller built on the STM32H743. It targets compact
multirotors and small fixed-wing aircraft running ArduPilot.

> **Revision:** Cycle 12 · **Hardware rev:** `TBD` · **Doc last verified against hwdef:** `TBD`

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout](#pinout)
4. [UART Mapping](#uart-mapping)
5. [PWM Outputs & Timer Groups](#pwm-outputs--timer-groups)
6. [Power & Analog](#power--analog)
7. [Building ArduPilot](#building-ardupilot)
8. [Flashing](#flashing)
9. [First-Time Setup](#first-time-setup)
10. [Thermal Specifications](#thermal-specifications)
11. [Troubleshooting](#troubleshooting)
12. [Repository Structure](#repository-structure)
13. [Contributing](#contributing)
14. [License](#license)

---

## Specifications

| Parameter            | Value                                                  |
|----------------------|--------------------------------------------------------|
| MCU                  | STM32H743VIT6, Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM |
| IMU                  | `TBD` (e.g., ICM-42688-P) ⚠️ VERIFY                    |
| Barometer            | `TBD` (e.g., DPS310 / BMP280) ⚠️ VERIFY                |
| OSD                  | `TBD` (e.g., AT7456E) ⚠️ VERIFY                        |
| Blackbox / logging   | `TBD` (microSD / SPI flash) ⚠️ VERIFY                  |
| Input voltage        | `TBD` S LiPo (`TBD`–`TBD` V) ⚠️ VERIFY                 |
| BEC outputs          | 5 V @ `TBD` A, 10 V @ `TBD` A (VTX) ⚠️ VERIFY          |
| UARTs                | `TBD`                                                  |
| PWM outputs          | `TBD`                                                  |
| I²C                  | `TBD` (external compass / peripherals)                 |
| CAN                  | `TBD` (H743 supports FDCAN; populated? ⚠️ VERIFY)      |
| USB                  | USB-C, USB 2.0 FS                                      |
| Mounting             | `TBD` × `TBD` mm, M`TBD`                               |
| Dimensions           | `TBD` × `TBD` × `TBD` mm                               |
| Weight               | `TBD` g                                                |

---

## Board Layout

<!-- Replace with real renders/photos exported from your ECAD tool -->
| Top | Bottom |
|-----|--------|
| ![Top](docs/img/top.png) | ![Bottom](docs/img/bottom.png) |

---

## Pinout

> ⚠️ **All MCU pin assignments below are placeholders.** Copy the actual
> assignments from `hwdef/NanoH743AIO/hwdef.dat`. The hwdef file is the
> single source of truth. If it disagrees with this table, the hwdef wins.

### Solder Pad Map

| Pad Label | Function            | MCU Pin | Voltage Level | Notes                       |
|-----------|---------------------|---------|---------------|-----------------------------|
| `BAT+`    | Battery input       | —       | VBAT          | Via onboard ESC/power stage |
| `GND`     | Ground              | —       | 0 V           |                             |
| `5V`      | 5 V BEC out         | —       | 5 V           | Max `TBD` A                 |
| `10V`     | VTX supply          | —       | 10 V          | Switchable? ⚠️ VERIFY       |
| `TX1/RX1` | UART1               | `TBD`   | 3.3 V         |                             |
| `TX2/RX2` | UART2               | `TBD`   | 3.3 V         |                             |
| `SDA/SCL` | I²C                 | `TBD`   | 3.3 V         | Pull-ups onboard? ⚠️ VERIFY |
| `LED`     | WS2812 LED strip    | `TBD`   | 5 V data?     | ⚠️ VERIFY                   |
| `BZ+/BZ-` | Buzzer              | `TBD`   | 5 V           |                             |
| `CURR`    | Current sense       | `TBD`   | 0–3.3 V       |                             |
| `RSSI`    | Analog RSSI         | `TBD`   | 0–3.3 V       |                             |

---

## UART Mapping

ArduPilot assigns `SERIALn` ports in the order listed by `SERIAL_ORDER` in
`hwdef.dat`.

| ArduPilot Port | Hardware | Default Protocol            | Pads        | DMA      |
|----------------|----------|-----------------------------|-------------|----------|
| SERIAL0        | OTG1     | MAVLink2 (USB)              | USB-C       | —        |
| SERIAL1        | `TBD`    | MAVLink2 (telemetry)        | `TBD`       | `TBD`    |
| SERIAL2        | `TBD`    | `TBD`                       | `TBD`       | `TBD`    |
| SERIAL3        | `TBD`    | GPS (`SERIAL3_PROTOCOL=5`)  | `TBD`       | `TBD`    |
| SERIAL4        | `TBD`    | `TBD`                       | `TBD`       | `TBD`    |
| SERIAL5        | `TBD`    | ESC telemetry (`=16`)       | `TBD`       | `TBD`    |
| SERIAL6        | `TBD`    | RC input (`=23`)            | `TBD`       | `TBD`    |

---

## PWM Outputs & Timer Groups

Outputs that share a hardware timer **must run the same protocol and rate**.
For example, you cannot mix DShot and standard PWM within one group.

| Output | MCU Pin | Timer/Channel | Group | Bidirectional DShot |
|--------|---------|---------------|-------|---------------------|
| M1     | `TBD`   | `TBD`         | 1     | ⚠️ VERIFY           |
| M2     | `TBD`   | `TBD`         | 1     | ⚠️ VERIFY           |
| M3     | `TBD`   | `TBD`         | 2     | ⚠️ VERIFY           |
| M4     | `TBD`   | `TBD`         | 2     | ⚠️ VERIFY           |
| LED    | `TBD`   | `TBD`         | `TBD` | No                  |

---

## Power & Analog

Set these values in your hwdef defaults. Calibrate against a meter before flight.

| Parameter              | Default    | Notes                              |
|------------------------|------------|------------------------------------|
| `BATT_MONITOR`         | 4          | Analog voltage + current           |
| `BATT_VOLT_PIN`        | `TBD`      | ADC channel from hwdef             |
| `BATT_CURR_PIN`        | `TBD`      | ADC channel from hwdef             |
| `BATT_VOLT_MULT`       | `TBD`      | From divider ratio (R1+R2)/R2      |
| `BATT_AMP_PERVLT`      | `TBD`      | From shunt/sense amp gain          |

---

## Building ArduPilot

### Prerequisites

Supported build hosts are Ubuntu 22.04/24.04 and WSL2. On macOS, follow the
[ArduPilot macOS guide](https://ardupilot.org/dev/docs/building-setup-mac.html).

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Add the Board Definition

Until the board is merged upstream, copy the hwdef from this repo:

```bash
cp -r /path/to/nano-h743-aio-flight-controller/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

The hwdef directory must contain these files:

```
NanoH743AIO/
├── hwdef.dat         # main firmware definition
├── hwdef-bl.dat      # bootloader definition
├── defaults.parm     # board default parameters
└── README.md         # upstream-format board README
```

### Build the Bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin (and .hex)
```

### Build the Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter      # or: plane | rover | sub | heli
```

The build writes its artifacts to `build/NanoH743AIO/bin/`:

| File                    | Use                                              |
|-------------------------|--------------------------------------------------|
| `arducopter.apj`        | Upload via Mission Planner / QGC / MAVProxy      |
| `arducopter_with_bl.hex`| Full image (bootloader + firmware) for DFU/SWD   |
| `arducopter.bin`        | Raw application image                            |

### Clean Rebuild

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

---

## Flashing

### Method 1: DFU (first flash, or a board without an ArduPilot bootloader)

1. Hold the **BOOT** button while connecting USB, then release it.
2. Confirm that the board enumerates as a DFU device:
   ```bash
   dfu-util -l
   # Expect: "STM32 BOOTLOADER", alt=0, "@Internal Flash"
   ```
3. Flash the combined image. You can use STM32CubeProgrammer:
   ```bash
   STM32_Programmer_CLI -c port=usb1 -w arducopter_with_bl.hex -v -rst
   ```
   You can also convert the image to a binary and flash it with dfu-util:
   ```bash
   arm-none-eabi-objcopy -I ihex -O binary arducopter_with_bl.hex fw.bin
   dfu-util -a 0 --dfuse-address 0x08000000:leave -D fw.bin
   ```

> **Linux:** If `dfu-util` reports `LIBUSB_ERROR_ACCESS`, install STM32 udev
> rules or run the command with `sudo`.

### Method 2: ArduPilot Bootloader (routine updates)

Use this method once a bootloader is installed:

```bash
./waf copter --upload
```

You can also load `arducopter.apj` through **Mission Planner → Install Firmware →
Load custom firmware** or **QGroundControl → Firmware → Advanced → Custom**.

### Method 3: SWD (recovery)

Use this method on a bricked board via the SWD test pads (`SWDIO`, `SWCLK`, `GND`, `3V3`):

```bash
STM32_Programmer_CLI -c port=SWD -e all -w arducopter_with_bl.hex -v -rst
```

### Updating the Bootloader Only

Once ArduPilot is running, you can update the bootloader from the MAVLink
console. Do not power-cycle the board during this operation:

```
flash-bootloader
```

---

## First-Time Setup

1. Connect to a GCS (Mission Planner, QGroundControl, or MAVProxy).
2. Set the frame type: `FRAME_CLASS`, `FRAME_TYPE`.
3. Set the motor protocol: `MOT_PWM_TYPE=6` (DShot600), or as appropriate.
4. Enable bidirectional DShot if your ESCs support it: `SERVO_BLH_BDMASK`.
5. Run the accelerometer, compass (if fitted), and radio calibration.
6. Verify motor order and direction with **Motor Test**, with props removed.
7. Calibrate the battery voltage and current against a multimeter.

---

## Thermal Specifications

> **Datasheet limits** come from STMicroelectronics (STM32H743xI datasheet,
> DS12110). **Board-level values** are `TBD` and must be filled in from your
> own thermal testing. Do not publish estimates as measurements.

### Component Limits

| Component           | Parameter                     | Value                    | Source       |
|---------------------|-------------------------------|--------------------------|--------------|
| STM32H743 (VIT6)    | Ambient operating range       | −40 °C to +85 °C         | Datasheet    |
| STM32H743           | Max junction temp (Tj)        | +125 °C (suffix-dependent; check ordering code) | Datasheet |
| IMU                 | Operating range               | `TBD` (typ. −40 to +85 °C) | IMU datasheet |
| BEC regulator       | Thermal shutdown              | `TBD`                    | Reg. datasheet |
| ESC MOSFETs (AIO)   | Max Tj                        | `TBD` (typ. 150–175 °C)  | FET datasheet |

### Board-Level Measurements (to be completed)

| Condition                               | MCU Temp | IMU Temp | BEC Temp | FET Temp | Method       |
|-----------------------------------------|----------|----------|----------|----------|--------------|
| Idle, USB powered, 25 °C ambient        | `TBD`    | `TBD`    | `TBD`    | —        | `TBD`        |
| Armed, hover, 25 °C ambient, no airflow | `TBD`    | `TBD`    | `TBD`    | `TBD`    | `TBD`        |
| Full BEC load (`TBD` A), 25 °C          | `TBD`    | `TBD`    | `TBD`    | —        | `TBD`        |
| Sustained `TBD` A/motor, prop airflow   | `TBD`    | `TBD`    | `TBD`    | `TBD`    | `TBD`        |

Record each result together with its test method (thermocouple location, IR
emissivity setting, or onboard sensor), the soak time, and the enclosure state.

### Operating Guidance

- **Bench testing:** AIO boards can overheat on the bench with no prop wash.
  Limit armed bench time, or provide forced airflow.
- **VTX on 10 V rail:** High-power VTXs add significant heat near the BEC.
  Avoid running them at full power on the bench.
- **IMU temperature:** You can monitor the IMU temperature in logs (`IMU.T`).
  If the hwdef supports IMU heating (`BRD_HEAT_TARG`), document whether it is
  supported on this board.
- **Conformal coating:** Coating affects heat dissipation. If you apply it,
  re-validate the thermal measurements above.

---

## Troubleshooting

| Symptom