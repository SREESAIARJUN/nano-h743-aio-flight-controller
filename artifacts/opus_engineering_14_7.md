**Note:** I don't have access to the actual schematic, hwdef, or test data for `nano-h743-aio-flight-controller`, so I haven't invented board-specific values. A wrong pinout or thermal limit in a flight controller README can cause miswired peripherals, burned ESCs, or in-flight failures.

The README below is complete in structure and uses accurate, fixed information wherever it exists: STM32H743 silicon facts, ArduPilot build and flash workflow, and DFU procedures. Board-specific values are marked `⟨TBD⟩` and should be filled in from your schematic and bench measurements. If you paste your `hwdef.dat` or schematic netlist, I can populate every table.

---

```markdown
# Nano H743 AIO Flight Controller

![Status](https://img.shields.io/badge/status-pre--release-orange)
![MCU](https://img.shields.io/badge/MCU-STM32H743-blue)
![Firmware](https://img.shields.io/badge/firmware-ArduPilot-green)
![License](https://img.shields.io/badge/license-⟨TBD⟩-lightgrey)

A compact all-in-one (AIO) flight controller built on the STM32H743 (Arm Cortex-M7, up to 480 MHz).
It integrates ⟨TBD: 4-in-1 ESC rating / none⟩, OSD, barometer, IMU(s), and blackbox
storage on a ⟨TBD⟩ × ⟨TBD⟩ mm board with ⟨TBD⟩ mm mounting.

> ⚠️ **Safety:** Always remove propellers when powering the board over USB or
> configuring outputs. Verify motor order and direction in ArduPilot's Motor Test
> before the first flight.

---

## Table of Contents
1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout Charts](#pinout-charts)
4. [UART Mapping](#uart-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Power & Analog Sensing](#power--analog-sensing)
7. [Building ArduPilot](#building-ardupilot)
8. [Flashing](#flashing)
9. [First-Time Configuration](#first-time-configuration)
10. [Thermal Specifications](#thermal-specifications)
11. [Troubleshooting](#troubleshooting)
12. [Revision History](#revision-history)
13. [License](#license)

---

## Specifications

| Item                 | Value                                              |
|----------------------|----------------------------------------------------|
| MCU                  | STM32H743⟨TBD suffix, e.g. VIT6⟩, 480 MHz, 2 MB Flash, 1 MB RAM |
| IMU                  | ⟨TBD, e.g. ICM-42688-P⟩ (SPI)                      |
| Barometer            | ⟨TBD, e.g. DPS310 / BMP280⟩ (I²C/SPI)              |
| OSD                  | ⟨TBD, e.g. AT7456E⟩ (SPI)                          |
| Blackbox             | ⟨TBD: microSD (SDMMC) / SPI flash, capacity⟩       |
| Integrated ESC       | ⟨TBD: protocol, continuous/burst A, firmware⟩      |
| Input voltage        | ⟨TBD⟩ S LiPo (⟨TBD⟩–⟨TBD⟩ V)                       |
| BEC outputs          | 5 V @ ⟨TBD⟩ A, ⟨TBD⟩ V @ ⟨TBD⟩ A                   |
| UARTs                | ⟨TBD⟩                                              |
| CAN                  | ⟨TBD: yes/no, transceiver⟩                         |
| USB                  | ⟨TBD: USB-C⟩ (USB FS, PA11/PA12)                   |
| Dimensions           | ⟨TBD⟩ × ⟨TBD⟩ mm, ⟨TBD⟩ mm mounting, M⟨TBD⟩        |
| Weight               | ⟨TBD⟩ g                                            |

---

## Board Layout

<!-- Replace with rendered images from your PCB tool -->
| Top | Bottom |
|-----|--------|
| ![top](docs/img/top.png) | ![bottom](docs/img/bottom.png) |

---

## Pinout Charts

### Fixed silicon assignments (STM32H743)

These are determined by the MCU and are the same on every H743 board:

| Function    | MCU Pin | Notes                                  |
|-------------|---------|----------------------------------------|
| USB D−      | PA11    | OTG_FS_DM                              |
| USB D+      | PA12    | OTG_FS_DP                              |
| SWDIO       | PA13    | Debug, do not reassign                 |
| SWCLK       | PA14    | Debug, do not reassign                 |
| BOOT0       | BOOT0   | Hold high at reset to enter ROM DFU    |
| HSE         | PH0/PH1 | Crystal: ⟨TBD⟩ MHz                     |

### Board-specific assignments

> Fill these from `hwdef.dat`. The ArduPilot hwdef is the source of truth.
> The README must match it.

| Pad Label | Function     | MCU Pin | Peripheral       | Voltage | Notes |
|-----------|--------------|---------|------------------|---------|-------|
| ⟨TBD⟩     | IMU SCK/MISO/MOSI/CS | ⟨TBD⟩ | SPI⟨n⟩     | 3.3 V   |       |
| ⟨TBD⟩     | IMU DRDY     | ⟨TBD⟩   | EXTI             | 3.3 V   |       |
| ⟨TBD⟩     | Baro         | ⟨TBD⟩   | I2C⟨n⟩/SPI⟨n⟩    | 3.3 V   |       |
| ⟨TBD⟩     | OSD CS       | ⟨TBD⟩   | SPI⟨n⟩           | 3.3 V   |       |
| SDA / SCL | Ext. I²C (compass) | ⟨TBD⟩ | I2C⟨n⟩      | 3.3 V   | 5 V-tolerant? ⟨TBD⟩ |
| LED       | WS2812       | ⟨TBD⟩   | TIM⟨n⟩_CH⟨n⟩     | 5 V     |       |
| BZ+ / BZ− | Buzzer       | ⟨TBD⟩   | GPIO/PWM         | 5 V     |       |
| CAM       | Camera ctrl  | ⟨TBD⟩   | ⟨TBD⟩            | 3.3 V   |       |
| VTX       | Video out    | —       | OSD              | —       |       |

---

## UART Mapping

ArduPilot maps physical UARTs to `SERIALx` via the `SERIAL_ORDER` line in `hwdef.dat`.

| SERIALx  | MCU UART | TX Pin | RX Pin | Default Protocol      | Pad Label |
|----------|----------|--------|--------|-----------------------|-----------|
| SERIAL0  | OTG1     | PA12   | PA11   | MAVLink2 (USB)        | USB       |
| SERIAL1  | ⟨TBD⟩    | ⟨TBD⟩  | ⟨TBD⟩  | MAVLink2 (telem)      | ⟨TBD⟩     |
| SERIAL2  | ⟨TBD⟩    | ⟨TBD⟩  | ⟨TBD⟩  | MAVLink2              | ⟨TBD⟩     |
| SERIAL3  | ⟨TBD⟩    | ⟨TBD⟩  | ⟨TBD⟩  | GPS                   | ⟨TBD⟩     |
| SERIAL4  | ⟨TBD⟩    | ⟨TBD⟩  | ⟨TBD⟩  | GPS 2 / user          | ⟨TBD⟩     |
| SERIAL5  | ⟨TBD⟩    | ⟨TBD⟩  | ⟨TBD⟩  | ESC Telemetry (16)    | ⟨TBD⟩     |
| SERIAL6  | ⟨TBD⟩    | ⟨TBD⟩  | ⟨TBD⟩  | RC Input (23)         | ⟨TBD⟩     |
| SERIAL7  | ⟨TBD⟩    | ⟨TBD⟩  | ⟨TBD⟩  | DisplayPort / MSP     | ⟨TBD⟩     |

All UARTs support DMA unless noted. RC input supports CRSF/ELRS, SBUS (inverted,
⟨TBD: on dedicated pad⟩), IBUS, and other protocols auto-detected by `SERIALx_PROTOCOL 23`.

---

## PWM / Motor Outputs

| Output | MCU Pin | Timer       | DShot | BDShot | Group | Default Use |
|--------|---------|-------------|-------|--------|-------|-------------|
| M1     | ⟨TBD⟩   | TIM⟨n⟩_CH⟨n⟩ | ✅   | ⟨TBD⟩  | 1     | Motor 1     |
| M2     | ⟨TBD⟩   | TIM⟨n⟩_CH⟨n⟩ | ✅   | ⟨TBD⟩  | 1     | Motor 2     |
| M3     | ⟨TBD⟩   | TIM⟨n⟩_CH⟨n⟩ | ✅   | ⟨TBD⟩  | 1     | Motor 3     |
| M4     | ⟨TBD⟩   | TIM⟨n⟩_CH⟨n⟩ | ✅   | ⟨TBD⟩  | 1     | Motor 4     |
| S5     | ⟨TBD⟩   | TIM⟨n⟩_CH⟨n⟩ | ⟨TBD⟩ | ⟨TBD⟩ | 2     | Servo/Aux   |
| LED    | ⟨TBD⟩   | TIM⟨n⟩_CH⟨n⟩ | —     | —     | 3     | NeoPixel    |

> **Note:** All outputs sharing a timer group must use the same protocol and rate.
> For example, you cannot mix DShot and 50 Hz PWM servo within one group.

---

## Power & Analog Sensing

| Parameter              | Value   | ArduPilot Param       |
|------------------------|---------|-----------------------|
| Battery voltage pin    | ⟨TBD⟩   | `BATT_VOLT_PIN`       |
| Voltage divider scale  | ⟨TBD⟩   | `BATT_VOLT_MULT`      |
| Current sense pin      | ⟨TBD⟩   | `BATT_CURR_PIN`       |
| Current scale (A/V)    | ⟨TBD⟩   | `BATT_AMP_PERVLT`     |
| Current offset (V)     | ⟨TBD⟩   | `BATT_AMP_OFFSET`     |
| RSSI analog pin        | ⟨TBD⟩   | `RSSI_ANA_PIN`        |

Calibrate current with a known load. The default scale is a starting point and
varies with shunt tolerance.

---

## Building ArduPilot

### Prerequisites
Ubuntu 22.04 / 24.04 (WSL2 works):

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Add the board definition
Copy this repository's hwdef into the ArduPilot tree:

```bash
cp -r hwdef/NanoH743AIO libraries/AP_HAL_ChibiOS/hwdef/
# Contents:
#   hwdef.dat       – main firmware pin/peripheral definition
#   hwdef-bl.dat    – bootloader definition
#   defaults.parm   – board default parameters
#   README.md       – board doc (this file, for upstream submission)
```

### Build the bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

### Build the firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane, rover, sub, heli, antennatracker
```

Artifacts are written to `build/NanoH743AIO/bin/`:

| File                        | Use                                         |
|-----------------------------|---------------------------------------------|
| `arducopter.apj`            | Upload via Mission Planner / QGC / uploader |
| `arducopter_with_bl.hex`    | First-time flash via DFU (includes bootloader) |
| `arducopter.bin`            | Raw app image (offset 0x08020000)           |

### Clean rebuild
```bash
./waf distclean
./waf configure --board NanoH743AIO && ./waf copter
```

---

## Flashing

### Method A: First-time flash via ROM DFU

1. Hold the **BOOT** button and connect USB. Release the button after connecting.
2. Verify that the device is detected:
   ```bash
   dfu-util -l     # expect "Found DFU: [0483:df11] ... STM32 BOOTLOADER"
   ```
3. Flash the combined image.

   **STM32CubeProgrammer (recommended for .hex):**
   ```bash
   STM32_Programmer_CLI -c port=usb1 -w arducopter_with_bl.hex -v -rst
   ```

   **dfu-util (.bin with bootloader at 0x08000000):**
   ```bash
   dfu-util -a 0 --dfuse-address 0x08000000:leave -D arducopter_with_bl.bin
   ```

   **Windows:** If the device shows as "STM32 BOOTLOADER" without a driver,
   install WinUSB using [Zadig](https://zadig.akeo.ie/).
4. Unplug and replug the board. The status LED should show ArduPilot's boot pattern.

### Method B: Updates (ArduPilot bootloader already installed)

```bash
./waf copter --upload
# or
Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
```

You can also use Mission Planner → *Install Firmware* → *Load custom firmware* → `.apj`.

### Method C: SWD (recovery)

```bash
STM32_Programmer_CLI -c port=SWD -e all -w arducopter_with_bl.hex -v -rst
```

---

## First-Time Configuration

1. Connect with Mission Planner or QGroundControl.
