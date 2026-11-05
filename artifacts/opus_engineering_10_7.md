I don't have access to your repository, schematic, or prior cycles, so I can't confirm board-specific values. This README uses correct ArduPilot and STM32H743 procedures throughout. Every value that depends on your hardware is marked **⚠ VERIFY**. Before you merge, replace those values with data from your schematic, `hwdef.dat`, and thermal test logs. Publishing unverified pin assignments can lead users to wire ESCs or receivers to the wrong pads.

---

````markdown
# Nano H743 AIO Flight Controller

<p align="center">
  <img src="docs/images/board-top.png" alt="Nano H743 AIO – top view" width="420">
</p>

<p align="center">
  <a href="#"><img src="https://img.shields.io/badge/MCU-STM32H743-blue" alt="MCU"></a>
  <a href="#"><img src="https://img.shields.io/badge/firmware-ArduPilot-green" alt="ArduPilot"></a>
  <a href="#"><img src="https://img.shields.io/badge/hardware%20rev-v1.0-orange" alt="Rev"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-see%20LICENSE-lightgrey" alt="License"></a>
</p>

A compact all-in-one (AIO) flight controller built on the STM32H743 (480 MHz Cortex-M7). It integrates a 4-in-1 ESC and targets sub-250 g multirotors, cinewhoops, and small autonomous platforms running **ArduPilot**.

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout & Pinout](#board-layout--pinout)
3. [UART Mapping](#uart-mapping)
4. [Motor / PWM Outputs](#motor--pwm-outputs)
5. [Power & Electrical Limits](#power--electrical-limits)
6. [Thermal Specifications](#thermal-specifications)
7. [Building ArduPilot](#building-arducopter-firmware)
8. [Flashing](#flashing)
9. [First-Time Configuration](#first-time-configuration)
10. [Troubleshooting](#troubleshooting)
11. [Repository Structure](#repository-structure)
12. [Contributing](#contributing)
13. [Safety Notice](#safety-notice)
14. [License](#license)

---

## Specifications

| Parameter            | Value                                                    |
|----------------------|----------------------------------------------------------|
| MCU                  | STM32H743VIT6 (480 MHz, 2 MB Flash, 1 MB RAM) ⚠ VERIFY package |
| IMU                  | ⚠ VERIFY (e.g. ICM-42688-P / BMI270), SPI               |
| Barometer            | ⚠ VERIFY (e.g. DPS310 / BMP280), I²C                    |
| OSD                  | ⚠ VERIFY (AT7456E analog / MSP DisplayPort only)        |
| Blackbox / Logging   | ⚠ VERIFY (microSD / onboard SPI flash, size)            |
| Integrated ESC       | 4-in-1, ⚠ VERIFY continuous / burst current, BLHeli_S / AM32 / Bluejay |
| ESC protocol         | DShot300/600 with bidirectional telemetry ⚠ VERIFY      |
| Input voltage        | ⚠ VERIFY (e.g. 2–4S LiPo)                               |
| BEC outputs          | 5 V @ ⚠ VERIFY A, 9/10 V @ ⚠ VERIFY A (VTX)            |
| UARTs                | ⚠ VERIFY count                                           |
| Mounting             | ⚠ VERIFY (e.g. 25.5 × 25.5 mm, M2)                      |
| Dimensions / Weight  | ⚠ VERIFY mm / g                                          |
| USB                  | USB-C, full-speed (DFU capable)                          |

---

## Board Layout & Pinout

> **⚠ VERIFY:** The tables below are a template. They must match
> `hwdef/NanoH743AIO/hwdef.dat` exactly. The hwdef file is the source of truth.

<p align="center">
  <img src="docs/images/pinout-top.png" alt="Top pinout" width="45%">
  <img src="docs/images/pinout-bottom.png" alt="Bottom pinout" width="45%">
</p>

### Solder Pads

| Pad       | Function                         | MCU Pin    | Notes                       |
|-----------|----------------------------------|------------|-----------------------------|
| VBAT / GND| Battery input                    | —          | Main power, see limits      |
| 5V        | 5 V BEC out                      | —          |                             |
| 9V / 10V  | VTX BEC out                      | —          | Switchable via ⚠ VERIFY GPIO|
| TX1 / RX1 | SERIAL1                          | ⚠ VERIFY   |                             |
| TX2 / RX2 | SERIAL2                          | ⚠ VERIFY   |                             |
| SDA / SCL | I²C (external compass)           | ⚠ VERIFY   | 2.2 kΩ pull-ups onboard?    |
| LED       | WS2812 serial LED                | ⚠ VERIFY   | Timer-capable pin           |
| BZ+ / BZ− | Buzzer                           | ⚠ VERIFY   |                             |
| CURR      | Current sensor (internal)        | ⚠ VERIFY ADC |                           |
| RSSI      | Analog RSSI                      | ⚠ VERIFY ADC |                           |
| BOOT      | Bootloader (DFU) button / pads   | BOOT0      |                             |

### Onboard Peripherals

| Device    | Bus     | Pins (SCK/MISO/MOSI/CS or SCL/SDA) | DRDY / INT |
|-----------|---------|------------------------------------|------------|
| IMU       | SPI1    | ⚠ VERIFY                            | ⚠ VERIFY   |
| OSD       | SPI2    | ⚠ VERIFY                            | —          |
| Flash/SD  | SPI3 / SDMMC | ⚠ VERIFY                       | —          |
| Baro      | I2C2    | ⚠ VERIFY                            | —          |

---

## UART Mapping

| ArduPilot `SERIALn` | STM32 Peripheral | Default Protocol (`SERIALn_PROTOCOL`) | DMA |
|---------------------|------------------|---------------------------------------|-----|
| SERIAL0             | USB (OTG1)       | MAVLink2 (2)                          | —   |
| SERIAL1             | ⚠ VERIFY         | MAVLink2 (2) – telemetry              | ⚠   |
| SERIAL2             | ⚠ VERIFY         | ⚠ VERIFY                              | ⚠   |
| SERIAL3             | ⚠ VERIFY         | GPS (5)                               | ⚠   |
| SERIAL4             | ⚠ VERIFY         | ⚠ VERIFY                              | ⚠   |
| SERIAL5             | ⚠ VERIFY         | ESC Telemetry (16)                    | ⚠   |
| SERIAL6             | ⚠ VERIFY         | RCIN (23)                             | ⚠   |

**Receiver:** Connect CRSF/ELRS, SBUS, or other serial RC to the UART marked RCIN, then set `SERIALn_PROTOCOL = 23`. For SBUS inversion, see `SERIALn_OPTIONS`. On H7 parts, UARTs support hardware inversion.

---

## Motor / PWM Outputs

| Output | MCU Pin  | Timer / Channel | Group | Default Function |
|--------|----------|-----------------|-------|------------------|
| M1     | ⚠ VERIFY | ⚠ TIMx_CHy      | 1     | Motor 1          |
| M2     | ⚠ VERIFY | ⚠ TIMx_CHy      | 1     | Motor 2          |
| M3     | ⚠ VERIFY | ⚠ TIMx_CHy      | 1     | Motor 3          |
| M4     | ⚠ VERIFY | ⚠ TIMx_CHy      | 1     | Motor 4          |
| LED    | ⚠ VERIFY | ⚠ TIMx_CHy      | 2     | Serial LED       |

> Outputs in the same group share a timer and **must use the same protocol and
> rate**. Bidirectional DShot requires a DMA-capable timer channel for each output.

---

## Power & Electrical Limits

| Parameter                         | Min | Typ | Max | Unit |
|-----------------------------------|-----|-----|-----|------|
| Battery input                     | ⚠   | —   | ⚠   | V    |
| ESC continuous current (per motor)| —   | —   | ⚠   | A    |
| ESC burst current (≤10 s)         | —   | —   | ⚠   | A    |
| 5 V BEC output                    | —   | 5.0 | ⚠   | V / A|
| VTX BEC output                    | —   | ⚠   | ⚠   | V / A|
| USB-only supply (no battery)      | —   | 5.0 | —   | V    |

**Battery monitor defaults** (set in `hwdef.dat`, then verify with a calibrated meter):

```
BATT_MONITOR      4
BATT_VOLT_PIN     ⚠ VERIFY
BATT_CURR_PIN     ⚠ VERIFY
BATT_VOLT_MULT    ⚠ VERIFY
BATT_AMP_PERVLT   ⚠ VERIFY
```

---

## Thermal Specifications

> **⚠ VERIFY:** The MCU limits below come from the STM32H743 datasheet. Board-level
> figures **must** come from your own measurements. Record the test method and date,
> and include the raw logs in `docs/thermal/`.

### Component Ratings

| Component        | Rating                                   | Source                  |
|------------------|------------------------------------------|-------------------------|
| STM32H743 (suffix 6) | Ambient −40 °C to +85 °C             | ST datasheet DS12110    |
| STM32H743 (suffix 3) | Ambient −40 °C to +125 °C            | ST datasheet DS12110    |
| IMU              | ⚠ VERIFY per datasheet                   |                         |
| ESC MOSFETs      | T<sub>J</sub> max ⚠ VERIFY °C            | MOSFET datasheet        |
| BEC regulator    | Thermal shutdown ⚠ VERIFY °C             | Regulator datasheet     |

### Measured Board Performance (⚠ fill from test data)

| Condition                                   | MCU Temp | ESC Hotspot | BEC Temp | Ambient |
|---------------------------------------------|----------|-------------|----------|---------|
| USB powered, idle, no airflow               | ⚠ °C     | —           | ⚠ °C     | 25 °C   |
| Battery, disarmed, VTX on, no airflow       | ⚠ °C     | ⚠ °C        | ⚠ °C     | 25 °C   |
| Hover, ⚠ A total, prop wash                 | ⚠ °C     | ⚠ °C        | ⚠ °C     | 25 °C   |
| Continuous ⚠ A/motor, 30 s, prop wash       | ⚠ °C     | ⚠ °C        | ⚠ °C     | 25 °C   |

**Test method:** ⚠ describe it here (e.g. FLIR model, thermocouple placement, airflow, frame).

### Thermal Guidance

- Don't leave the board powered from a battery on the bench with the VTX on and no airflow. AIO boards can exceed component limits within minutes.
- Leave at least ⚠ mm clearance above the ESC section. Don't wrap it in tape or heat-shrink.
- Use the IMU temperature log message (`IMU.T`) to monitor MCU and IMU temperature. On H7 targets, `MCU` log messages report MCU temperature.
- If the board has an IMU heater, enable it with `BRD_HEAT_TARG`. ⚠ VERIFY that heater hardware is present.
- Derate continuous ESC current by ⚠ VERIFY %/°C above 25 °C ambient.

---

## Building ArduCopter Firmware

### 1. Set Up the Build Environment

Tested on Ubuntu 22.04. For other systems, see the [ArduPilot build docs](https://ardupilot.org/dev/docs/building-the-code.html).

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### 2. Add the Board Definition

Until this target is merged upstream, copy the hwdef from this repo:

```bash
cp -r /path/to/nano-h743-aio-flight-controller/hwdef/NanoH743AIO \
      libraries/AP_HAL_ChibiOS/hwdef/
```

The directory must contain:

```
NanoH743AIO/
├── hwdef.dat        # Main board definition
├── hwdef-bl.dat     # Bootloader definition
├── defaults.parm    # Default parameters
└── README.md        # Upstream board docs (required for PR)
```

The `APJ_BOARD_ID` in `hwdef.dat` must be unique. Request one by PR to `Tools/AP_Bootloader/board_types.txt`. **⚠ VERIFY the assigned ID.**

### 3. Build the Bootloader (first time / bootloader changes only)

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / _bl.hex
```

### 4. Build the Vehicle Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter          # or: plane, rover, sub, heli
```

Output files are written to `build/NanoH743AIO/bin/`:

| File                        | Use                                        |
|-----------------------------|--------------------------------------------|
| `arducopter.apj`            | Upload via Mission Planner / QGC / uploader |
| `arducopter_with_bl.hex`    | First-time flash via DFU (includes bootloader) |
| `arducopter.bin`            | Raw binary (no bootloader)                 |

Optional: build with specific features using `--enable-<feature>` / `--disable-<feature>`. You can also use the custom firmware server at [custom.ardupilot.org](https://custom.ardupilot.org) once the board is upstream.

---

## Flashing

### Method A: First Flash via DFU (blank board or bootloader recovery)

1. Unplug the battery.
2. Hold the **BOOT** button, or bridge the BOOT pads, while plugging in USB. Then release.
3. Confirm the board is in DFU mode:
   ```bash
   dfu-util -l    # should list "STM32 BOOTLOADER" (0483:df11)
   ```
4. Flash the combined image.

   **STM32CubeProgrammer (recommended, handles .hex):**
   ```bash
   STM32_Programmer_CLI -c port=USB1 -w arducopter_with_bl.hex -v -rst
   ```

   **dfu-util (requires .bin at 0x08000000):**
   ```bash
   arm-none-eabi-objcopy -I ihex -O binary arducopter_with_bl.hex arducopter_with_bl.bin
   dfu-util -a 0 -s 0x08000000:leave -D arducopter_with_bl.bin
   ```

On Windows, use