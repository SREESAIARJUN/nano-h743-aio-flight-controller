I don't have access to the `nano-h743-aio-flight-controller` repository, its schematics, or the "Cycle 13" revision notes. So I wrote this README with two kinds of content:

- **Real, verifiable content:** STM32H743 facts and the standard ArduPilot build and flash workflow.
- **Board-specific values** (pin assignments, current-sensor scale, thermal measurements) are marked `⚠️ TBD`. Fill these in from your hwdef and test data. I didn't invent them, because a wrong pinout or current scale in a flight-controller README can cause hardware damage or a crash.

---

```markdown
# Nano H743 AIO Flight Controller

> Compact all-in-one (AIO) STM32H743 flight controller for ArduPilot (Copter / Plane / Rover).
> **Hardware revision:** Cycle 13 · **Firmware:** ArduPilot (ChibiOS HAL)

![Board top](docs/img/board-top.png) ![Board bottom](docs/img/board-bottom.png)

---

## Table of Contents
1. [Overview](#overview)
2. [Specifications](#specifications)
3. [Pinout](#pinout)
4. [UART / Serial Mapping](#uart--serial-mapping)
5. [PWM Outputs & Timer Groups](#pwm-outputs--timer-groups)
6. [Power & Battery Monitoring](#power--battery-monitoring)
7. [Building ArduPilot](#building-arduploilot)
8. [Flashing](#flashing)
9. [Thermal Specifications](#thermal-specifications)
10. [Mounting & Dimensions](#mounting--dimensions)
11. [Troubleshooting](#troubleshooting)
12. [Revision History](#revision-history)
13. [License](#license)

---

## Overview

The Nano H743 AIO combines the flight controller and peripherals on one board for small multirotors and fixed-wing aircraft.

**Key features:**
- STM32H743 MCU (Arm Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM)
- IMU(s): ⚠️ TBD (e.g., ICM-42688-P / BMI270)
- Barometer: ⚠️ TBD
- OSD: ⚠️ TBD (e.g., AT7456E analog OSD)
- Blackbox: ⚠️ TBD (microSD or onboard flash)
- Integrated ESC: ⚠️ TBD (if AIO includes ESC, specify continuous/burst current)

---

## Specifications

| Parameter            | Value                                   |
|----------------------|-----------------------------------------|
| MCU                  | STM32H743VIT6 (⚠️ confirm package/suffix) |
| Core                 | Cortex-M7, 480 MHz, double-precision FPU |
| Flash / RAM          | 2 MB / 1 MB                             |
| IMU                  | ⚠️ TBD (SPI bus ⚠️ TBD)                   |
| Barometer            | ⚠️ TBD (I2C/SPI ⚠️ TBD)                   |
| Input voltage        | ⚠️ TBD (e.g., 2–6S LiPo)                  |
| BEC outputs          | 5 V @ ⚠️ TBD A, 9 V/10 V @ ⚠️ TBD A        |
| UARTs                | ⚠️ TBD                                    |
| PWM outputs          | ⚠️ TBD                                    |
| Mounting             | ⚠️ TBD (e.g., 25.5 × 25.5 mm, M2)          |
| Weight               | ⚠️ TBD g                                  |

---

## Pinout

> Source of truth: `libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/hwdef.dat`.
> If this table and the hwdef disagree, **the hwdef wins**. Please open an issue.

### Pad Diagram

```
          ┌──────────────── TOP ────────────────┐
   M1 ●   │                                     │   ● M2
          │   [USB-C]                [SD]       │
   5V ●   │                                     │   ● GND
   T1 ●   │          STM32H743                  │   ● R1
   T2 ●   │                                     │   ● R2
   ...    │          (⚠️ replace with real        │   ...
          │           pad layout from PCB)      │
   M4 ●   │                                     │   ● M3
          └─────────────────────────────────────┘
```

### MCU Pin Assignment

| Function        | MCU Pin | Peripheral      | Notes                  |
|-----------------|---------|-----------------|------------------------|
| IMU SCK/MISO/MOSI | ⚠️ TBD | SPI⚠️           |                        |
| IMU CS          | ⚠️ TBD  | GPIO            |                        |
| IMU INT/DRDY    | ⚠️ TBD  | EXTI            |                        |
| Baro SDA/SCL    | ⚠️ TBD  | I2C⚠️           |                        |
| OSD CS          | ⚠️ TBD  | SPI⚠️           |                        |
| SD card         | ⚠️ TBD  | SDMMC1          |                        |
| VBAT sense      | ⚠️ TBD  | ADC⚠️           | Divider: ⚠️ TBD         |
| Current sense   | ⚠️ TBD  | ADC⚠️           | Scale: ⚠️ TBD A/V       |
| LED (status)    | ⚠️ TBD  | GPIO            |                        |
| Buzzer          | ⚠️ TBD  | GPIO/TIM        |                        |
| Boot / DFU      | BOOT0   | —               | Pad or button location ⚠️ |

---

## UART / Serial Mapping

| ArduPilot SERIALn | MCU UART | Pads   | Default Protocol      | DMA |
|-------------------|----------|--------|-----------------------|-----|
| SERIAL0           | USB      | USB-C  | MAVLink2              | —   |
| SERIAL1           | ⚠️ USART⚠️ | T1/R1  | MAVLink2 (telem)      | ⚠️  |
| SERIAL2           | ⚠️       | T2/R2  | ⚠️ (e.g., RCIN / CRSF) | ⚠️  |
| SERIAL3           | ⚠️       | T3/R3  | GPS                   | ⚠️  |
| SERIAL4           | ⚠️       | T4/R4  | ⚠️                    | ⚠️  |
| SERIAL6           | ⚠️       | T6/R6  | ⚠️ (e.g., ESC telem)   | ⚠️  |

**RC input:** ⚠️ TBD. Describe whether a dedicated RCIN timer pad exists, or whether RC runs over a UART with `SERIALn_PROTOCOL = 23`.

---

## PWM Outputs & Timer Groups

Outputs that share a timer **must use the same protocol and rate** (e.g., all DShot600 or all PWM 400 Hz).

| Output  | MCU Pin | Timer/Ch   | Group | DShot | BiDir DShot |
|---------|---------|------------|-------|-------|-------------|
| M1      | ⚠️      | TIM⚠️_CH⚠️ | 1     | ✅    | ⚠️          |
| M2      | ⚠️      | TIM⚠️_CH⚠️ | 1     | ✅    | ⚠️          |
| M3      | ⚠️      | TIM⚠️_CH⚠️ | 2     | ✅    | ⚠️          |
| M4      | ⚠️      | TIM⚠️_CH⚠️ | 2     | ✅    | ⚠️          |
| LED     | ⚠️      | TIM⚠️_CH⚠️ | 3     | —     | —           |

---

## Power & Battery Monitoring

Default parameters (set in hwdef `defaults.parm`):

```
BATT_MONITOR     4
BATT_VOLT_PIN    ⚠️ TBD
BATT_CURR_PIN    ⚠️ TBD
BATT_VOLT_MULT   ⚠️ TBD
BATT_AMP_PERVLT  ⚠️ TBD
```

> **Calibrate before flight.** Measure the pack with a multimeter and set `BATT_VOLT_MULT`.
> Then compare mAh consumed against charger mAh replaced, and adjust `BATT_AMP_PERVLT`.

---

## Building ArduPilot

### Prerequisites

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Board Definition

Place this repository's hwdef files at:

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat
├── hwdef-bl.dat
├── defaults.parm
└── README.md
```

> The board name `NanoH743AIO` is a placeholder. Use the exact directory name from the hwdef.
> If the board needs an official `APJ_BOARD_ID` for upstream submission, request one through
> `Tools/AP_Bootloader/board_types.txt`.

### Build the Bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / _bl.hex
```

### Build Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # or: plane, rover, sub, heli
```

Artifacts are written to `build/NanoH743AIO/bin/`:

| File                         | Use                                         |
|------------------------------|---------------------------------------------|
| `arducopter.apj`             | Upload over existing ArduPilot bootloader   |
| `arducopter_with_bl.hex`     | First-time flash via DFU / STM32CubeProgrammer |

### Useful Build Options

```bash
./waf configure --board NanoH743AIO --debug        # debug symbols
./waf copter --upload                              # build + upload over USB
./waf configure --board NanoH743AIO --enable-scripting   # if not default
```

---

## Flashing

### Method A: First Flash / Recovery (DFU)

1. Hold the **BOOT** button (or bridge the BOOT pads) while plugging in USB.
2. Confirm that DFU mode is detected:
   ```bash
   dfu-util -l    # look for "STM32 BOOTLOADER" / 0483:df11
   ```
3. Flash using one of the following.
   - **STM32CubeProgrammer** (recommended): connect via USB and load `arducopter_with_bl.hex`, then press *Download*.
   - **dfu-util** (bootloader binary only):
     ```bash
     dfu-util -a 0 --dfuse-address 0x08000000:mass-erase:force -D NanoH743AIO_bl.bin
     ```
     Then install firmware using Method B.
4. Unplug and replug USB **without** holding BOOT.

> If the board previously ran Betaflight or INAV, a full mass-erase is recommended.

### Method B: Firmware Update (ArduPilot Bootloader Present)

- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* → select the `.apj` file.
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*.
- **CLI:**
  ```bash
  ./waf copter --upload
  # or
  python3 Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
  ```

### Post-Flash Checklist
- [ ] Accelerometer calibration
- [ ] Compass calibration (if an external compass is fitted)
- [ ] Radio calibration
- [ ] Motor order and direction test (*props off*)
- [ ] Battery monitor calibration
- [ ] Failsafe configuration

---

## Thermal Specifications

### Component Limits (Datasheet)

| Component      | Parameter                    | Limit                                   |
|----------------|------------------------------|-----------------------------------------|
| STM32H743 (suffix 6) | Ambient operating      | −40 °C to +85 °C                        |
| STM32H743      | Max junction (Tj)            | ⚠️ confirm per datasheet for exact part suffix |
| IMU            | Operating range              | ⚠️ TBD per IMU datasheet                 |
| ESC MOSFETs    | Max junction                 | ⚠️ TBD per MOSFET datasheet              |
| BEC regulator  | Thermal shutdown             | ⚠️ TBD                                   |

### Measured Performance (Cycle 13)

> ⚠️ Replace with real bench data, and record the test conditions: ambient temperature,
> airflow, input voltage, load, and measurement method (thermocouple or IR camera).

| Condition                          | MCU (°C) | ESC FETs (°C) | BEC (°C) | Notes |
|------------------------------------|----------|---------------|----------|-------|
| Idle on USB, 25 °C ambient, no airflow | ⚠️     | —             | ⚠️       |       |
| Armed, 4S, hover, prop wash        | ⚠️       | ⚠️            | ⚠️       |       |
| Sustained ⚠️ A per motor, 60 s      | ⚠️       | ⚠️            | ⚠️       |       |
| 5 V BEC at max rated load, 10 min  | ⚠️       | —             | ⚠️       |       |

### Thermal Guidance
- AIO boards rely on **prop wash for cooling**. Do not run the ESC at high throttle on the bench without airflow.
- Avoid fully enclosed mounting with no air path.
- If an IMU heater is configured (`BRD_HEAT_TARG`), document the default target here: ⚠️ TBD.
- Monitor the MCU temperature in logs. If it is consistently above ⚠️ TBD °C in flight, review mounting and airflow.

---

## Mounting & Dimensions

| Parameter        | Value          |
|------------------|----------------|
| Board outline    | ⚠️ TBD mm       |
| Hole pattern     | ⚠️ TBD mm       |
| Hole diameter    | ⚠️ TBD (M2/M3)  |
| Board thickness  | ⚠️ TBD mm       |
| Orientation      | Arrow forward; set `AHRS_ORIENTATION` if rotated |

Use soft-mount grommets. Keep the IMU away from motor wires and high-current traces.

---

## Troubleshooting

| Symptom                           | Likely Cause / Fix                                    |
|-----------------------------------|-------------------------------------------------------|
| No USB device                     | Charge-only cable; try another cable/port              |
| Stuck in DFU                      | No valid firmware; reflash `_with_bl.hex`              |
| "PreArm: Internal errors 0x…"     | Check hwdef, sensor detection, and logs                |
| Motors won't spin with DShot      | Mixed protocols within a timer group; see table above  |
| Battery voltage wrong             | Calibrate `BATT_VOLT_MULT`                             |
| Gyro/accel not detected           | Check IMU SPI pins and CS in the hwdef                 |

---

## Revision History

| Cycle | Date      | Changes                                  |
|-------|-----------|------------------------------------------|
|