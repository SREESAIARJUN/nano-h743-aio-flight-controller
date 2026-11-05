# README.md: nano-h743-aio-flight-controller

> **Note before you commit this:** I don't have access to your repository, schematic, or `hwdef.dat`, so I can't know your board's actual pin assignments, regulator ratings, or thermal test results. Rather than invent numbers that look authoritative, I've marked every hardware-specific value with **`⟨VERIFY⟩`** or **`TBD`**. The ArduPilot build, flash, and parameter procedures are standard and accurate as written. If you paste your `hwdef.dat`, `hwdef-bl.dat`, and schematic net list, I can fill in the tables exactly.

---

```markdown
# Nano H743 AIO Flight Controller

![Board top](docs/img/board_top.png)

A compact all-in-one (AIO) flight controller built on the STM32H743 MCU. It integrates
⟨VERIFY: 4-in-1 ESC / ELRS receiver / OSD / none⟩ and targets ArduPilot
(Copter, Plane, Rover). It is aimed at 2"–3.5" ⟨VERIFY⟩ multirotors and small fixed-wing airframes.

[![License](https://img.shields.io/badge/license-⟨VERIFY⟩-blue.svg)](LICENSE)
[![ArduPilot](https://img.shields.io/badge/firmware-ArduPilot-green.svg)](https://ardupilot.org)

---

## Table of Contents
1. [Specifications](#specifications)
2. [Board Layout & Pinout](#board-layout--pinout)
3. [UART / Serial Mapping](#uart--serial-mapping)
4. [PWM Outputs & Timer Groups](#pwm-outputs--timer-groups)
5. [Building ArduPilot](#building-ardupilot)
6. [Flashing](#flashing)
7. [Initial Configuration](#initial-configuration)
8. [Thermal Specifications](#thermal-specifications)
9. [Electrical Limits](#electrical-limits)
10. [Troubleshooting](#troubleshooting)
11. [Revision History](#revision-history)
12. [License](#license)

---

## Specifications

| Item | Value |
|---|---|
| MCU | STM32H743VIT6, Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM |
| IMU | ⟨VERIFY: e.g. ICM-42688-P⟩ (SPI) |
| Barometer | ⟨VERIFY: e.g. DPS310 / BMP280⟩ (I²C/SPI) |
| OSD | ⟨VERIFY: AT7456E analog / none⟩ |
| Blackbox / logging | ⟨VERIFY: microSD / onboard flash, size⟩ |
| Integrated ESC | ⟨VERIFY: 4-in-1, xx A continuous / xx A burst, BLHeli_S/AM32⟩ |
| Input voltage | ⟨VERIFY⟩ S LiPo (⟨VERIFY⟩ V – ⟨VERIFY⟩ V) |
| BECs | 5 V @ ⟨VERIFY⟩ A, 9/10 V @ ⟨VERIFY⟩ A (VTX), 3.3 V @ ⟨VERIFY⟩ mA |
| Current sensor | ⟨VERIFY⟩ (scale: `BATT_AMP_PERVLT` = ⟨VERIFY⟩) |
| Voltage divider | `BATT_VOLT_MULT` = ⟨VERIFY⟩ |
| UARTs | ⟨VERIFY⟩ |
| Mounting | ⟨VERIFY: 25.5 × 25.5 mm, M2⟩ |
| Dimensions | ⟨VERIFY⟩ mm |
| Weight | ⟨VERIFY⟩ g |
| ArduPilot board ID | `APJ_BOARD_ID` ⟨VERIFY⟩ |

---

## Board Layout & Pinout

![Top pinout](docs/img/pinout_top.png)
![Bottom pinout](docs/img/pinout_bottom.png)

### Solder Pads / Connectors

| Pad / Pin | Function | MCU Pin | Notes |
|---|---|---|---|
| VBAT | Battery + | — | Via ESC power input |
| GND | Ground | — | |
| 5V | 5 V BEC output | — | ⟨VERIFY⟩ A max |
| 9V / 10V | VTX supply | — | Switchable via `RELAY` pin ⟨VERIFY⟩ |
| 3V3 | 3.3 V output | — | Low current only |
| T1 / R1 | UART1 | ⟨VERIFY: PA9 / PA10⟩ | |
| T2 / R2 | UART2 | ⟨VERIFY⟩ | |
| SDA / SCL | I²C | ⟨VERIFY⟩ | External compass |
| LED | WS2812 data | ⟨VERIFY⟩ | |
| BZ+ / BZ− | Buzzer | ⟨VERIFY⟩ | |
| CUR | Current sense in | ⟨VERIFY⟩ | ADC |
| RSSI | Analog RSSI | ⟨VERIFY⟩ | ADC |
| BOOT | DFU boot | BOOT0 | Bridge to 3V3 or hold button at power-on |

> Generate this table directly from `hwdef.dat` so it cannot drift from the firmware.
> See [`tools/hwdef_to_md.py`](tools/hwdef_to_md.py) ⟨optional helper⟩.

---

## UART / Serial Mapping

| ArduPilot Port | UART | Default Protocol | `SERIALn_PROTOCOL` | Notes |
|---|---|---|---|---|
| SERIAL0 | USB | MAVLink2 | 2 | |
| SERIAL1 | ⟨VERIFY⟩ | MAVLink2 | 2 | Telemetry |
| SERIAL2 | ⟨VERIFY⟩ | MAVLink2 | 2 | |
| SERIAL3 | ⟨VERIFY⟩ | GPS | 5 | |
| SERIAL4 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | ⟨VERIFY⟩ | |
| SERIAL5 | ⟨VERIFY⟩ | ESC Telemetry | 16 | ⟨VERIFY: AIO ESC telem line⟩ |
| SERIAL6 | ⟨VERIFY⟩ | RCIN | 23 | Onboard / external receiver |

The order must match the `SERIAL_ORDER` line in `hwdef.dat`.

---

## PWM Outputs & Timer Groups

| Output | MCU Pin | Timer | Group | DShot | BiDir DShot |
|---|---|---|---|---|---|
| M1 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | 1 | ✔ | ✔ |
| M2 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | 1 | ✔ | ✔ |
| M3 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | 2 | ✔ | ✔ |
| M4 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | 2 | ✔ | ✔ |
| S5 | ⟨VERIFY⟩ | ⟨VERIFY⟩ | 3 | ✔ | — |
| LED | ⟨VERIFY⟩ | ⟨VERIFY⟩ | 4 | — | — |

**All outputs in the same timer group must use the same protocol and rate.**
For example, you cannot mix DShot and PWM servos within one group.

---

## Building ArduPilot

### 1. Toolchain setup (Ubuntu 22.04 / WSL2)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### 2. Add the board definition

```bash
cp -r <this-repo>/hwdef/NanoH743AIO libraries/AP_HAL_ChibiOS/hwdef/
# Expected files:
#   hwdef.dat       – main firmware hardware definition
#   hwdef-bl.dat    – bootloader definition
#   defaults.parm   – board default parameters
#   README.md       – board doc (ArduPilot wiki format)
```

### 3. Build the bootloader (first time only)

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

### 4. Build firmware

```bash
./waf configure --board NanoH743AIO
./waf copter          # or: plane | rover | antennatracker
```

Artifacts are written to `build/NanoH743AIO/bin/`:

| File | Use |
|---|---|
| `arducopter.apj` | Load via Mission Planner / QGC (bootloader already present) |
| `arducopter_with_bl.hex` | Full image (bootloader + firmware) for DFU / first flash |

### 5. Optional: build and upload in one step

```bash
./waf copter --upload
```

---

## Flashing

### A. First flash via DFU (blank board or bootloader recovery)

1. Hold **BOOT** (or bridge the BOOT pads) while plugging in USB.
2. Verify that DFU mode is detected:
   ```bash
   dfu-util -l     # expect "STM32 BOOTLOADER" / 0483:df11
   ```
3. Flash the full image with **one** of these tools:
   - **STM32CubeProgrammer (recommended):**
     ```bash
     STM32_Programmer_CLI -c port=USB1 -w arducopter_with_bl.hex -v -rst
     ```
   - **dfu-util (`.bin` only, address 0x08000000):**
     ```bash
     arm-none-eabi-objcopy -I ihex -O binary arducopter_with_bl.hex fw.bin
     dfu-util -a 0 -s 0x08000000:leave -D fw.bin
     ```
   - **Betaflight Configurator / INAV firmware flasher:** use "Load Firmware [Local]" with the `.hex` file and full chip erase.

> ⚠ The `.apj` file **cannot** be flashed via DFU. Use `_with_bl.hex`.

### B. Updating an existing ArduPilot install

- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* → select `.apj`
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*
- **CLI:**
  ```bash
  Tools/scripts/uploader.py --port /dev/ttyACM0 build/NanoH743AIO/bin/arducopter.apj
  ```

### C. Returning to Betaflight/INAV

Enter DFU mode as in section A and flash the target firmware with full chip erase.

### D. Linux udev (if DFU is not detected)

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666"' \
 | sudo tee /etc/udev/rules.d/49-stm32dfu.rules && sudo udevadm control --reload
```

---

## Initial Configuration

Recommended starting parameters (also shipped in `defaults.parm`):

```text
BATT_MONITOR      4
BATT_VOLT_PIN     ⟨VERIFY⟩
BATT_CURR_PIN     ⟨VERIFY⟩
BATT_VOLT_MULT    ⟨VERIFY⟩
BATT_AMP_PERVLT   ⟨VERIFY⟩
MOT_PWM_TYPE      6        # DShot600
SERVO_BLH_AUTO    1
SERVO_BLH_BDMASK  15       # BiDir DShot on M1–M4 (if ESC supports it)
SERVO_DSHOT_ESC   ⟨VERIFY: 1 BLHeli32/AM32, 2 BLHeli_S⟩
OSD_TYPE          1        # if analog OSD fitted
NTF_LED_TYPES     ⟨VERIFY⟩
```

After flashing:
1. Calibrate the accelerometer, then the compass if one is fitted.
2. Calibrate voltage and current against a multimeter and a known load.
3. Run *Motor Test* **with props removed** and confirm the order and direction.
4. Enable the harmonic notch filter (`INS_HNTCH_ENABLE=1`, `INS_HNTCH_MODE=3` with BiDir DShot).

---

## Thermal Specifications

> Values below marked TBD must come from your own bench and flight testing.
> Do not publish datasheet absolute-maximum figures as operating ratings.

### Component limits (from datasheets)

| Component | Parameter | Limit | Source |
|---|---|---|---|
| STM32H743VIT6 | Ambient (suffix "6") | −40 °C to +85 °C | ST datasheet DS12110 |
| STM32H743VIT6 | Junction max | ⟨VERIFY in DS12110 for your temp grade⟩ | ST datasheet |
| IMU | Operating | ⟨VERIFY⟩ | Vendor datasheet |
| ESC MOSFETs | Tj max | ⟨VERIFY, typically 150 °C⟩ | MOSFET datasheet |
| BEC regulator | Thermal shutdown | ⟨VERIFY⟩ | Regulator datasheet |

### Measured board performance

| Condition | MCU Temp | ESC FET Temp | BEC Temp | Notes |
|---|---|---|---|---|
| Idle on bench, USB only, 25 °C ambient | TBD | — | TBD | |
| Idle on bench, 4S, no airflow | TBD | TBD | TBD | VTX on/off? |
| Hover, ⟨VERIFY⟩ A total | TBD | TBD | TBD | |
| Full-throttle 10 s burst | TBD | TBD | TBD | |
| Sustained ⟨VERIFY⟩ A, 60 s | TBD | TBD | TBD | |

**Test method:** ⟨describe the thermocouple/IR camera, ambient temperature, airflow, and duration⟩.

### Guidance

- Ratings for the integrated ESC assume **prop-wash airflow**. On the bench, do not run motors loaded without airflow.
- Monitor MCU temperature in flight via the `MCU_TEMP` field in the `POWR` log.
- IMU temperature drift: consider enabling `INS_TCAL1_ENABLE` (temperature calibration) for consistent EKF performance.
- Mounting the VTX directly above the FC will raise board temperature. Leave clearance or use thermal isolation.
- **Derating:** ⟨VERIFY: e.g. reduce continuous current by X % above Y °C ambient⟩.

---

## Electrical Limits

| Rail | Absolute Max | Recommended Max Continuous |
|---|---|---|
| VBAT | ⟨VERIFY⟩ V | ⟨VERIFY⟩ V |
| 5 V BEC | ⟨VERIFY⟩ A