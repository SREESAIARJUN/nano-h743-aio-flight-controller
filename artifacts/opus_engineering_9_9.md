# Note Before Use

I don't have access to the actual schematic, `hwdef.dat`, or test data for a board called **nano-h743-aio-flight-controller**, so I can't state its real pin assignments or measured thermal results. To avoid giving you fabricated hardware facts, this README uses two kinds of content:

- **Generic STM32H743 and ArduPilot content** (build system, flashing, MCU limits). This reflects documented, standard behavior.
- **Board-specific values**, marked `⚠️ VERIFY` or shown as `TBD`. Fill these in from your schematic, `hwdef.dat`, and bench tests before publishing.

---

````markdown
# Nano H743 AIO Flight Controller

> Compact all-in-one (AIO) flight controller based on the STM32H743, targeting ArduPilot (Copter / Plane / Rover / Sub).

![Board Top](docs/img/board-top.png)
![Board Bottom](docs/img/board-bottom.png)

| Status | Value |
|---|---|
| Hardware revision | `⚠️ VERIFY` (e.g., Rev C) |
| ArduPilot board name | `NanoH743AIO` `⚠️ VERIFY` |
| APJ Board ID | `⚠️ VERIFY` (must be registered in `Tools/AP_Bootloader/board_types.txt`) |
| License (hardware) | `⚠️ VERIFY` (e.g., CERN-OHL-S-2.0) |
| License (firmware) | GPLv3 (ArduPilot) |

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout & Pinout](#board-layout--pinout)
3. [UART / Serial Mapping](#uart--serial-mapping)
4. [PWM Outputs & Timer Groups](#pwm-outputs--timer-groups)
5. [Power, Voltage & Current Sensing](#power-voltage--current-sensing)
6. [Building ArduPilot](#building-arduplot)
7. [Flashing](#flashing)
8. [Default Parameters](#default-parameters)
9. [Thermal Specifications](#thermal-specifications)
10. [Mechanical](#mechanical)
11. [Troubleshooting](#troubleshooting)
12. [Contributing](#contributing)
13. [Revision History](#revision-history)

---

## Specifications

| Category | Detail |
|---|---|
| MCU | STM32H743VIT6 / VIH6 `⚠️ VERIFY package`, Cortex-M7 @ 480 MHz, 2 MB flash, 1 MB RAM |
| IMU(s) | `⚠️ VERIFY` (e.g., ICM-42688-P on SPI1, BMI270 on SPI4) |
| Barometer | `⚠️ VERIFY` (e.g., DPS310 / SPL06 on I2C2) |
| OSD | `⚠️ VERIFY` (e.g., AT7456E on SPI2) |
| Blackbox / logging | `⚠️ VERIFY` (microSD on SDMMC1 **or** SPI flash) |
| Integrated ESC | `⚠️ VERIFY` (e.g., 4-in-1, BLHeli_S / AM32, xx A continuous / xx A burst) |
| Input voltage | `⚠️ VERIFY` (e.g., 2S–6S LiPo) |
| BEC outputs | `⚠️ VERIFY` (e.g., 5 V / 2 A, 10 V / 2 A switchable VTX rail) |
| UARTs | `⚠️ VERIFY` count |
| PWM outputs | `⚠️ VERIFY` count |
| I2C | `⚠️ VERIFY` (external compass / airspeed) |
| CAN | `⚠️ VERIFY` (FDCAN1 if broken out) |
| USB | USB-C, OTG FS |
| Mounting | `⚠️ VERIFY` (e.g., 25.5 × 25.5 mm, M2) |
| Weight | `⚠️ VERIFY` g |

---

## Board Layout & Pinout

> **The pin assignments below are placeholders.** Replace them with the values from the
> board's `hwdef.dat` and schematic. Every row must match the hwdef exactly.

### Solder Pad Map

```
             ┌────────────────────────────────────────┐
             │  [USB-C]                     [BOOT]    │
   M1 ●──────┤                                        ├──────● M2
             │   5V  GND  RX1  TX1   SDA  SCL  5V GND │
             │                                        │
             │            STM32H743                   │
             │                                        │
             │   VBAT GND  CUR  RSSI  LED  BZ+  BZ-   │
   M4 ●──────┤                                        ├──────● M3
             │  RX2 TX2  RX6 TX6   10V  GND  VTX  CAM │
             └────────────────────────────────────────┘
                    ⚠️ VERIFY: illustrative only
```

### MCU Pin Assignments

| Function | MCU Pin | Peripheral | Notes |
|---|---|---|---|
| IMU1 SCK / MISO / MOSI | `TBD` | SPI1 | |
| IMU1 CS | `TBD` | GPIO | |
| IMU1 DRDY | `TBD` | EXTI | Optional |
| OSD CS | `TBD` | SPI2 | |
| Baro SDA / SCL | `TBD` | I2C2 | Internal bus |
| Ext. I2C SDA / SCL | `TBD` | I2C1 | Compass, airspeed |
| VBAT sense | `TBD` | ADC | See scaling below |
| Current sense | `TBD` | ADC | |
| RSSI analog | `TBD` | ADC | |
| LED (status) | `TBD` | GPIO | |
| Buzzer | `TBD` | GPIO / TIM | |
| VTX power switch | `TBD` | GPIO (RELAY) | |
| Camera switch | `TBD` | GPIO (RELAY) | |
| WS2812 LED strip | `TBD` | TIMx_CHy | Uses DShot-capable timer |

---

## UART / Serial Mapping

| ArduPilot SERIALn | MCU UART | Pads | Default Protocol | DMA |
|---|---|---|---|---|
| SERIAL0 | OTG1 | USB-C | MAVLink2 | — |
| SERIAL1 | `TBD` | TX1 / RX1 | MAVLink2 (Telem) | `TBD` |
| SERIAL2 | `TBD` | TX2 / RX2 | MAVLink2 (Telem) | `TBD` |
| SERIAL3 | `TBD` | TX3 / RX3 | GPS | `TBD` |
| SERIAL4 | `TBD` | TX4 / RX4 | GPS2 | `TBD` |
| SERIAL5 | `TBD` | RX5 | ESC Telemetry (16) | `TBD` |
| SERIAL6 | `TBD` | TX6 / RX6 | RCIN (23) | `TBD` |
| SERIAL7 | OTG2 | USB (virtual) | SLCAN / MAVLink | — |

**Notes:**
- RC input: any UART can take RC with `SERIALn_PROTOCOL = 23`. Full-duplex protocols (CRSF/ELRS, FPort) need both TX and RX.
- Use `SERIALn_OPTIONS` for inversion and half-duplex where the hardware supports it.

---

## PWM Outputs & Timer Groups

Outputs that share a timer **must use the same protocol and rate** (e.g., all DShot600 or all PWM 50 Hz).

| Output | MCU Pin | Timer / Channel | Group | DShot | Bidir DShot |
|---|---|---|---|---|---|
| M1 | `TBD` | `TBD` | 1 | ✅ | `⚠️ VERIFY` |
| M2 | `TBD` | `TBD` | 1 | ✅ | `⚠️ VERIFY` |
| M3 | `TBD` | `TBD` | 1 | ✅ | `⚠️ VERIFY` |
| M4 | `TBD` | `TBD` | 1 | ✅ | `⚠️ VERIFY` |
| S5 | `TBD` | `TBD` | 2 | ✅ | — |
| S6 | `TBD` | `TBD` | 2 | ✅ | — |
| LED | `TBD` | `TBD` | 3 | Serial LED | — |

**Notes:**
- Bidirectional DShot needs DMA-capable input capture on each channel. Confirm `BIDIR` flags in `hwdef.dat`.
- Set the output protocol with `MOT_PWM_TYPE` (Copter) or `Q_M_PWM_TYPE` (QuadPlane).

---

## Power, Voltage & Current Sensing

| Parameter | Default | Notes |
|---|---|---|
| `BATT_MONITOR` | `4` | Analog voltage and current |
| `BATT_VOLT_PIN` | `TBD` | ADC channel |
| `BATT_CURR_PIN` | `TBD` | ADC channel |
| `BATT_VOLT_MULT` | `TBD` | e.g., `11.0` for a 10k:1k divider. `⚠️ VERIFY` |
| `BATT_AMP_PERVLT` | `TBD` | Calibrate against a reference meter |
| `BATT_AMP_OFFSET` | `TBD` | |

**Calibrating current sensing:**
1. Fly or bench-run a full pack.
2. Compare the mAh reported by ArduPilot with the mAh your charger puts back in.
3. Calculate the new value: `new_AMP_PERVLT = old × (charger_mAh / logged_mAh)`.

---

## Building ArduPilot

### Prerequisites

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y   # Ubuntu / WSL2
. ~/.profile
```

Use the `arm-none-eabi-gcc` version pinned by ArduPilot for your branch. Run `./waf --help` and see the ArduPilot dev docs for details.

### Adding the Board Definition

```
libraries/AP_HAL_ChibiOS/hwdef/NanoH743AIO/
├── hwdef.dat          # Main firmware definition
├── hwdef-bl.dat       # Bootloader definition
├── defaults.parm      # Board default parameters
└── README.md          # Copy of this file (required for upstream PR)
```

Register the board ID in `Tools/AP_Bootloader/board_types.txt` before submitting upstream.

### Build Firmware

```bash
./waf configure --board NanoH743AIO
./waf copter        # ArduCopter
./waf plane         # ArduPlane
./waf rover         # Rover
./waf sub           # ArduSub
```

Output files are written to `build/NanoH743AIO/bin/`:

| File | Purpose |
|---|---|
| `arducopter.apj` | Upload through an existing ArduPilot bootloader (Mission Planner / QGC / uploader.py) |
| `arducopter_with_bl.hex` | Full image with bootloader, for DFU / STM32CubeProgrammer |
| `arducopter.bin` | Raw application binary |

### Build the Bootloader

```bash
Tools/scripts/build_bootloaders.py NanoH743AIO
# Output: Tools/bootloaders/NanoH743AIO_bl.bin / .hex
```

### Clean Rebuild

```bash
./waf distclean
./waf configure --board NanoH743AIO
./waf copter
```

---

## Flashing

### Method A: Existing ArduPilot Bootloader (Normal Updates)

**Option 1: From the build tree**

```bash
./waf copter --upload
```

**Option 2: Standalone uploader**

```bash
python3 Tools/scripts/uploader.py build/NanoH743AIO/bin/arducopter.apj
```

**Option 3: GCS**
- **Mission Planner:** *Setup → Install Firmware → Load custom firmware* → select the `.apj` file.
- **QGroundControl:** *Vehicle Setup → Firmware → Advanced → Custom firmware file*.

### Method B: First-Time Flash / Recovery via DFU

1. Hold the **BOOT** button.
2. Connect USB-C.
3. Release BOOT.
4. Confirm the board is in DFU mode:

   ```bash
   dfu-util -l    # Should list "STM32 BOOTLOADER" (0483:df11)
   ```

**Flash using STM32CubeProgrammer CLI (recommended, accepts .hex):**

```bash
STM32_Programmer_CLI -c port=usb1 -e all
STM32_Programmer_CLI -c port=usb1 -w build/NanoH743AIO/bin/arducopter_with_bl.hex -v -rst
```

**Flash using dfu-util (bootloader only, then use Method A):**

```bash
dfu-util -a 0 --dfuse-address 0x08000000:leave -D Tools/bootloaders/NanoH743AIO_bl.bin
```

**Linux note:** Without proper permissions, DFU access fails. Add a udev rule:

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="0666"
```

**Windows note:** Use Zadig to bind the WinUSB driver to "STM32 BOOTLOADER" if `dfu-util` can't see the device.

### Migrating from Betaflight / INAV

1. Back up your Betaflight/INAV configuration with `diff all`.
2. Flash `arducopter_with_bl.hex` via DFU (Method B).
3. Load `defaults.parm` and reconfigure. Betaflight/INAV settings do **not** carry over.

---

## Default Parameters

The board's `defaults.parm` should set, at minimum:

```
SERIAL6_PROTOCOL 23
MOT_PWM_TYPE 6            # DShot600  ⚠️ VERIFY
SERVO_BLH_AUTO 1
BATT_MONITOR 4
BATT_VOLT_MULT TBD
BATT_AMP_PERVLT TBD
OSD_TYPE 1                # ⚠️ VERIFY (only if analog OSD chip fitted)
NTF_LED_TYPES TBD
```

---

## Thermal Specifications

### Component Limits (Datasheet Values)

| Component | Parameter | Limit | Source |
|---|---|---|---|
| STM32H743 (suffix 6) | Ambient operating | −40 °C to +85 °C | ST datasheet DS12110 |
| STM32H743 (suffix 3) | Ambient operating | −40 °C to +125 °C | ST datasheet. `⚠️ VERIFY` the fitted part |
| STM32H743 | Max junction temp (Tj) | Per suffix; see datasheet | DS12110 |
| IMU | Operating | `⚠️ VERIFY` (typ. −40 °C to +85 °C) | IMU datasheet |
| ESC MOSFETs | Max junction | `⚠️ VERIFY` (typ. 150–175 °C) | MOSFET datasheet |
| LiPo pack | Recommended | 0 °