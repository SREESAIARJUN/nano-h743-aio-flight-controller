# README.md

> **Before you publish:** I don't have your schematic, BOM, `hwdef.dat`, or thermal test data, so I couldn't take hardware-specific values from them. Every value that depends on your design is marked **⚠️ VERIFY**. The pinout follows a common STM32H743 assignment pattern but is **illustrative**. Wrong pinouts or current ratings in a published README can destroy hardware, so reconcile every marked value against your schematic and test data before merging. The ArduPilot build, bootloader, and flashing commands are standard and accurate as written.

---

```markdown
# Nano H743 AIO Flight Controller

<p align="center">
  <img src="docs/img/board-top.png" alt="Nano H743 AIO – top" width="45%">
  <img src="docs/img/board-bottom.png" alt="Nano H743 AIO – bottom" width="45%">
</p>

<p align="center">
  <a href="#"><img src="https://img.shields.io/badge/MCU-STM32H743-blue"></a>
  <a href="#"><img src="https://img.shields.io/badge/firmware-ArduPilot-green"></a>
  <a href="#"><img src="https://img.shields.io/badge/hardware%20rev-7-orange"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-CERN--OHL--S--2.0%20%2F%20GPLv3-lightgrey"></a>
</p>

A compact all-in-one (AIO) flight controller with an integrated 4-in-1 ESC, built around
the STM32H743 (480 MHz Cortex-M7). It is designed for 2"–3.5" ArduPilot multirotors where
weight, wiring simplicity, and processing headroom all matter.

---

## Table of Contents

1. [Specifications](#specifications)
2. [Board Layout](#board-layout)
3. [Pinout](#pinout)
4. [UART / Serial Mapping](#uart--serial-mapping)
5. [PWM / Motor Outputs](#pwm--motor-outputs)
6. [Power & Analog](#power--analog)
7. [Thermal Specifications](#thermal-specifications)
8. [Building ArduPilot](#building-ardupilot)
9. [Flashing](#flashing)
10. [First-Time Setup](#first-time-setup)
11. [Troubleshooting](#troubleshooting)
12. [Repository Structure](#repository-structure)
13. [Revision History](#revision-history)
14. [Contributing](#contributing)
15. [License](#license)
16. [Safety Notice](#safety-notice)

---

## Specifications

### Flight Controller

| Parameter            | Value                                              |
|----------------------|----------------------------------------------------|
| MCU                  | STM32H743VIT6, 480 MHz, 2 MB flash, 1 MB RAM        |
| IMU                  | ⚠️ VERIFY (e.g. ICM-42688-P on SPI1)                |
| Barometer            | ⚠️ VERIFY (e.g. DPS310 / BMP280 on I2C)             |
| OSD                  | ⚠️ VERIFY (AT7456E on SPI2, or none for HD/MSP)     |
| Blackbox / Logging   | ⚠️ VERIFY (microSD via SDMMC1, or SPI flash)        |
| USB                  | USB-C, USB 2.0 Full Speed                           |
| UARTs                | 7 (see [mapping](#uart--serial-mapping))            |
| I2C                  | 1 (shared: baro, external compass)                  |
| CAN                  | ⚠️ VERIFY (none / 1× FDCAN)                         |
| PWM outputs          | 4 internal ESC + ⚠️ VERIFY auxiliary                |
| BEC outputs          | 5 V @ ⚠️ VERIFY A, 10 V @ ⚠️ VERIFY A (VTX)          |

### Integrated ESC

| Parameter            | Value                                              |
|----------------------|----------------------------------------------------|
| Input voltage        | 2S–4S LiPo (7.4–16.8 V) ⚠️ VERIFY                   |
| Continuous current   | ⚠️ VERIFY A per motor                               |
| Burst current        | ⚠️ VERIFY A per motor, ≤ 10 s                       |
| ESC firmware         | ⚠️ VERIFY (BLHeli_S / Bluejay / AM32)               |
| Protocol             | DShot300/600, Bidirectional DShot (firmware-dependent) |
| Current sensor       | Onboard shunt, scale ⚠️ VERIFY A/V                  |

### Mechanical

| Parameter            | Value                                              |
|----------------------|----------------------------------------------------|
| Mounting pattern     | 25.5 × 25.5 mm, M2 ⚠️ VERIFY                        |
| Board dimensions     | ⚠️ VERIFY mm                                        |
| Weight               | ⚠️ VERIFY g (without wires)                         |
| PCB                  | ⚠️ VERIFY layers, ⚠️ VERIFY oz copper               |

---

## Board Layout

```
            ┌─────────────────────────────────────┐
            │  M4 ●                         ● M2  │
            │                                     │
            │   [T1][R1]   [ BOOT ]   [T2][R2]    │
            │                                     │
            │        ┌───────────────────┐        │
            │  5V ●  │    STM32H743      │  ● GND │
            │        │                   │        │
            │  SDA ● │    ▲ FRONT        │  ● SCL │
            │        └───────────────────┘        │
            │   [T6][R6]   [ USB-C ]  [VTX][10V]  │
            │                                     │
            │  M3 ●      [ BAT+ ][ BAT- ]   ● M1  │
            └─────────────────────────────────────┘
```

> The diagram is schematic, not to scale. Rendered top/bottom pad drawings are in
> [`docs/img/`](docs/img). The IMU arrow defines the **forward** direction. The default
> `AHRS_ORIENTATION` is `0` (None) when the arrow faces the nose.

---

## Pinout

### MCU Pin Assignment

> ⚠️ **VERIFY:** this table must match `hwdef/NanoH743AIO/hwdef.dat` exactly. That file
> is the source of truth.

| Function        | MCU Pin | Peripheral       | Notes                          |
|-----------------|---------|------------------|--------------------------------|
| IMU SCK         | PA5     | SPI1_SCK         |                                |
| IMU MISO        | PA6     | SPI1_MISO        |                                |
| IMU MOSI        | PD7     | SPI1_MOSI        |                                |
| IMU CS          | PC15    | GPIO             | Active low                     |
| IMU INT/DRDY    | PB2     | GPIO             | Optional                       |
| OSD SCK         | PB13    | SPI2_SCK         | If analog OSD fitted           |
| OSD MISO        | PB14    | SPI2_MISO        |                                |
| OSD MOSI        | PB15    | SPI2_MOSI        |                                |
| OSD CS          | PB12    | GPIO             |                                |
| I2C1 SCL        | PB6     | I2C1_SCL         | Baro + external compass        |
| I2C1 SDA        | PB7     | I2C1_SDA         |                                |
| USB D−          | PA11    | OTG_FS_DM        |                                |
| USB D+          | PA12    | OTG_FS_DP        |                                |
| VBAT sense      | PC0     | ADC1_INP10       | Divider ⚠️ VERIFY              |
| Current sense   | PC1     | ADC1_INP11       | Scale ⚠️ VERIFY                |
| RSSI (analog)   | PC5     | ADC1_INP8        | Optional                       |
| LED (status)    | PE3     | GPIO             | Active low                     |
| Buzzer          | PA15    | TIM2_CH1         |                                |
| LED strip       | PA8     | TIM1_CH1         | WS2812                         |
| BOOT0           | —       | Button           | Hold while plugging in = DFU   |

### Solder Pad Map

| Pad    | Signal        | Voltage    | Max Current          |
|--------|---------------|------------|----------------------|
| BAT+   | Battery in    | 2S–4S      | ESC rating           |
| 5V     | BEC output    | 5.0 V      | ⚠️ VERIFY A (shared) |
| 10V    | VTX BEC       | 10.0 V     | ⚠️ VERIFY A          |
| 3V3    | LDO output    | 3.3 V      | ⚠️ VERIFY mA         |
| GND    | Ground        | —          | —                    |
| T1/R1  | UART1         | 3.3 V      | —                    |
| T2/R2  | UART2         | 3.3 V      | —                    |
| ...    | ...           | ...        | —                    |

> All UART and GPIO pads are **3.3 V logic and are not 5 V tolerant** unless they are marked
> FT in the STM32H743 datasheet *and* your schematic has no series protection that changes
> this. Never apply battery voltage to a signal pad.

---

## UART / Serial Mapping

| ArduPilot `SERIALn` | UART   | TX    | RX    | Default Protocol        | Pads   |
|---------------------|--------|-------|-------|-------------------------|--------|
| SERIAL0             | USB    | —     | —     | MAVLink2                | USB-C  |
| SERIAL1             | USART1 | PA9   | PA10  | MAVLink2 (telemetry)    | T1/R1  |
| SERIAL2             | USART2 | PD5   | PD6   | RCIN (CRSF/ELRS)        | T2/R2  |
| SERIAL3             | USART3 | PD8   | PD9   | GPS                     | T3/R3  |
| SERIAL4             | UART4  | PB9   | PB8   | MSP DisplayPort (HD VTX)| T4/R4  |
| SERIAL5             | USART6 | PC6   | PC7   | ESC Telemetry           | R6     |
| SERIAL6             | UART7  | PE8   | PE7   | Spare                   | T7/R7  |
| SERIAL7             | UART8  | PE1   | PE0   | Spare                   | T8/R8  |

All UARTs support DMA. Protocols are set by `SERIALn_PROTOCOL` and can be changed freely.

**Receiver note:** for CRSF/ELRS on SERIAL2, set:

```
SERIAL2_PROTOCOL = 23   # RCIN
RSSI_TYPE        = 3    # Receiver protocol
```

---

## PWM / Motor Outputs

| Output | MCU Pin | Timer     | DMA | Bi-dir DShot | Group | Function   |
|--------|---------|-----------|-----|--------------|-------|------------|
| M1     | PB0     | TIM3_CH3  | Yes | Yes          | 1     | Motor 1    |
| M2     | PB1     | TIM3_CH4  | Yes | Yes          | 1     | Motor 2    |
| M3     | PA0     | TIM5_CH1  | Yes | Yes          | 2     | Motor 3    |
| M4     | PA1     | TIM5_CH2  | Yes | Yes          | 2     | Motor 4    |
| S5     | PA2     | TIM5_CH3  | Yes | Yes          | 2     | Aux / servo|
| S6     | PA3     | TIM5_CH4  | Yes | Yes          | 2     | Aux / servo|
| LED    | PA8     | TIM1_CH1  | Yes | —            | 3     | NeoPixel   |

> Outputs in the same **group** share a timer and must use the same protocol and rate.
> You cannot mix DShot and PWM inside one group.

Motor order follows the ArduPilot QUAD/X convention:

```
   M3(CW)    M1(CCW)
        \  /
         ><      ▲ front
        /  \
   M2(CCW)   M4(CW)
```

⚠️ VERIFY that the physical motor pad labels match this numbering. If they don't, remap
with `SERVOn_FUNCTION` rather than swapping wires.

---

## Power & Analog

Default battery monitor parameters (compiled into `hwdef.dat`):

| Parameter            | Default        | Notes                              |
|----------------------|----------------|------------------------------------|
| `BATT_MONITOR`       | 4              | Analog voltage + current           |
| `BATT_VOLT_PIN`      | 10             | PC0                                |
| `BATT_CURR_PIN`      | 11             | PC1                                |
| `BATT_VOLT_MULT`     | ⚠️ VERIFY      | = (R1 + R2) / R2 of divider         |
| `BATT_AMP_PERVLT`    | ⚠️ VERIFY      | Calibrate against a reference meter |
| `BATT_AMP_OFFSET`    | ⚠️ VERIFY      |                                    |

**Calibration procedure:**

1. Measure the pack with a multimeter. Adjust `BATT_VOLT_MULT` until the GCS reading matches within 0.05 V.
2. Fly a full pack. Compare mAh consumed against charger mAh replaced:
   `new_AMP_PERVLT = old × (charger_mAh / reported_mAh)`.

---

## Thermal Specifications

> ⚠️ **VERIFY all measured values.** The rows marked *measured* must come from your own
> thermal testing. The silicon limits come from manufacturer datasheets; confirm them against
> the exact part numbers and temperature grades on your BOM.

### Component Limits

| Component            | Parameter               | Limit            | Source                    |
|----------------------|-------------------------|------------------|---------------------------|
| STM32H743 (grade 6)  | Ambient operating range | −40 to +85 °C    | ST datasheet DS12110      |
| STM32H743            | Max junction temp       | ⚠️ VERIFY (grade-dependent) | ST datasheet DS12110 |
| IMU                  | Operating range         | −40 to +85 °C    | IMU datasheet ⚠️ VERIFY   |
| ESC MOSFETs          | Max junction temp       | ⚠️ VERIFY (typ