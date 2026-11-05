I don't have access to the contents of `osmo-pocket-esp32-gimbal`, so I wrote this from a reference design: an ESP32, three BLDC gimbal motors, SimpleFOC, AS5048A encoders and an ICM-42688-P IMU. Replace values marked **⚠ VERIFY** with your actual hardware and firmware values before merging.

---

````markdown
# osmo-pocket-esp32-gimbal

> An open-source, 3-axis handheld stabilizer in the style of a pocket gimbal. It uses an ESP32 to run Field-Oriented Control (FOC) on three BLDC gimbal motors, with absolute magnetic encoders and a 6-DoF IMU.

![build](https://img.shields.io/badge/build-PlatformIO-orange)
![mcu](https://img.shields.io/badge/MCU-ESP32--WROOM--32E-blue)
![foc](https://img.shields.io/badge/FOC-SimpleFOC%202.3.x-green)
![license](https://img.shields.io/badge/license-MIT-lightgrey)

**Disclaimer:** This is an independent hobby project. It is not affiliated with, endorsed by, or derived from DJI. "Osmo Pocket" is a trademark of SZ DJI Technology Co., Ltd. It is used here only to describe the form factor.

---

## Table of Contents

1. [Features](#features)
2. [System Architecture](#system-architecture)
3. [Hardware Overview](#hardware-overview)
4. [Wiring & Pinouts](#wiring--pinouts)
5. [Power System](#power-system)
6. [Firmware Build & Flash](#firmware-build--flash)
7. [First Power-Up & Calibration](#first-power-up--calibration)
8. [FOC Tuning Procedure](#foc-tuning-procedure)
9. [Stabilization Loop Tuning](#stabilization-loop-tuning)
10. [Operating Modes & Controls](#operating-modes--controls)
11. [Serial Command Reference](#serial-command-reference)
12. [Bill of Materials](#bill-of-materials)
13. [Troubleshooting](#troubleshooting)
14. [Safety](#safety)
15. [Repository Layout](#repository-layout)
16. [Contributing](#contributing)
17. [License](#license)

---

## Features

- **3-axis stabilization** (pan / roll / tilt) with closed-loop FOC on every axis.
- **Absolute 14-bit magnetic encoders** (AS5048A). The gimbal needs no homing sweep at boot.
- **1 kHz IMU attitude estimation** using a Mahony filter with gyro bias estimation.
- **Dual-core real-time split.** Core 1 runs the FOC and control loops. Core 0 runs BLE, UI and telemetry.
- **Modes:** Follow, Pan-Lock, FPV, Full Lock and Recenter.
- **Fail-safes:** boot-safe motor enable, IMU watchdog, encoder sanity checks, over-temperature limits and low-battery cutoff.
- **Live tuning** over USB serial (SimpleFOC Commander), with parameters stored in NVS.
- **BLE control** for mode selection, joystick input and telemetry.

---

## System Architecture

### Block Diagram

```
                         ┌──────────────────────────────────────────┐
   2S Li-ion ──► Fuse ──►│ VBAT bus (6.0 – 8.4 V)                   │
                 │       └──┬──────────────┬────────────────────────┘
                 │          │              │
                 │      ┌───▼────┐     ┌───▼──────────────────────┐
                 │      │ Buck   │     │ 3× DRV8313 driver        │
                 │      │ 3.3 V  │     │ (SimpleFOC Mini)         │
                 │      └───┬────┘     └──┬────────┬────────┬─────┘
                 │          │             │ UVW    │ UVW    │ UVW
  VBAT divider ──┘          │          ┌──▼──┐  ┌──▼──┐  ┌──▼──┐
       │                    │          │ M1  │  │ M2  │  │ M3  │
       │     ┌──────────────▼───────┐  │TILT │  │ROLL │  │ PAN │
       │     │       ESP32          │  └──┬──┘  └──┬──┘  └──┬──┘
       └────►│ ADC1      MCPWM/LEDC ├─────┘ PWM     │        │
             │                      │  (9 phases + shared EN)
             │  VSPI ◄──────────────┼── AS5048A ×3 (one per motor)
             │  I2C  ◄──────────────┼── ICM-42688-P IMU (camera plate)
             │  GPIO ◄──────────────┼── Button, joystick
             │  BLE / USB-UART ─────┼──► Phone app / PC tuning
             └──────────────────────┘
```

### Control Loop Hierarchy

```
 User input ──► Mode logic ──► Target attitude (q_target)
                                     │
       IMU (1 kHz) ──► Mahony ──► q_est
                                     │
                         ┌───────────▼───────────┐
                         │ Attitude error → axis │  OUTER LOOP
                         │ PID per axis (500 Hz) │  (stabilization)
                         └───────────┬───────────┘
                                     │ motor angle / velocity setpoint
                         ┌───────────▼───────────┐
                         │ SimpleFOC angle loop  │  MIDDLE LOOP
                         │ P → velocity PI       │
                         └───────────┬───────────┘
                                     │ Uq (voltage-mode torque)
                         ┌───────────▼───────────┐
                         │ FOC commutation (SVPWM│  INNER LOOP
                         │ ~5–8 kHz loopFOC)     │  (encoder-based)
                         └───────────────────────┘
```

### Task / Core Allocation (FreeRTOS)

| Task          | Core | Priority | Rate       | Responsibility                           |
|---------------|------|----------|------------|------------------------------------------|
| `foc_task`    | 1    | 24 (max) | free-run   | `loopFOC()` + `move()` on all 3 motors   |
| `ctrl_task`   | 1    | 22       | 1 kHz      | IMU read, Mahony update, outer PID       |
| `safety_task` | 0    | 20       | 100 Hz     | Watchdogs, temperature, battery, faults  |
| `ui_task`     | 0    | 5        | 50 Hz      | Button, joystick, mode state machine     |
| `ble_task`    | 0    | 3        | event      | NimBLE GATT service                      |
| `telemetry`   | 0    | 2        | 50 Hz      | Serial / BLE streaming                   |

> The Wi-Fi radio is disabled. BLE runs on core 0 so that radio interrupts do not add jitter to commutation.

---

## Hardware Overview

| Subsystem   | Part                                  | Notes                                       |
|-------------|---------------------------------------|---------------------------------------------|
| MCU         | ESP32-WROOM-32E (4 MB)                | **Do not use WROVER.** GPIO16/17 are needed. |
| Motors      | 3× 2204 260KV gimbal BLDC, 12N14P     | 7 pole pairs, ~10 Ω phase-to-phase          |
| Drivers     | 3× SimpleFOC Mini (DRV8313)           | Up to 2.5 A peak; we run < 0.8 A            |
| Encoders    | 3× AS5048A (SPI, 14-bit)              | Diametric magnet, 1–2 mm air gap            |
| IMU         | ICM-42688-P (I2C @ 1 MHz)             | Mounted rigidly on the camera plate         |
| Power       | 2S 18650 or 2S 1000 mAh LiPo          | 6.0 – 8.4 V                                 |
| Regulator   | MP1584 buck → 3.3 V                   | Feeds ESP32, IMU and encoders               |

---

## Wiring & Pinouts

### ESP32 Pin Map

| GPIO | Function            | Direction | Notes                                                   |
|------|---------------------|-----------|---------------------------------------------------------|
| 25   | TILT phase A PWM    | OUT       | MCPWM0                                                  |
| 26   | TILT phase B PWM    | OUT       | MCPWM0                                                  |
| 27   | TILT phase C PWM    | OUT       | MCPWM0                                                  |
| 32   | ROLL phase A PWM    | OUT       | MCPWM1                                                  |
| 33   | ROLL phase B PWM    | OUT       | MCPWM1                                                  |
| 13   | ROLL phase C PWM    | OUT       | MCPWM1                                                  |
| 14   | PAN phase A PWM     | OUT       | LEDC (both MCPWM units are in use)                      |
| 16   | PAN phase B PWM     | OUT       | LEDC                                                    |
| 17   | PAN phase C PWM     | OUT       | LEDC                                                    |
| 2    | DRIVER_EN (shared)  | OUT       | **10 kΩ pull-down.** Motors stay disabled through boot; the pin also satisfies the strapping requirement |
| 18   | SPI SCK             | OUT       | VSPI                                                    |
| 19   | SPI MISO            | IN        | VSPI                                                    |
| 23   | SPI MOSI            | OUT       | VSPI                                                    |
| 5    | CS – TILT encoder   | OUT       | Strapping pin; idle-high is fine                        |
| 15   | CS – ROLL encoder   | OUT       | Strapping pin; idle-high is fine                        |
| 4    | CS – PAN encoder    | OUT       |                                                         |
| 21   | I2C SDA (IMU)       | I/O       | 2.2 kΩ pull-up to 3.3 V                                 |
| 22   | I2C SCL (IMU)       | OUT       | 2.2 kΩ pull-up to 3.3 V                                 |
| 35   | IMU INT1 (data-ready) | IN      | Input-only pin                                          |
| 34   | VBAT sense (ADC1_6) | IN        | Divider 100 k / 33 k → 8.4 V maps to 2.08 V             |
| 36   | Joystick X (ADC1_0) | IN        | Optional                                                |
| 39   | Mode button         | IN        | External 10 kΩ pull-up; active low                      |
| 1/3  | USB-UART TX/RX      | —         | Used for flashing and the tuning console                |

> **⚠ ADC2 is unavailable while the radio is active.** All analog inputs therefore use ADC1 pins (32–39).
> **⚠ GPIO 0, 2, 5, 12 and 15 are strapping pins.** Do not add external pull-ups on GPIO2 or GPIO12.

### Driver Board Wiring (×3)

```
 SimpleFOC Mini           ESP32 / Power
 ┌──────────────┐
 │ IN1  ────────┼──── PWM A (see table)
 │ IN2  ────────┼──── PWM B
 │ IN3  ────────┼──── PWM C
 │ EN   ────────┼──── GPIO2 (shared, 10k pull-down)
 │ nFT  ────────┼──── (optional fault, wired-OR) 
 │ GND  ────────┼──── Star ground
 │ VM   ────────┼──── VBAT (after fuse