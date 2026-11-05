I don't have access to the contents of an `osmo-pocket-esp32-gimbal` repository or earlier "cycles," so this is a complete, self-consistent **reference design README**. The pin assignments, part choices, and tuning values are internally checked against each other, the ESP32-S3 strapping and PSRAM constraints, and the driver voltage ranges. If your hardware differs, update the **Pinout**, **BOM**, and `config.h` sections together.

---

````markdown
# osmo-pocket-esp32-gimbal

> A pocket-sized, 3-axis brushless camera gimbal built on the **ESP32-S3**, using **field-oriented control (FOC)** with absolute magnetic encoders and a high-rate IMU. It is inspired by the DJI Osmo Pocket form factor.

![status](https://img.shields.io/badge/status-beta-yellow)
![mcu](https://img.shields.io/badge/MCU-ESP32--S3-blue)
![framework](https://img.shields.io/badge/framework-Arduino%20%2B%20SimpleFOC-green)
![license](https://img.shields.io/badge/license-MIT%20%2F%20CERN--OHL--P-lightgrey)

---

## Table of Contents

1. [Features](#features)
2. [System Overview](#system-overview)
3. [Block Diagrams](#block-diagrams)
4. [Hardware](#hardware)
5. [Wiring & Pinout](#wiring--pinout)
6. [Power System](#power-system)
7. [Bill of Materials](#bill-of-materials)
8. [Firmware Architecture](#firmware-architecture)
9. [Building & Flashing](#building--flashing)
10. [First Power-Up & Calibration](#first-power-up--calibration)
11. [FOC Tuning Procedure](#foc-tuning-procedure)
12. [Stabilization Loop Tuning](#stabilization-loop-tuning)
13. [Operating Modes & Controls](#operating-modes--controls)
14. [Serial Command Reference](#serial-command-reference)
15. [Troubleshooting](#troubleshooting)
16. [Safety](#safety)
17. [Repository Layout](#repository-layout)
18. [Roadmap](#roadmap)
19. [Contributing](#contributing)
20. [License](#license)

---

## Features

- **3-axis stabilization** (pitch / roll / yaw) with closed-loop FOC on each motor.
- **Absolute encoders** (MT6701, 14-bit SSI). There is no startup "twitch" calibration after the first alignment.
- **ICM-42688-P IMU** sampled at 1 kHz over SPI, with Mahony AHRS fusion.
- **Cascaded control:** attitude loop (500 Hz) → joint angle loop → velocity loop → voltage-mode FOC (≈ 5–10 kHz loop, 25 kHz PWM).
- **Modes:** Follow, Pan-Follow, Lock, FPV, and Recenter. Switching is single-button driven.
- **Single-cell Li-ion** input (18650 / 21700) with an on-board 12 V boost for the motors.
- **Wireless:** Wi-Fi web UI for live tuning and BLE for joystick/phone control (optional).
- **Persistent config** stored in NVS (encoder offsets, PID gains, mode).
- **Safety:** driver fault monitoring, stall detection, thermal/current-limit derating, and undervoltage cutoff.

> **Payload:** This design stabilizes a **micro action-camera module** (e.g. RunCam Thumb / Caddx Peanut class, ≤ 25 g). The ESP32-S3 does **not** capture video. All of its GPIOs are spent on motor control and sensing.

---

## System Overview

| Subsystem      | Implementation                                       |
|----------------|------------------------------------------------------|
| MCU            | ESP32-S3-WROOM-1-N16R8 (dual-core LX7 @ 240 MHz)     |
| Motor drivers  | 3 × TI DRV8313 (3-PWM, 8–60 V, 2.5 A peak)           |
| Motors         | 3 × 1806/2204-class gimbal BLDC, 7 pole-pairs        |
| Encoders       | 3 × MT6701 (SSI, 14-bit) with diametric magnets      |
| IMU            | TDK ICM-42688-P (SPI, mounted on camera head)        |
| Power          | 1S Li-ion → 12 V boost (motors) + 3.3 V buck-boost (logic) |
| Interconnect   | 6-wire FPC/flex per joint or a 12-wire slip ring on yaw |

**Core assignment**

| Core | Tasks                                                                 |
|------|-----------------------------------------------------------------------|
| 1    | `foc_task`: `loopFOC()` + `move()` for all 3 motors (tight loop, highest priority) |
| 0    | `imu_task` (1 kHz), `stab_task` (500 Hz), `ui_task`, Wi-Fi/BLE stack   |

---

## Block Diagrams

### System block diagram

```mermaid
flowchart LR
    BAT[1S Li-ion 3.0–4.2 V] --> PROT[Protection + Charger<br/>TP4056/DW01 or BQ24075]
    PROT --> SW[Power switch]
    SW --> BOOST[Boost 12 V<br/>MT3608 / TPS61088]
    SW --> LDO[3.3 V buck-boost<br/>TPS63020]
    SW --> VSENSE[VBAT divider] --> ESP

    BOOST --> DRVP[DRV8313 Pitch]
    BOOST --> DRVR[DRV8313 Roll]
    BOOST --> DRVY[DRV8313 Yaw]
    LDO --> ESP[ESP32-S3]
    LDO --> IMU[ICM-42688-P]
    LDO --> ENC[MT6701 ×3]

    ESP -- 3×PWM + EN --> DRVP --> MP((Pitch BLDC))
    ESP -- 3×PWM + EN --> DRVR --> MR((Roll BLDC))
    ESP -- 3×PWM + EN --> DRVY --> MY((Yaw BLDC))

    ENC -- SSI bus --> ESP
    IMU -- SPI + INT --> ESP
    ESP -- USB-C --> PC[Host / Web UI]
    ESP -. Wi-Fi/BLE .-> PHONE[