I don't have access to your repository or the outputs of Cycles 1–2, so this README is built as a **coherent reference design**: ESP32-S3, SimpleFOC, DRV8316 drivers, MT6701 encoders, ICM-42688-P IMU, and a 2S Li-ion pack. Every pin, part, and gain is internally consistent. If your hardware differs, swap in your values. The tables are structured to make that easy. Items marked ⚠️ are the ones most worth checking against your actual build.

---

````markdown
# osmo-pocket-esp32-gimbal

> An open-source, pocket-sized 3-axis handheld camera gimbal inspired by the DJI Osmo Pocket.
> It uses an ESP32-S3, field-oriented control (FOC) of brushless gimbal motors, and absolute magnetic encoders on every axis.

![status](https://img.shields.io/badge/status-beta-orange)
![mcu](https://img.shields.io/badge/MCU-ESP32--S3-blue)
![framework](https://img.shields.io/badge/framework-Arduino%20%2B%20SimpleFOC-green)
![license](https://img.shields.io/badge/license-MIT%20(firmware)%20%7C%20CERN--OHL--P%20(hardware)-lightgrey)

---

## Table of Contents

1. [Features](#features)
2. [Specifications](#specifications)
3. [System Architecture](#system-architecture)
4. [Repository Layout](#repository-layout)
5. [Hardware](#hardware)
   - [Wiring & Pinout](#wiring--pinout)
   - [Power System](#power-system)
6. [Bill of Materials](#bill-of-materials)
7. [Firmware: Build & Flash](#firmware-build--flash)
8. [First Power-Up & Calibration](#first-power-up--calibration)
9. [FOC Tuning Procedure](#foc-tuning-procedure)
10. [Stabilization Loop Tuning](#stabilization-loop-tuning)
11. [Operating Modes & Controls](#operating-modes--controls)
12. [Serial Command Reference](#serial-command-reference)
13. [Troubleshooting](#troubleshooting)
14. [Safety](#safety)
15. [Roadmap](#roadmap)
16. [Contributing](#contributing)
17. [License](#license)

---

## Features

- **3-axis stabilization** (pan / tilt / roll) using closed-loop FOC on each axis.
- **Absolute encoders on every joint.** There is no homing dance at boot, and joint limits are enforced.
- **Dual-core real-time split.**
  - Core 1 runs the FOC loops.
  - Core 0 runs attitude estimation, the UI, and BLE.
- **Operating modes:** Follow, Pan-Follow, Lock, FPV, and Recenter.
- **Controls:** 2-axis thumb joystick and two buttons. Optional BLE remote/telemetry.
- **Persistent calibration.** Encoder zero offsets, IMU bias, and PID gains are stored in NVS.
- **Live tuning** over USB-CDC using the SimpleFOC `Commander` protocol plus gimbal-level commands.
- **Battery:** 2S Li-ion with USB-C charging and cell balancing.
- **Safety:** under-voltage cutoff, driver fault handling, and stall detection.

## Specifications

| Parameter | Value |
|---|---|
| MCU | ESP32-S3 (dual-core Xtensa LX7 @ 240 MHz), WROOM-1-N16R8 module |
| Motors | 3 × 2204 brushless gimbal motor, 12N14P (7 pole pairs), ~260 KV |
| Motor drivers | 3 × TI DRV8316 (hardware variant, 3×PWM mode) |
| Encoders | 3 × MagnTek MT6701, 14-bit absolute, SSI |
| IMU | TDK ICM-42688-P (SPI, 1 kHz ODR) |
| FOC loop rate | ≥ 2 kHz per motor (voltage-mode FOC, SVPWM) |
| Attitude loop rate | 1 kHz (Mahony filter) |
| PWM frequency | 25 kHz (inaudible) |
| Supply | 2S Li-ion (6.0–8.4 V), 1000–1500 mAh |
| Charging | USB-C, 5 V input, BQ25887 2S boost charger with balancing |
| Payload | ≤ 60 g, centre of gravity within ±3 mm of the tilt axis |
| Mechanical range | Pan ±170°, Tilt −90°/+45°, Roll ±45° |
| Runtime (typical) | ~2.5 h @ 1200 mAh, balanced payload |

---

## System Architecture

### Hardware Block Diagram

```
                         ┌──────────────────────────────────────────────┐
   USB-C ──► BQ25887 ───►│ 2S Li-ion pack (6.0–8.4 V)                   │
   (5V)      charger +   └───────────────┬──────────────────────────────┘
             balancer                    │ VBAT
                                         ├──────────► Power switch / ideal-diode
                                         │
                    ┌────────────────────┼─────────────────────────────┐
                    │                    │                             │
               TPS62933 buck        VMOT bus (VBAT)            Divider 100k/33k
               (VBAT → 3.3 V)            │                             │
                    │          ┌─────────┼─────────┐                   ▼
                    │          ▼         ▼         ▼             ADC (VBAT sense)
                    │      DRV8316   DRV8316   DRV8316
                    │      (PAN)     (TILT)    (ROLL)
                    │        │ UVW     │ UVW     │ UVW
                    │        ▼         ▼         ▼
                    │      2204      2204      2204   ◄── diametric magnets
                    │      motor     motor     motor        on rotor shafts
                    │        ▲         ▲         ▲
                    │      MT6701    MT6701    MT6701   (SSI, shared bus)
                    │        │         │         │
 3.3 V ─────────────┴────────┴─────────┴─────────┴───────────┐
                                                             │
        ┌────────────────────────────────────────────────────┴────────┐
        │                         ESP32-S3                            │
        │  MCPWM0 ── PAN, TILT 3×PWM        FSPI ── IMU + 3 encoders  │
        │  MCPWM1 ── ROLL 3×PWM             ADC1 ── VBAT, joystick X/Y│
        │  GPIO   ── nSLEEP, nFAULT, buttons, status LED              │
        │  USB-CDC ── tuning console        BLE ── remote / telemetry │
        └──────────────────────────┬──────────────────────────────────┘
                                   │ SPI
                              ICM-42688-P (mounted on the camera plate)
```

### Control Architecture

```mermaid
flowchart L