I don't have access to the actual `osmo-pocket-esp32-gimbal` repository or the outputs of Cycles 1–9, so this README is a complete reference design built on common, verifiable parts and SimpleFOC conventions. Before merging, diff the pin map, part numbers and tuning constants against your schematic and `config.h`. Anything that depends on your hardware is marked **⚠ verify**.

---

```markdown
# osmo-pocket-esp32-gimbal

> Open-source, pocket-sized 3-axis handheld camera gimbal built around the ESP32-S3,
> brushless gimbal motors, absolute magnetic encoders, and field-oriented control (FOC).

![status](https://img.shields.io/badge/status-beta-orange)
![mcu](https://img.shields.io/badge/MCU-ESP32--S3-blue)
![foc](https://img.shields.io/badge/FOC-SimpleFOC%202.3.x-green)
![license](https://img.shields.io/badge/license-MIT%20%2F%20CERN--OHL--S-lightgrey)

---

## Table of Contents

1. [Overview](#1-overview)
2. [Feature Matrix](#2-feature-matrix)
3. [System Architecture](#3-system-architecture)
4. [Hardware](#4-hardware)
5. [Wiring & Pinouts](#5-wiring--pinouts)
6. [Bill of Materials](#6-bill-of-materials-bom)
7. [Firmware](#7-firmware)
8. [Build & Flash](#8-build--flash)
9. [First Power-Up & Calibration](#9-first-power-up--calibration)
10. [FOC Tuning Procedure](#10-foc-tuning-procedure)
11. [Stabilization Loop Tuning](#11-stabilization-loop-tuning)
12. [Operating Modes & UI](#12-operating-modes--ui)
13. [BLE Control Protocol](#13-ble-control-protocol)
14. [Safety & Fault Handling](#14-safety--fault-handling)
15. [Testing & Validation](#15-testing--validation)
16. [Troubleshooting](#16-troubleshooting)
17. [Repository Layout](#17-repository-layout)
18. [Contributing](#18-contributing)
19. [License & Disclaimer](#19-license--disclaimer)

---

## 1. Overview

`osmo-pocket-esp32-gimbal` is a handheld 3-axis stabilizer inspired by the
form factor of the DJI Osmo Poc