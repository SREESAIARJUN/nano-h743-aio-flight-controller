# STM32H743 Nano Flight Controller + 4-in-1 ESC AIO

**Autonomous Engineering Release - Cycle 21**  
*Ultra-Dense 25.5x25.5mm (Whoop/Toothpick/Nano) AIO Board for ArduPilot & PX4 with Integrated 4x 20A/35A ESC.*

---

## 1. System Overview
- **MCU**: STM32H743VIT6 (480 MHz ARM Cortex-M7, 2MB Flash, 1MB RAM, Double-Precision FPU).
- **PCB Stackup**: 6-Layer High-Density Interconnect (HDI) with 2oz copper outer & inner power planes.
- **Dual IMUs**:
  - Primary: ICM-42688-P on high-speed SPI1 with isolated ground plane and silicon damping pads.
  - Secondary: ICM-42688-P on SPI4 for redundancy and sensor voting.
- **Barometer**: Bosch / Infineon DPS310 high-precision baro on I2C1 with acoustic dampening.
- **OSD**: AT7456E analog OSD on SPI2 + dedicated 6-pin SH1.0 connector for DJI O3 / Walksnail Avatar / HDZero digital VTX.
- **DroneCAN / UAVCAN**: On-board TCAN334 CAN FD transceiver on FDCAN1.
- **Blackbox**: MicroSD card slot with 4-bit high-speed SDIO (SDMMC1).
- **Integrated 4-in-1 ESC**:
  - 4x Brushless ESC channels (20A continuous, 35A burst each, 2S–4S LiPo input).
  - 24x Low-RDS(on) 40V N-MOSFETs (TPHR8504PL, 1.4 mΩ).
  - 4x FD6288Q 3-phase half-bridge gate drivers with bootstrap diodes.
  - 0.5 mΩ 2W shunt resistor with INA226 / INA169 current monitor.
  - Support for BLHeli_32 / AM32 firmware, DShot300/600/1200, and bi-directional DShot RPM filtering.

---

## 2. Repository Contents
- `kicad/`: Complete KiCad 8 6-layer schematic and PCB layout.
- `gerbers/`: Production-ready Gerbers and drill files.
- `ardupilot/`: Official ArduPilot board definition `hwdef.dat` and bootloader `hwdef-bl.dat`.
- `am32_esc/`: AM32 ESC target configuration `am32_target.h`.
- `analysis/`: Python thermal loss analysis (`thermal_power_analysis.py`) and operating curve plot (`thermal_dissipation_curve.png`).
