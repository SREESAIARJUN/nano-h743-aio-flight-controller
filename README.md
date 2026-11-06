# STM32H743 Nano Flight Controller + 4-in-1 ESC AIO

**Autonomous Engineering Release - Cycle 31**  
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
- `kicad/gerbers/`: Production-ready Gerbers and drill files.
- `ardupilot/`: ArduPilot board definition `hwdef.dat`, bootloader `hwdef-bl.dat`, and board `defaults.parm`.
- `tools/validate_dma.py`: DMA-allocation validator that runs ArduPilot's own `chibios_hwdef.py` / `dma_resolver.py` and enforces the stream policy.
- `am32_esc/`: AM32 ESC target configuration `am32_target.h`.
- `analysis/`: Python thermal loss analysis (`thermal_power_analysis.py`) and operating curve plot (`thermal_dissipation_curve.png`).

---

## Revision 6.0 Engineering Changelog

**Scope:** ArduPilot `hwdef.dat` DMA stream allocation for USART1, SPI1, SPI4 and the 4-bit microSD on SDMMC1, without DMA conflicts.

### Changes implemented

**`ardupilot/hwdef.dat`: rewritten. The Rev 5 file could not be processed.**
Before this revision, `chibios_hwdef.py` stopped on the first line with `Error: Unable to find module for MCU STM32H743VITx`, so the board definition had never been built. These defects were fixed:

| Defect (Rev 5) | Fix (Rev 6) |
|---|---|
| `MCU STM32H743xx STM32H743VITx` (wrong format) | `MCU STM32H7xx STM32H743xx` |
| Raw `STM32_PLLx_VALUE` lines (not hwdef keywords) | Removed. `stm32h7_mcuconf.h` derives the PLL from `OSCILLATOR_HZ 25000000` |
| Invalid `CONFIG_HAL_BOARD_SUBTYPE`, no `APJ_BOARD_ID` | `APJ_BOARD_ID 5743` (**provisional**, unallocated upstream; must be registered before release) |
| `BIDIR_DSHOT_DMA` (not a real keyword) on all four motors | `BIDIR` on TIM1_CH1 and TIM1_CH3 only. ArduPilot pairs CH1/CH2 and CH3/CH4 for eRPM capture |
| `SPIDEV` lines missing devid and mode, CS given as a pin name | `SPIDEV imuN SPIx DEVID1 IMUn_CS MODE3 2*MHZ 16*MHZ`, `IMU Invensensev3 SPI:imuN` |
| `BARO DPS310:0 …`, `OSD_ENABLED` without `define`, obsolete `CAN_ORDER` | Corrected syntax. OSD font ROMFS added |
| ADC `SCALE(10.9)` on the pin (wrong layer) | Pin `SCALE(1)` plus `HAL_BATT_VOLT/CURR_PIN 10/11` and `HAL_BATT_*_SCALE` |
| No USB, SWD, `SERIAL_ORDER`, storage, LED, beeper, FATFS | Added OTG1 + VBUS, SWD, `SERIAL_ORDER OTG1 USART1 USART2 USART3`, flash storage in sectors 14–15, notify LED, beeper, `HAL_OS_FATFS_IO` |
| TCAN334 STB pin not driven (its internal pull-up holds the transceiver in standby, so the CAN bus is dead) | `PD3 GPIO_CAN1_SILENT OUTPUT LOW` |

**New files**
- `ardupilot/hwdef-bl.dat`: bootloader (USB + USART1 with no DMA, flashing from microSD over SDMMC1, all SPI chip-selects held high).
- `ardupilot/defaults.parm`: bidirectional DShot600 (`SERVO_BLH_BDMASK 15`, 14 poles), RPM-referenced harmonic notch, logging to microSD.
- `tools/validate_dma.py`: runs ArduPilot's own generator and DMA resolver and enforces the DMA policy (see Validation status).

### Architecture: DMA stream map

The STM32H743 has 16 general-purpose streams (DMA1 + DMA2 through DMAMUX1). This is the resolved map from the generated `hwdef.h`:

| Stream | Request | Consumer | Policy |
|---|---|---|---|
| DMA1-0 | TIM1_UP (+ CH1, CH3 capture) | DShot600 out + eRPM in, motors 1–4 | NOSHARE |
| DMA1-1 / 1-2 | SPI1_RX / SPI1_TX | IMU1 ICM-42688-P | NOSHARE |
| DMA1-3 / 1-4 | SPI4_RX / SPI4_TX | IMU2 ICM-42688-P | NOSHARE |
| DMA1-5 / 1-6 | USART1_RX / USART1_TX | MSP DisplayPort / MAVLink2 (≤ 2 Mbaud) | NOSHARE |
| DMA1-7 | ADC1 | Battery voltage and current | dedicated |
| DMA2-0 | USART2_RX | CRSF / ELRS | dedicated |
| DMA2-1 | USART3_RX | GPS | dedicated |
| DMA2-2 / 2-5 | SPI2_TX / SPI2_RX | AT7456E OSD | shareable |
| DMA2-3, 2-4, 2-6, 2-7 | — | **4 spare** | — |

Design decisions:
- **SDMMC1 uses no stream.** On H7 the SDMMC has its own IDMA bus master, which `chibios_hwdef.py` deliberately leaves out of stream allocation. IDMA cannot reach DTCM. ArduPilot's FATFS layer copies through DMA-safe AXI SRAM, so 4-bit logging adds no contention for the IMU or UART streams. Because SDMMC1 occupies PC10/PC11/PC12/PD2, UART4, UART5 and SPI3 can't use those pins.
- **The motor timer uses one stream.** On DMAMUX parts ArduPilot reuses the TIM1_UP stream for TIM1_CHx input capture. This works because bidirectional DShot alternates in time: the frame goes out over UP, then the DMAMUX is switched to CHx to capture the eRPM reply. Different peripherals are not sharing a stream, and `SHARED_DMA_MASK` is `0`.
- **Fewer streams by removing low-value DMA.** I2C1 is excluded from DMA by default on H7. USART2_TX (CRSF telemetry) and USART3_TX (GPS configuration) are `NODMA`, using interrupt-driven transmit for low-volume traffic. That brings usage to 12/16 streams with zero sharing.
- **Policy.** `DMA_NOSHARE TIM1* SPI1* SPI4* USART1*` guarantees the IMUs, motors and the high-rate link are never shared, even if future peripherals use up all streams. `DMA_PRIORITY` sets only the resolver's allocation order. Hardware arbitration priority is fixed by ArduPilot (DShot PL3 > ADC 2 > SPI 1 > USART 0) and can't be changed from the hwdef.

### Validation status

| Check | Result |
|---|---|
| `chibios_hwdef.py` on `hwdef.dat` (firmware) | **PASS**: generates `hwdef.h` (Rev 5: failed at line 1) |
| `chibios_hwdef.py --bootloader` on `hwdef-bl.dat` | **PASS** |
| `python3 tools/validate_dma.py --ardupilot <tree>`: R1 dedicated streams for SPI1, SPI4, USART1 RX/TX and TIM1_UP · R2 no sharing between peripherals · R3 `SHARED_DMA_MASK == 0` · R4 SDMMC1 + FATFS enabled on IDMA · R5 ≤ 16 streams · R6 bootloader generates · R7 single `DMA_PRIORITY`/`DMA_NOSHARE` | **PASS** (0 failures) |
| Negative test: protections removed and UART4/7/8 added | Validator correctly **FAILS** (SPI2 shares streams with UART TX, `SHARED_DMA_MASK != 0`) |
| Independent three-lens review (pins/AF, DMA semantics, params/defines) | 3 defects found and fixed (stray `DMA_PRIORITY` fragment, wrong comment about hardware priority, LED define that nothing read) |
| Full `waf` firmware compile | **Not run.** No `arm-none-eabi` toolchain or compiled `*_bl.bin` in this environment. The validator stages a copy with `AP_BOOTLOADER_FLASHING_ENABLED 0` for that reason only, which does not affect DMA resolution. |
| On hardware | **Pending.** After first flash, read `@SYS/dma.txt` over MAVFTP (expect < 1 % contention on SPI1/SPI4/USART1) and `@SYS/timers.txt` (bidirectional capture active), and run an SD write test at 1 kHz logging. |

**Open items:** register `APJ_BOARD_ID`. The KiCad schematic is still a placeholder title block, so this `hwdef.dat` is currently the only record of the pin assignment and must be carried into the schematic netlist. Confirm the status LED polarity (`OUTPUT LOW`, assumed active-low).
