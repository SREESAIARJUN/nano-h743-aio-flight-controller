# STM32H743 Nano Flight Controller + 4-in-1 ESC AIO

**Autonomous Engineering Release - Cycle 31**  
*Ultra-Dense 25.5x25.5mm (Whoop/Toothpick/Nano) AIO Board for ArduPilot & PX4 with Integrated 4x 20A/35A ESC.*

---

## 1. System Overview
- **MCU**: STM32H743VIT6 (480 MHz ARM Cortex-M7, 2MB Flash, 1MB RAM, Double-Precision FPU), clocked from a 24 MHz HSE crystal. 480 MHz requires revision V silicon.
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
- `tools/gen_mcu_core_sch.py`: generates the MCU-core schematic, project symbol library and `sym-lib-table` from `ardupilot/hwdef.dat` (Rev 7).
- `tools/verify_mcu_core_sch.py`: schematic gate covering KiCad ERC, netlist topology of the HSE and VDDA circuits, and the hwdef ↔ schematic pin map (Rev 7).
- `analysis/hse_vdda_analysis.py`: design check for the 24 MHz crystal and the VDDA filter (clock tree, load capacitance, gm_crit, drive level, LC damping). Writes `analysis/hse_vdda_report.json`.
- `am32_esc/`: AM32 ESC target configuration `am32_target.h`.
- `analysis/`: Python thermal loss analysis (`thermal_power_analysis.py`) and operating curve plot (`thermal_dissipation_curve.png`).

---

## Revision 7.0 Engineering Changelog

**Scope:** KiCad 8 schematic for the STM32H743VIT6 at 480 MHz, covering the external 24 MHz crystal oscillator, its load capacitance, and ferrite-bead filtering of VDDA.

### Changes implemented

**1. The 25 MHz HSE could never run at 480 MHz, so the crystal moved to 24 MHz.**
Rev 6 set `OSCILLATOR_HZ 25000000`. The README advertised 480 MHz, but ArduPilot's `stm32h7_mcuconf.h` has no PLL set for 25 MHz with a custom clock rate. It hits `#error "Unable to configure custom clockrate"`. I confirmed this by running the real header through `gcc -E` with `STM32_HSECLK=25000000U` and `HAL_CUSTOM_MCU_CLOCKRATE=480000000`. At 24 MHz the same header resolves to:

| PLL | Ref (÷M) | VCO (×N) | Outputs |
|---|---|---|---|
| PLL1 | 24/3 = 8 MHz | 960 MHz | P/2 = **480 MHz SYSCLK**, Q/12 = 80 MHz (SPI1-3, SDMMC1, FDCAN) |
| PLL2 | 24/2 = 12 MHz | 600 MHz | P/3 = 200, Q/6 = 100 MHz (SPI4/5, UARTs), R/3 = 200 MHz |
| PLL3 | 24/6 = 4 MHz | 288 MHz | Q/6 = **48.000 MHz USB (exact)**, R/9 = 32 MHz (ADC, I2C) |

- `ardupilot/hwdef.dat`: `OSCILLATOR_HZ 24000000` plus `MCU_CLOCKRATE_MHZ 480`. The generated `hwdef.h` has `HAL_CUSTOM_MCU_CLOCKRATE 480000000`. ChibiOS sets ODEN, giving VOS0.
- `ardupilot/hwdef-bl.dat`: `OSCILLATOR_HZ 24000000`. The bootloader stays at 400 MHz on the rev Y/X-compatible path.
- **Silicon:** a 960 MHz VCO and 480 MHz are only specified for **revision V** (`REV_ID 0x2003`). ArduPilot does not check the silicon revision at run time. A rev Y part would boot and overclock silently. The requirement is therefore recorded as the `Silicon_Rev` procurement field on U1 and enforced by the verifier.
- The hwdef SPI comment was wrong. ArduPilot's divisor search turns the `16*MHZ` request into **10 MHz on SPI1** (80 MHz kernel clock) and **12.5 MHz on SPI4** (100 MHz kernel clock). The comment now states the real rates.

**2. The KiCad schematic is real now. Before, it was an empty title block.**
`kicad/nano-h743-aio.kicad_sch` (A3, KiCad 8 format 20231120) is generated deterministically by `tools/gen_mcu_core_sch.py`. The generator also writes the project symbol library `kicad/nano-h743-aio.kicad_sym` and `kicad/sym-lib-table`, so the project does not depend on stock libraries being installed.

| Ref | Value | Part (LCSC) | Function |
|---|---|---|---|
| U1 | STM32H743VITx | STM32H743VIT6 (C114409), LQFP-100, **rev V** | MCU, all 100 pins placed |
| Y1 | 24 MHz, CL 12 pF | Yajingxin TXM24M0004322FBCDO00T (C362340), 3225-4P, ESR ≤ 30 Ω, C0 ≤ 5 pF, ±10/±20 ppm | HSE crystal; case pins 2 and 4 to GND |
| C1, C2 | 16 pF | Murata GJM1555C1H160FB01J (C3853959), C0G ±1 % | load caps on OSC_IN and OSC_OUT |
| R1 | 0 Ω | 0402 (C17168) | Rext footprint on OSC_OUT. **Never fit more than 6.8 Ω** |
| FB1 | 600 Ω @ 100 MHz | Murata BLM15AX601SN1D (C76886), Rdc ≤ 0.39 Ω, 500 mA | VDDA ferrite bead |
| R2 | 2.2 Ω | UNI-ROYAL 0402WGF220KTCE (C25101), 1 % | damps the bead-plus-capacitor LC resonance |
| C3/C4, C5/C6 | 1 µF + 100 nF each | CL05A105KA5NQNC (C52923) / CL05B104KO5NNNC (C1525) | at VDDA pin 21 and at VREF+ pin 20 |
| C7, C8 | 2.2 µF | CL10B225KP8NNNC (C100082), X7R | VCAP pins 48 and 73 (LDO, ESR < 100 mΩ) |
| C9–C13, C14, C15 | 5 × 100 nF, 4.7 µF, 100 nF | C1525, C19666 | VDD pins 11/27/50/75/100, bulk, VBAT |
| C16, R3 | 100 nF, 10 kΩ | C1525, C25744 | NRST filter, BOOT0 pull-down |

Net topology: `+3V3 → FB1 → VDDA_FB → R2 → +3.3VA`. The +3.3VA net feeds only VDDA (21), VREF+ (20) and C3–C6. VREF+ is tied to VDDA, which is legal while VREFBUF stays off. VSSA (19) goes to GND. VBAT is tied to VDD because the board has no RTC backup. PC14 and PC15 are left unconnected because there is no LSE.

**3. The MCU pin map is derived from the firmware.** Every GPIO assigned in `hwdef.dat` (43 pins) is broken out with a global label named after its hwdef function, such as `SPI1_SCK`, `IMU1_CS` and `TIM1_CH1`. The other 37 GPIOs get explicit no-connect flags. This closes the Rev 6 open item about `hwdef.dat` being the only record of the pin assignment. Changing a pin in `hwdef.dat` without regenerating the schematic fails gate S1.

### Architecture improvements

- **Functional MCU symbol.** Supplies sit on the top edge, with VBAT plus five VDD on a +3V3 bus and VREF+/VDDA on a separate +3.3VA bus. Returns sit on the bottom edge (five VSS plus VSSA). Clock, reset, boot and VCAP pins are grouped at the top left, and ports A/B (left) and C/D/E (right) are listed in order. Physical pin numbers match KiCad's official `MCU_ST_STM32H7:STM32H743VITx` symbol pin for pin.
- **Crystal design** (`analysis/hse_vdda_analysis.py`; inputs taken from the Y1 datasheet where it gives them):
  - **Load caps:** C = 2 × (CL − Cstray) = 2 × (12 − 4) = **16 pF**, giving CL_eff = 12.00 pF and 0 ppm nominal pull. Over a stray range of 2.5–5.5 pF, ±1 % caps and a motional capacitance up to 8 fF, CL_eff spans 10.42–13.58 pF and the pull spans **+24 / −20 ppm**.
  - **Start-up margin (AN2867):** gm_crit = 4·ESR·(2πf)²·(C0+CL)² = **0.94 mA/V** at the worst corner (ESR 30 Ω, C0 5 pF, CL 13.58 pF). That is a **×1.59 margin** against the STM32H7 HSE limit Gm_critmax = 1.5 mA/V. Required margin is ×1.2.
  - **Substitution rule** (schematic note): a replacement crystal needs ESR_max ≤ 40 Ω at CL 12 pF, or ≤ 20 Ω at CL 20 pF. Typical 18–20 pF / 50 Ω parts fail.
  - **Frequency error over 10 years:** 104 ppm (pull 24 + tolerance 10 + stability 20 + aging 5 ppm/yr). Allowed: 1000 ppm. The tightest consumer is DroneCAN at 1 Mbit/s, which tolerates ≥ 3000 ppm; I applied a 3× margin. USB FS allows 2500 ppm.
  - **Drive level:** DL = ESR·(π·f·Vpp·Ctot)²/2 is 27.6 µW at 1 Vpp and 301 µW at 3.3 Vpp. The datasheet limit is 100 µW, so **Vpp on OSC_OUT must be ≤ 1.90 V**, and this is checked at bring-up. R1 cannot fix an overdrive: anything above 9.8 Ω (6.8 Ω E12) erodes the ×1.2 start-up margin. If bring-up measures more than 1.9 Vpp, change the crystal to a part with a higher drive-level rating.
- **VDDA filter design.** The bead's low-frequency inductance is not specified, so it is swept from 0.47 to 4.7 µH. Bead Rdc is swept from 0.10 to 0.39 Ω. The capacitors are modelled with 45 % DC-bias loss and 1 nH ESL each, and R2 at −1 %.

  | R2 | Worst peaking | DC drop at 10 mA |
  |---|---|---|
  | 0 Ω (bead only) | **+25.0 dB** | 3.9 mV |
  | 1.0 Ω | +5.4 dB (fails) | 14.0 mV |
  | 1.5 Ω | +2.6 dB (only 0.4 dB under the hard limit) | 19.1 mV |
  | **2.2 Ω (selected)** | **+0.5 dB** | **26.1 mV** |

  R2 is chosen by the script: the smallest value with peaking ≤ 2 dB, which leaves a 1 dB guard band under the 3 dB hard limit and keeps the DC drop ≤ 30 mV. Resonance sits between 50 and 211 kHz depending on the corner. Worst-case attenuation is ≥ 29 dB at 1 MHz, ≥ 57 dB at 10 MHz and ≥ 65 dB at 100 MHz. These figures are for SMPS harmonics and MCU core noise. The 26 mV drop is a constant ratiometric offset of 0.8 % on ADC readings, which `BATT_VOLT_MULT` calibration absorbs.
- **Layout rules** (on the schematic sheet, to be applied in the PCB revision): Y1, C1 and C2 within 5 mm of pins 12/13 on the U1 layer, with no vias in the OSC traces. Add a GND guard ring stitched to In1 every 1 mm, with no signals under Y1 on In1. Return C1 and C2 to a GND island tied to VSS pin 10, and keep ESC return currents away from it. Place C3/C4 within 1 mm of pin 21 and C5/C6 within 1 mm of pin 20. Via VSSA straight into a solid, unsplit In1 plane.

### Validation status

| Check | Result |
|---|---|
| `kicad-cli sch erc` (KiCad 8.0.9, all severities) | **0 errors.** 43 warnings, all `global_label_dangling` on hwdef interface labels whose peripherals (IMUs, OSD, ESC, connectors) go on sheets that don't exist yet. |
| `python3 tools/verify_mcu_core_sch.py`: S1 regeneration is byte-identical · S2 ERC · S3 U1 pins match KiCad's official symbol (independent table) · S4 exact node sets for every non-interface net, plus all 43 hwdef pins on the correct LQFP-100 numbers · S5 exact values for all 22 parts, key MPNs, no DNP, rev V field · S7 OSCILLATOR_HZ (fw + bl) equals the Y1 frequency and MCU_CLOCKRATE is 480 | **PASS** (0 failures) |
| Verifier mutation tests: VCAP cap returned to +3V3; VDD decoupling shorted to GND; PA9/PA10 swapped in the generator pin table; NRST cap returned to +3V3; R3 = 1 MΩ; Y1 CL 20 pF; all passives DNP; hwdef pin moved without regenerating; 25 MHz OSCILLATOR_HZ; R2 removed | **every mutation caught**; a no-op control passes |
| `python3 analysis/hse_vdda_analysis.py --ardupilot <tree>` (21 checks; PLL dividers read back from `stm32h7_mcuconf.h`) | **PASS** (0 failures) → `analysis/hse_vdda_report.json` |
| `gcc -E` on ArduPilot `stm32h7_mcuconf.h` | 24 MHz @ 480 resolves M3/N120/P2/Q12, PLL3 M6/N72/Q6. 25 MHz @ 480 hits `#error` (the Rev 6 defect) |
| `python3 tools/validate_dma.py --ardupilot <tree>` (Rev 6 DMA policy, re-run on the new clock) | **PASS** (0 failures, 12/16 streams). Generated `hwdef.h`: `STM32_HSECLK 24000000U`, SYSCLK 480 MHz, SDMMC1/FDCAN 80 MHz, SPI4/5 100 MHz |
| Independent 4-lens adversarial review (crystal math, VDDA/power, KiCad rigour, firmware clock) | All verified findings fixed: C0 3 → 5 pF from the datasheet, aging 3 → 5 ppm/yr, R1 limit 33 → 6.8 Ω, R2 1.0 → 2.2 Ω after widening the bead-L sweep, capacitor ESL added, verifier made exact and non-circular, rev V field, SPI rate comment. Earlier self-checks had already caught 15 pF load caps (+22 ppm corner) and two wrong LCSC codes (C25087 is 200 Ω, not 1 Ω) |
| PCB / Gerbers | Title block and Gerbers regenerated as rev 7.0. **The board has no footprints yet**, so no netlist-to-PCB sync or DRC has been run |
| Full `waf` firmware build / hardware | **Not run.** No ARM toolchain here. Bring-up must do three things: measure Vpp on OSC_OUT with a ≤ 1 pF active probe (gate: ≤ 1.90 V); measure HSE frequency through MCO1 (expect 24 MHz ± 50 ppm at 25 °C); check that `@SYS/uarts.txt` and USB enumeration are good at 480 MHz |

**Open items:** the bead's low-frequency inductance and the crystal's motional capacitance should be confirmed from Murata's SPICE model and the Yajingxin factory data, and the analysis re-run with them. The IMU, OSD, ESC, power and connector sheets are still to be drawn against the labels on this sheet.

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
