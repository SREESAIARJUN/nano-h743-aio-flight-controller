# STM32H743 Nano Flight Controller + 4-in-1 ESC AIO

**Autonomous Engineering Release - Cycle 31**  
*Dense 36 × 36 mm AIO (30.5 × 30.5 mm M3 mount) for ArduPilot & PX4 with Integrated 4x 20A/35A ESC. Rev 8.0 moved off the 25.5 mm outline because 24 SOP-Advance MOSFETs cannot fit on it; see the Rev 8.0 changelog.*

---

## 1. System Overview
- **MCU**: STM32H743VIT6 (480 MHz ARM Cortex-M7, 2MB Flash, 1MB RAM, Double-Precision FPU), clocked from a 24 MHz HSE crystal. 480 MHz requires revision V silicon.
- **PCB Stackup**: 6 layers, JLCPCB JLC061612-1080, 1.614 mm, ENIG. 1 oz outer for signals and fine pitch, 2 oz inner planes (In1 GND / In2 VBAT / In3 VBAT / In4 GND), filled-and-capped via-in-pad (POFV, IPC-4761 Type VII). Through vias only, no HDI microvias.
- **Dual IMUs**:
  - Primary: ICM-42688-P on high-speed SPI1 with isolated ground plane and silicon damping pads.
  - Secondary: ICM-42688-P on SPI4 for redundancy and sensor voting.
- **Barometer**: Bosch / Infineon DPS310 high-precision baro on I2C1 with acoustic dampening.
- **OSD**: AT7456E analog OSD on SPI2 + dedicated 6-pin SH1.0 connector for DJI O3 / Walksnail Avatar / HDZero digital VTX.
- **DroneCAN / UAVCAN**: On-board TCAN334 CAN FD transceiver on FDCAN1.
- **Blackbox**: MicroSD card slot with 4-bit high-speed SDIO (SDMMC1).
- **Integrated 4-in-1 ESC**:
  - 4x Brushless ESC channels, 2S–4S LiPo input. **Rev 9 thermal rating per motor, all four loaded, 24 kHz PWM:** 20 A continuous / 25 A for 10 s in typical prop wash; 16 A / 21 A in the pessimistic case. 35 A is held for only ~2.4 s, and only ~8 A is allowed on a static bench. These figures require 12 AWG battery and 20 AWG motor leads. Rev 9 lowered them from 20/28 and 17/22 A once the FD6288Q's built-in dead time was included in the loss model.
  - 24x Toshiba TPHR8504PL 40 V N-MOSFETs, SOP Advance. RDS(on) max 0.85 mΩ at VGS=10 V and 1.40 mΩ at 4.5 V, Rth(ch-c) 0.88 K/W. ESC1/2 sit on the top face and ESC3/4 on the bottom; each half-bridge site is shared by both faces, and every high-side pair has a 16-via POFV thermal array.
  - 4x AT32F421K8U7 ESC MCUs running AM32 (target `NANO_H743_AIO_F421`), each driving one FD6288Q 3-phase gate driver. The FD6288Q has no internal bootstrap diode, so each phase has an external low-leakage Schottky, 2.2 Ω and 2.2 µF. Each gate has a 0 Ω series footprint (kept so the slew rate can be re-tuned at bring-up) and a 2.2 nF C0G Cgs. AM32 `DEAD_TIME 43` (358 ns) adds to the driver's fixed 100–300 ns dead time. The FC battery failsafe `BATT_CRT_VOLT 6.4` protects the driver's UVLO (Rev 9).
  - 0.5 mΩ shunt resistor with INA226 / INA169 current monitor. **Rev 8 finding:** at 4 × 20 A, D=0.9 (72 A battery current) it dissipates 2.6 W, which is over the 2 W rating previously specified. It needs a ≥ 4 W part or 0.25 mΩ; this is an open item.
  - Support for BLHeli_32 / AM32 firmware, DShot300/600/1200, and bi-directional DShot RPM filtering.

---

## 2. Repository Contents
- `kicad/`: KiCad 8 project. The schematic covers the MCU core (Rev 7). The 6-layer PCB has the stackup, planes and the 24-MOSFET power stage (Rev 8); the other functions are not placed yet. Project footprint library `nano-h743-aio.pretty`, custom DRC rules `nano-h743-aio.kicad_dru`.
- `kicad/gerbers/`: Gerber X2 for 6 copper layers plus paste, mask, silk and edge, Excellon PTH/NPTH with drill maps, and a Gerber job file carrying the stackup. These are a power-stage fab preview, **not a releasable board yet** (see Rev 8.0 open items).
- `ardupilot/`: ArduPilot board definition `hwdef.dat`, bootloader `hwdef-bl.dat`, and board `defaults.parm`.
- `tools/validate_dma.py`: DMA-allocation validator that runs ArduPilot's own `chibios_hwdef.py` / `dma_resolver.py` and enforces the stream policy.
- `tools/gen_mcu_core_sch.py`: generates the MCU-core schematic, project symbol library and `sym-lib-table` from `ardupilot/hwdef.dat` (Rev 7).
- `tools/verify_mcu_core_sch.py`: schematic gate covering KiCad ERC, netlist topology of the HSE and VDDA circuits, and the hwdef ↔ schematic pin map (Rev 7).
- `analysis/hse_vdda_analysis.py`: design check for the 24 MHz crystal and the VDDA filter (clock tree, load capacitance, gm_crit, drive level, LC damping). Writes `analysis/hse_vdda_report.json`.
- `am32_esc/am32_target.h`: AM32 `Inc/targets.h` block for the AT32F421 + FD6288Q channel (Rev 9).
- `am32_esc/gate_drive_design.py`: FD6288Q/TPHR8504PL gate-drive design (gate-resistor sweep, dead-time derivation, DTG encoding, driver and bootstrap checks). It checks `am32_target.h` and `eeprom_settings.json` against the derived values and writes `gate_drive_report.json`, which the thermal model reads (Rev 9).
- `am32_esc/eeprom_settings.json`: the AM32 EEPROM fields the analysis depends on (PWM frequency, complementary PWM, minimum duty, brake dead-time override, cutoffs), with offsets (Rev 9).
- `tools/am32_build_target.sh`: splices the target into an AM32 checkout, builds it with AM32's pinned toolchain, checks the resolved macros and runs a SITL twin (Rev 9).
- `tools/gen_power_stage_pcb.py`: generates the footprint library, DRC rules, project rules and the 6-layer board (Rev 8).
- `tools/verify_power_stage_pcb.py`: PCB gate covering stackup, MOSFET population, thermal via geometry, through-hole net safety, topology, DRC with a rule-liveness check, regeneration, fab outputs and the thermal report (Rev 8).
- `analysis/thermal_power_analysis.py`: per-MOSFET loss model and a thermal network that reads its geometry back from the KiCad board. Writes `thermal_report.json` and `thermal_dissipation_curve.png` (rewritten in Rev 8).
- `firmware/bdshot_telem/`: bidirectional DShot telemetry reference in C++. It is bit-exact with ArduPilot's GCR/eRPM decode, and adds EDT decoding, round-robin health and plausibility tracking, and notch planning. `make test` runs 46 checks under ASan/UBSan (Rev 10).
- `tools/validate_bdshot.py`: bidir-DShot gate run against an ArduPilot tree. It covers the generated DMA/IC config, capture coverage, parameter validity, a bit-exact decoder cross-check, unit tests, analysis gates and the upstream patch (Rev 10).
- `analysis/bdshot_timing_analysis.py`: TIM1/DShot600/GCR timing, round-robin eRPM rates, notch-lag sweep over `SERVO_DSHOT_RATE`, quantisation and line electricals. Writes `bdshot_report.json` (Rev 10).
- `ardupilot/patches/`: upstream ArduPilot fix for bdshot `RECV_FAILED` error attribution (Rev 10).

---

## Revision 10.0 Engineering Changelog

**Scope:** Bidirectional DShot600 RPM filtering. Covers the ArduPilot TIM1 timer-channel and input-capture configuration, and processing of the ESC telemetry fed back from the motors. Every claim below was checked against ArduPilot 4.7.1 source (`AP_HAL_ChibiOS/RCOutput_bdshot.cpp`, `RCOutput.cpp`, `chibios_hwdef.py`, `dma_resolver.py`, `Filter/HarmonicNotchFilter.cpp`, `AP_ESC_Telem`, `AP_Vehicle`). An independent adversarial review then went over the claims, and its corrections are included.

### Changes implemented

**1. TIM1 timer-channel / input-capture configuration (`ardupilot/hwdef.dat`).** The pin assignment stays `PE9/PE11/PE13/PE14 = TIM1_CH1..CH4`, with `BIDIR` on CH1 and CH3. The block now records why that set is complete and how it actually works, and `tools/validate_bdshot.py` proves it against the generated `hwdef.h`:
- **Capture pairing.** `bdshot_setup_group_ic_DMA()` captures a channel with no IC DMA of its own on its partner's capture unit: CH2 on IC1 with `CC1S=10` (TI2→IC1), and CH4 on IC3. Pairing only exists within CH1/CH2 and CH3/CH4, so BIDIR on 1 and 3 covers all four motors. No `CHxN` outputs are used, because ArduPilot cannot input-capture them.
- **DMA.** `dma_resolver.py` places `TIM1_CH1` and `TIM1_CH3` on the `TIM1_UP` stream (DMA1-0). RCOutput re-points the DMAMUX request from UP to CHx for each receive window (`has_shared_ic_up_dma()`). The whole motor group still costs one stream, and `SHARED_DMA_MASK` stays 0.
- **Round-robin (new finding).** One timer group captures **one** motor's reply per DShot frame (`bdshot.curr_telem_chan`, `bdshot_find_next_ic_channel()`). With all four motors on TIM1, each motor's eRPM refreshes at f_DShot / 4. Earlier revisions did not account for this.
- **Timing at 480 MHz** (TIM1 kernel clock 240 MHz):

| Quantity | Value | Source |
|---|---|---|
| DShot600 output | PSC 49 → 4.8 MHz, 8 ticks/bit, exact | `setup_group_DMA`, `DSHOT_BIT_WIDTH_TICKS` |
| Command frame | 16 + 1 + 2 slots = 31 µs | `dshot_bit_length`, `pulse_send_time_us` |
| GCR reply | 750 kbit/s, 21 bits = 28 µs, ~30 µs after the frame | `bdshot_get_output_rate_hz` (5/4) |
| Capture prescaler | `telempsc` = 19 → 83.3 ns/tick, **exactly 16 ticks per GCR bit** | `RCOutput_bdshot.cpp:300` |
| Receive window | 31 + 30 + 10 = 71 µs (the reply needs 58 µs) | `bdshot_receive_pulses_DMAR` |
| Bidir cycle | 102 µs of a 625 µs frame period (16 %) | |

- **Electrical.** In bidir mode ArduPilot configures the pins as PUSHPULL + PULLUP at OSPEED MID1. The 40 kΩ internal pull-up settles the idle level within 2.8 µs (5τ), well inside the 30 µs turnaround.

**2. DShot series termination in the schematic (`tools/gen_mcu_core_sch.py`).** Added R4–R7: 100 Ω 0402, UNI-ROYAL 0402WGF1000TCE, LCSC C25076 (JLC basic part). Each sits in series on `TIM1_CHn → ESCn_DSHOT` and goes within 3 mm of U1. With ~14 pF of line capacitance, the RC is 1.4 ns (3.1 ns 10–90 %), which is < 2 % of a GCR bit. The resistors damp the ringing from MID1 edges into the ESC MCU input, and limit the current when the FC and ESC both drive the line during turnaround. **No external pull-up was added on purpose.** ESC bootloaders treat a signal line that is high at power-on as a host waiting for 1-wire passthrough. `verify_mcu_core_sch.py` gained the matching netlist, value, MPN and BOM checks, plus a new S8 check that R4..R7 follow hwdef `PWM(1..4)` order to `ESC1..4_DSHOT`.

**3. Parameters (`ardupilot/defaults.parm`).**

| Parameter | Rev 6–9 | Rev 10 | Reason |
|---|---|---|---|
| `SERVO_DSHOT_RATE` | 0 (default) | **4** | Rate 0 sends on every loop push and also on 1 ms timer ticks that each push re-arms, giving uneven 0.5 / 1 / 1 ms spacing (~1.2 kHz). Rate 4 sends a frame every 625 µs, phase-locked to the 400 Hz loop. That gives **400 Hz per motor** under round-robin, the fastest of rates 0–4, and ArduPilot keeps loop × rate ≤ 4096 Hz. |
| `INS_HNTCH_OPTS` | 2 | **6** | Adds bit 2, loop-rate update. Without it the notch is re-centred only when more than 5 ms have passed, which is every 3rd loop (~133 Hz). |
| `INS_HNTCH_HMNCS` / `ATT` | implicit | 3 / 40 | Now explicit. At `INS_GYRO_RATE` 2 kHz, the 2nd harmonic of motors above 480 Hz is past the 0.48 × 2 kHz = 960 Hz Nyquist guard and is skipped. |
| `SERVO_DSHOT_ESC` | 3 | 3 | Documented: ArduPilot sends DShot command 13 to enable EDT, and AM32 then interleaves temperature, voltage and current frames. |

Behaviour below `INS_HNTCH_FREQ` 80 Hz was derived from `HarmonicNotchFilter`. The notch is held at 80 Hz, and its **linear** gain A, not its dB depth, is interpolated towards 1. At 50 Hz this leaves 10.4 dB, not "half of 40 dB". Below 20 Hz the notch is disabled.

**4. ESC telemetry processing reference (`firmware/bdshot_telem/`, C++17, no allocation).**
- `decode_gcr_edges()` is **bit-exact** with `RCOutput::bdshot_decode_telemetry_packet()`. `validate_bdshot.py` V4 extracts the upstream function from the ArduPilot tree, compiles it next to ours and compares 421 184 cases: all payloads × ±6–19 % clock error, jitter, dropped and extra edges, and 200 000 random edge trains. **0 mismatches.** The review caught a divergence on reversed (synthetic) edge intervals, where ArduPilot's signed/unsigned arithmetic yields len 0. That behaviour is now reproduced. Cortex-M7 `LSL` semantics are modelled for upstream's `1U << (len-1)` with len = 0.
- `decode_payload()` covers eRPM and EDT (temperature, voltage, current, debug, v2 stress and status). It keeps ArduPilot's uint16 truncation and the `ZERO_ERPM` case, and adds a distinct `ZeroPeriod` frame type, which ArduPilot counts as clean but which carries no RPM.
- `MotorTelemetryMonitor` tracks staleness with round-robin awareness: a dead ESC is flagged after 3 of its own slots, i.e. 12 frames or 7.5 ms. It also adds a **plausibility gate** that ArduPilot lacks: CRC-valid eRPM with a period under 100 µs is rejected. The 4-bit CRC lets ~5 % of heavily corrupted frames through (test T4), and periods under 10 µs wrap in ArduPilot's uint16 arithmetic. Example: a 9 µs frame reads as 113 100 eRPM.
- `plan_notches()` mirrors the HarmonicNotchFilter centre, Nyquist, minimum-frequency, fade and disable rules for 16 harmonics, capped at `HAL_HNF_MAX_FILTERS` = 54.

**5. Upstream ArduPilot defect + patch (`ardupilot/patches/0001-…patch`).** `bdshot_finish_dshot_gcr_transaction()` rotates `curr_telem_chan` *before* `bdshot_prepare_for_next_pulse()` books the result. `RECV_FAILED` (no edges, i.e. the ESC is silent) is then booked on `curr_telem_chan`, which is the **next** motor. A dead ESC2 therefore inflates ESC3's logged error rate. The patch books the failure on `group.chan[prev_telem_chan]` and applies the 5 s statistics reset on that path as well. Without the reset, the silent channel's uint16 counter would grow until it wraps. The patch applies cleanly (V7). The defect is reproduced in a host model of the counter logic (T8), not on hardware.

**6. Timing analysis (`analysis/bdshot_timing_analysis.py` → `bdshot_report.json`).** The rate sweep uses ArduPilot's own scheduling model. The notch-lag column is the age of the RPM the notch actually uses: round-robin wait + `AP_ESC_Telem::get_rpm()` interpolation (one more update period) + loop consumption + AM32's 6-commutation average (0.43 ms). Punch lag assumes a full punch of ~9.4 kHz/s mechanical-frequency slew.

| `SERVO_DSHOT_RATE` | Frames | Spacing | Per-motor eRPM | RPM age | Punch lag |
|---|---|---|---|---|---|
| 0 | ~1200 Hz | 0.5 / 1 ms, uneven | 300 Hz | 9.8 ms | 92 Hz |
| 1, 2 | 800 Hz | 1.25 ms | 200 Hz | 12.9 ms | 122 Hz |
| 3 | 1200 Hz | 0.83 ms | 300 Hz | 9.6 ms | 91 Hz |
| **4** | **1600 Hz** | **0.625 ms** | **400 Hz** | **7.9 ms** | **75 Hz** |

eRPM quantisation is ≤ 0.40 % (0.12 Hz) from 1 500 to 42 000 RPM, and the slowest representable speed is 131 RPM.

### Architecture improvements
- The FC/ESC telemetry contract is now one checked chain: `hwdef.dat` (pins, BIDIR pairing) → generated `hwdef.h` (DMA) → `defaults.parm` (rates, notch) → AM32 EEPROM `motor_poles` (`am32_esc/eeprom_settings.json`) → schematic R4–R7. `validate_bdshot.py` fails if any link drifts.
- `defaults.parm` is validated against the real ArduPilot `@Param` documentation (names, `@Values`, `@Bitmask`, `@Range`), instead of trusting comments.
- AM32-side assumptions are corrected. EDT is off in AM32 until DShot command 13, and AM32 auto-detects the DShot rate, so neither is a compile-time macro. The eRPM period is independent of AM32's `motor_poles`, so only the FC's `SERVO_BLH_POLES` sets the notch frequency.

### Validation status
- `python3 tools/validate_bdshot.py --ardupilot <ArduPilot 4.7.1>`: **PASS**. V1 generation and DMA, V2 capture coverage and BDMASK, V3 all 15 parameters valid, V4 0/421 184 mismatches, V5 46/46 unit checks under ASan + UBSan, V6 all analysis gates, V7 the patch applies.
- `make -C firmware/bdshot_telem test`: **PASS** (46 checks). This includes the exhaustive GCR round trip and decoding with up to ±14 % ESC bit-clock error.
- `python3 tools/validate_dma.py`: **PASS**, unchanged DMA map with 12/16 streams.
- `python3 tools/verify_mcu_core_sch.py`: **PASS** on the Rev 10 hwdef pin set (ERC 0 errors, exact netlist with R4–R7, new S8 check). *Working-tree note:* concurrent Rev 11 edits add `PE1 IMU2_DRDY` / `PD12 IMU_CLKIN_MON` to `hwdef.dat` before the schematic has been regenerated, so S1/S6 fail until `tools/gen_mcu_core_sch.py` is re-run as part of that work.
- `tools/verify_power_stage_pcb.py`: 2 **pre-existing** failures (the Gerber file set and silkscreen re-plot have been out of step since Rev 9). Rev 10 does not touch the PCB.

### Open items
- **Hardware not yet verified:** scope the GCR reply at the H743 pin to confirm the 30 µs turnaround and edge quality through R4–R7. Fly a log with `INS_LOG_BAT_OPT` 4 (pre/post filter) to confirm notch placement.
- The notch cannot follow a full-throttle punch at any DShot rate: it lags by ≥ 75 Hz and keeps < 1 dB of depth during the ~60 ms slew. Options are a higher `SCHED_LOOP_RATE`, or splitting the motors over two timers (two captures per frame). The second needs a pin change, because PE9/11/13/14 are TIM1-only.
- Submit the `RECV_FAILED` patch upstream.
- Leave PCB placement for R4–R7 to the signal-layout revision.

---

## Revision 9.0 Engineering Changelog

**Scope:** AM32 ESC firmware target for the FD6288Q gate drivers: the dead-time generator setting and slew-rate (gate network) tuning.

### Changes implemented

**1. A real AM32 target in place of a placeholder.** The Rev ≤ 8 `am32_esc/am32_target.h` defined `DEAD_TIME_NS 60`, `PWM_FREQ_KHZ 48`, `TEMPERATURE_CUTOFF_C`, `BIDIRECTIONAL_DSHOT` and similar macros. AM32 reads none of them. The board also had no ESC MCU, and AM32 cannot run on the H743. Rev 9 picks the **AT32F421K8U7** (one per motor). AM32 has 86 F421 targets, the MCU's TMR1 runs at 120 MHz (8.33 ns dead-time resolution), and it costs under $1. The new file is a drop-in `Inc/targets.h` block, checked against AM32 v2.21 (3d8cd50):

| Macro | Value | Why |
|---|---|---|
| `FILE_NAME` / `FIRMWARE_NAME` | `NANO_H743_AIO_F421` / `NanoH743AIO` | `_F421` suffix makes `make` discover the target. The name must be ≤ 12 characters (`_Static_assert`, main.c:351) |
| `HARDWARE_GROUP_AT_B` | — | Complementary TMR1 pairs: HIN A/B/C ← PA10/PA9/PA8 (CH3/2/1), LIN A/B/C ← PB1/PB0/PA7 (CH3N/2N/1N). Signal input PB4 (TMR3_CH1) |
| `HARDWARE_GROUP_AT_045` | — | Back-EMF comparator A = PA0, B = PA4, C = PA5 |
| `DEAD_TIME` | **43** | Raw TMR1_BRK.DTC code: 43 × 8.333 ns = **358.3 ns**. Derived in section 3 |
| `TARGET_VOLTAGE_DIVIDER` | 110 | 100 k / 10 k on PA3. AM32 scales V × 10, full scale 36.3 V |
| `CURRENT_OFFSET` / `MILLIVOLT_PER_AMP` | 0 / 20 | There is no per-channel current amplifier. PA6 is grounded through 10 k, so AM32 reports 0 A |
| *not defined* | `USE_INVERTED_*`, `USE_OPEN_DRAIN_*`, `PWM_ENABLE_BRIDGE`, `USE_SERIAL_TELEMETRY` | FD6288Q **HIN and LIN are both active-high** (datasheet V1.6 §1.3: "output in phase with input"; Table 1-2). Telemetry is extended DShot telemetry over the signal wire (Rev 10) |

**2. Slew-rate tuning: the gate network is chosen by a calibrated sweep (`am32_esc/gate_drive_design.py`).** The script integrates the TPHR8504PL gate-charge curve (Qgs1 25 nC, Qgd 12.4 nC, Qg 103 nC at 10 V). The Miller plateau depends on current, temperature and Vth corner (Fig. 8.1, shifted by −8.5 mV/K and by Vth,min − Vth,typ). The FD6288Q is modelled as a resistive output (VOH/VOL ⇒ 6/9 Ω source, 3/4.5 Ω sink) **limited to its rated IOH/IOL** (1.1–1.9 A / 1.3–2.3 A, scaled down below VCC 15 V). Calibrated against Toshiba's switching test circuit, the model gives turn-on 25.9 ns vs 26 ns and turn-off 98.6 ns vs 63 ns, so it errs slow on turn-off, which is the safe side for dead time. It sweeps 99 networks (Rg_on × {single R, Rg ∥ Schottky + R_off} × Cgs_ext). Each network is checked at the fast corner (4S HV 17.4 V, 35 A, 125 °C, Vth,min; the victim FET held off through rg_int,max) against four limits:

- dv/dt ≤ 5 V/ns
- Miller shoot-through energy ≤ 10 % of the turn-on energy
- VDS peak ≤ 32 V with a 3 nH loop
- FD6288Q Tj ≤ 125 °C at a 105 °C board

Selection rule: within 10 % of the minimum feasible loss, fewest parts per FET, then lowest dv/dt. Every gate keeps a series-resistor footprint.

| Network (per FET) | Switching loss per HS, 4S 20 A 24 kHz | dv/dt | Shoot-through E_st/E_on | Result |
|---|---|---|---|---|
| **0 Ω footprint + 2.2 nF C0G Cgs (selected)** | **117 mW** | **2.85 V/ns** | **5.2 %** | Selected: best shoot-through within the loss band |
| 0 Ω + 1.0 nF Cgs | 110 mW | 2.88 V/ns | 8.9 % | Feasible, less shoot-through margin |
| 15 Ω ∥ Schottky | 145 mW | 1.21 V/ns | 9.5 % | Feasible but +24 % loss |
| 4.7 Ω single (Rev 8 thermal assumption) | 183 mW | — | > 10 % | **Rejected:** shoot-through |
| 0 Ω, no Cgs | 106 mW | > 5 V/ns | > 10 % | Rejected |

With 0 Ω, the driver's current limit sets the slew rate. Gate-loop damping is still ζ ≥ 1.19 for a 10 nH loop (check C9). The 2.2 nF Cgs divides the Cdv/dt charge: induced Vgs is 1.07 V peak against Vth,min(125 °C) 0.55 V, with 0.39 µJ shoot-through per edge.

**3. Dead time is derived, not guessed.** The FD6288Q already inserts a fixed 100/200/300 ns interlock (min/typ/max, Table 3-4), so the MCU only covers the rest:

DT_mcu ≥ toff_prop,max + t_out(→ Vth,min,125 °C | slow) − (toff_prop,min + DT_int,min + t_in(→ Vth,min | fast)) + MT 30 ns + 25 ns guard

The worst case is evaluated over both transitions, with the outgoing FET hard-switched (at 10/20/35 A) or soft, at 4S HV and 2S min:

| Bus | Transition | Outgoing soft | Outgoing hard (10 A, plateau 1.67 V) |
|---|---|---|---|
| 4S HV | HS off → LS on | 324.8 ns | 349.8 ns |
| 4S HV | LS off → HS on (negative phase current: deceleration, braking) | 328.9 ns | **353.9 ns** |
| 2S min | HS off → LS on / LS off → HS on | 239.0 / 243.8 ns | 247.9 / 252.8 ns |

The requirement is 353.9 ns, which rounds up to **`DEAD_TIME 43` = 358.3 ns**. It is in the linear DTC range (< 128), and the same encoder/decoder handles codes ≥ 128. Body-diode conduction per edge (outgoing channel off → incoming channel on) is **457 ns typ / 755 ns max**. The thermal model uses the max.

**4. EEPROM settings AM32 actually reads (`am32_esc/eeprom_settings.json`, offsets from `Inc/eeprom.h`).**
- `pwm_frequency` 24 kHz, `variable_pwm` 0, `comp_pwm` 1.
- `minimum_duty_cycle` **4**. AM32 duty runs 0..2000 of ARR, so this gives 842 ns. The AM32 default of 1 gives 208 ns, which is shorter than the 688 ns worst-case dead time, so the shortest pulses would vanish.
- `driving_brake_strength` **10**. Strengths 1–9 **OR** (`BDTR |=`, main.c:708) `DEAD_TIME + 150 − 10·s` into the register, which gives a non-monotonic 0.93–2.12 µs.
- `low_voltage_cut_off` **0**. AM32's cutoff latches `allOff()` on one ESC until power cycle, and mode 1 guesses the cell count once at arming. The FD6288Q UVLO floor is enforced by the FC instead: `BATT_CRT_VOLT 6.4` / `BATT_FS_CRT_ACT 1` (Land), added to `ardupilot/defaults.parm`.
- `limits.temperature` 115 °C. This is the uncalibrated AT32 die sensor with a last-resort step derate.
- `limits.current` 0.
- `motor_poles` 14, matching `SERVO_BLH_POLES`.

**5. Gate-drive hardware requirements (report `requirements`, for the ESC schematic sheet).**
- FD6288Q VCC = VBAT through 10 Ω, with 10 µF and an 18 V Zener. The HS bootstrap can pump to 20.9 V on a −4 V VS undershoot, against VGSS 20 V.
- Bootstrap per phase: 40 V Schottky with IR ≤ 100 µA at 105 °C, 2.2 Ω in series, and 2.2 µF/25 V. The worst-case HS on-time without refresh is 1.25 ms at 2S (≥ 2.3 k rpm at 100 % duty), 13.6 ms at 3S and 25.9 ms at 4S, including IQBS, diode leakage and the 100 k HS pull-down.
- Pull-downs: 10 k on each LS, 100 k on each HS.
- Commutation loop ≤ 3 nH.

**6. Thermal model updated (`analysis/thermal_power_analysis.py`).** It reads `gate_drive_report.json` as the single source of truth:
- HS switching energy now comes from the simulator's (I, Vbus) table instead of the optimistic Qsw/I_plateau formula.
- Body-diode window: 755 ns, not the Rev 8 value of 60 ns.
- Gate power now includes Cgs.
- RDS(on) tempco corrected from 0.0056 to 0.0072 /K (Fig. 8.9).
- Miller plateau 3.0 → 3.3 V.
- Datasheet citation corrected to Rev.5.0 (2019-10-24).

**The published per-motor ratings drop from 20 A / 28 A (typical) and 17 A / 22 A (conservative) to 20 A / 25 A and 16 A / 21 A.** The search confirms these are the largest whole-amp values that pass (17, 22 and 26 A fail).

### Architecture improvements
- **Single source of truth, closed loop.** Datasheet tables feed `gate_drive_design.py`, which writes `gate_drive_report.json`. That report feeds the thermal model, and the thermal model feeds the published ratings, which `verify_power_stage_pcb.py` P11 checks. The script also checks the header (H1–H7: DEAD_TIME, groups, polarity, names, divider, dead macros, quoted ns figures), the EEPROM file (E1–E2) and `defaults.parm` (E3, SERVO_BLH_POLES). Hand-editing any of them breaks the gate.
- **Real-toolchain build gate.** `tools/am32_build_target.sh <AM32>` copies AM32, splices the block in before AM32's "Missing defines" guard, and installs AM32's pinned xPack GCC 10.3.1. It then builds `NANO_H743_AIO_F421`, asserts 34 resolved macros from `arm-none-eabi-gcc -E -dM` (pins, comparator, clock, DEAD_TIME, absent inversion macros), and runs a SITL twin to confirm the DTC code reaches the timer's BDTR unchanged.

### Validation status

| Check | Result |
|---|---|
| `python3 am32_esc/gate_drive_design.py`: C0 calibration, C1–C3, C9 slew/damping, C4 DTG, C5–C8 supply/driver/bootstrap, H1–H7 header, E1–E3 EEPROM/FC | **PASS** (20/20) |
| Mutation tests on that gate (13 cases): old DEAD_TIME 34, stale header ns, `USE_INVERTED_HIGH`, LVC on, AM32-default min duty, brake 5, variable PWM, BATT_CRT_VOLT 5.5, FS action 0, model without IOL cap, model without LS-hard rows, 10 k HS pull-down, 40 nH gate loop | **every mutation caught**; the unmodified control passes |
| `tools/am32_build_target.sh /tmp/AM32` (AM32 v2.21) | **PASS**: 21,284 B flash (77 % of 27 KB), 3,680 B RAM, all 34 macros resolved. SITL: `dead time 268ns (BDTR 0x2b)` (SITL clock 160 MHz; on the F421 at 120 MHz, 0x2B = 358 ns) |
| `python3 analysis/thermal_power_analysis.py` | **PASS** at 16/21/20/25 A. Conservative 10 s burst ends at 109.4 °C board against a 110 °C limit: 0.6 K margin |
| `python3 tools/verify_power_stage_pcb.py` P11 (thermal report re-run identical, ratings hold) | **PASS**. The 2 P10 Gerber failures predate this revision; Rev 9 changes no PCB file |
| Independent 3-lens adversarial review (physics, AM32 semantics, thermal coupling), each finding verified by a separate refuter | 11 confirmed, 10 refuted. **All 11 fixed:** IOL/IOH current caps, LS hard-turn-off rows, corner-consistent plateau, rg_int,max on the Miller victim, bootstrap pull-down and leakage, latching LVC replaced by FC failsafe, min-duty units, thermal switching energy from the simulator, temperature-limit wording, header ns figures |
| `BATT_CRT_VOLT` / `BATT_FS_CRT_ACT` names | Present in upstream `AP_BattMonitor_Params.cpp` (Copter: 1 = Land) |
| Hardware | **Not run.** Bring-up must measure: dead time on the HO/LO pins at 4S under load (expect ≥ 358 ns MCU + driver interlock); switch-node dv/dt and VDS overshoot at 35 A (≤ 2.9 V/ns, ≤ 24 V); LS gate bounce during HS turn-on (≤ 1.1 V); AT32 die-sensor offset to the board |

**Open items.**
- **FD6288Q supply:** LCSC C328453 shows 0 in stock. The AT32F421K8U7 QFN has 39.
- **Model inputs to confirm:** the FD6288Q output resistance and the minimum turn-off propagation delay are not in the datasheet. They are derived or assumed and flagged in the report.
- **Vth tempco:** the design is sensitive to it. At −12 mV/K instead of −8.5 mV/K, no network meets the shoot-through limit. Confirm it against Toshiba data.
- **ESC schematic:** the sheet with the parts above is still to be drawn.
- **Burst margin:** the 0.6 K margin on the conservative burst rating is thin.

---

## Revision 8.0 Engineering Changelog

**Scope:** KiCad 8 6-layer PCB with a 2 oz inner copper plane stackup, and thermal vias under the 24× TPHR8504PL MOSFETs.

### Changes implemented

**1. A real 6-layer stackup in place of a label.** Before Rev 8, `nano-h743-aio.kicad_pcb` was an empty outline: six copper layers with no stackup definition, so the Gerber job file reported 35 µm everywhere. The board now carries JLCPCB's orderable **JLC061612-1080** stackup, which is written into the board and the Gerber job file:

| Layer | Copper | Dielectric to next layer | Use |
|---|---|---|---|
| L1 F.Cu | 1 oz (35 µm) | 0.1375 mm 1080×2 prepreg, εr 3.91 | FETs ESC1/2, phase pours, signals |
| L2 In1.Cu | **2 oz** (61 µm finished) | 0.35 mm core, εr 4.6 | **GND plane**. Reference for L1, HSE/VDDA return from Rev 7 |
| L3 In2.Cu | **2 oz** | 0.325 mm 1080+7628+1080, εr 4.24 | **VBAT plane** |
| L4 In3.Cu | **2 oz** | 0.35 mm core | **VBAT plane** |
| L5 In4.Cu | **2 oz** | 0.1375 mm 1080×2 | **GND plane**. Reference for L6 |
| L6 B.Cu | 1 oz | — | FETs ESC3/4, phase pours |

The stackup totals 1.614 mm, with an ENIG finish and filled-and-capped via-in-pad (POFV, IPC-4761 Type VII), which JLC includes free on 6-layer boards. Outer copper stays at 1 oz on purpose. The LQFP-100 has a 0.5 mm pitch, and 2 oz outer copper would push it to JLC's 0.15–0.2 mm 2 oz trace and space limit. GND/VBAT/VBAT/GND gives two GND–VBAT pairs 0.35 mm apart for the hot loop, two parallel 2 oz layers per rail (0.14 mΩ/sq each), and a solid GND reference directly under both outer signal layers.

**2. The 24 MOSFETs did not fit on a 25.5 mm board, so the outline moved to 36 × 36 mm with a 30.5 × 30.5 mm M3 mount.** The previous outline was 25.5 mm square with holes 20 mm apart, inconsistent with its own "25.5 mm" label. On Toshiba's land pattern, a TPHR8504PL needs 38.7 mm² of courtyard, so 24 of them need 929 mm². A 25.5 mm square has 650 mm² per side before the LQFP-100, which takes about 290 mm² more. The fit check is built into the generator, and its result made 30.5 mm-class the smallest standard frame size that works.

**3. Project footprint library (`kicad/nano-h743-aio.pretty`).** The land pattern is Toshiba's recommended SOP Advance (2-5Q1S) pattern. The research reviewer found that the LCSC/EasyEDA footprint for C5331611 is actually a Diodes PowerDI5060 footprint, so it isn't used. The pattern details:

- G/S pads: 0.85 × 1.45 mm on a 1.27 mm pitch.
- Gap to the drain: 0.7 mm.
- Drain land: 4.7 × 4.8 mm, with four 0.85 mm fingers for pins 5–8 (pin 5 sits opposite pin 4).
- Paste: 2 × 2 windowpane at 69 % coverage.
- Zone connection: solid, set on the footprint.

There are three variants:
- `…_ThermalVias_4x4`: 16 × 0.30/0.50 mm POFV vias in the drain body, on a 1.27 × 1.0 mm pitch.
- `…_SourceVias_2x2`: 4 POFV vias in source pads 2 and 3.
- Plain.

The same five-file library also holds `MountingHole_3.2mm_M3_Grommet_Keepout` and `MotorPad_4.0x1.3mm`.

**4. Power-stage layout (`tools/gen_power_stage_pcb.py`).** The layout is placed in a single frame of reference along the board edge, then rotated onto the N, S, W and E edges. Each of the 444 pad positions is checked against a closed-form transform at generation time. Six half-bridge sites each hold one top half-bridge (ESC1/2) and one mirrored bottom half-bridge (ESC3/4) at the same XY position:

- **H slot.** The top and bottom high-side drains are both VBAT, so they share one 16-via POFV array. The array connects solidly to In2/In3 and to both drain tabs. Every high-side FET therefore gets a thermal-via array, while only 96 drills are needed for 12 high-side FETs.
- **L slot.** The top and bottom low-side sources are both GND. Mirroring makes top pins 2/3 coincide with bottom pins 3/2, so 4 in-pad POFV vias serve both FETs. A row of 3 return vias just inboard, tied to both faces by 0.6 mm tracks, brings the total to **7 GND vias per low-side pair**. Without that row, the four barrels would carry about 16 A RMS combined at 20 A.
- **No drain vias on the low-side FETs, by design.** The top and bottom low-side drains are phase nodes of different ESCs, so any through via under one would short it to the other. Their heat path is the L1/L6 phase pour plus the 0.1375 mm prepreg into the In1/In4 GND plane, modelled at 8.3 K/W. Any other arrangement costs a third slot per phase, which doesn't fit.
- **Phase pours.** On the top face, the high-side sources (pins 1–3) face the low-side drain across a 0.7 mm gap, so the pour bridges them directly. On the mirrored bottom face, the gate pad sits on that side, so the pour runs through the motor-pad strip. That leaves the gap free as the bottom gate's escape channel. A second 0.7 mm channel on the far side serves the top gate.
- **Motor pads.** Each phase has a 4.0 × 1.3 mm wire pad at the board edge, inside its own pour.
- **Mounting holes.** Each M3 hole has a 6.4 mm grommet keep-out on all copper.
- **MCU area.** A 15.7 mm square is reserved for the MCU and marked on Cmts.User.

**5. Thermal relief policy: the MOSFETs get solid connections, not spokes.** A 4-spoke relief in 2 oz copper adds 9–18 K/W per layer, which is two to four times the whole via-array resistance. All Q* and motor pads therefore connect solid. This is set on the footprint and enforced by a `.kicad_dru` rule. The planes keep 0.25 mm gap / 0.4 mm spoke reliefs for any future through-hole parts, which need them to solder.

**6. Custom DRC rules (`kicad/nano-h743-aio.kicad_dru`):**
- 2 oz inner layers: clearance 0.2 mm, width 0.16 mm, hole-to-copper 0.2 mm.
- Solid zone connection on Q* and MP*.
- POFV drill must be exactly 0.30 mm.
- NPTH grommet clearance 0.5 mm.

**Defect found while building this:** the rules file originally had no units, and **KiCad then ignores the whole file without warning**. `kicad-cli` reported a clean DRC while no custom rule was active. The rule-liveness gate caught it. The fix: every value now carries `mm`, and the verifier also requires the file to parse in pcbnew's DRC engine.

**7. Thermal analysis rewritten (`analysis/thermal_power_analysis.py`).** Several defects in the Rev ≤ 7 script are fixed:

- It multiplied the **total board loss by a single θJA**, so four ESCs were treated as one junction.
- It used 1.4 mΩ, the 4.5 V RDS(on), at every gate drive.
- It reported **218 °C** at 35 A as `CERTIFIED_SAFE_UNDER_FLIGHT_AIRFLOW`.

The new model:

- **Losses per FET, 6-step complementary PWM:** HS conduction I²R·D/3 and LS I²R·(2−D)/3, with RDS(on)(Tj) iterated. Switching losses use gate-charge transition times (t_on 19.7 ns, t_off 75 ns through the FD6288Q plus 4.7 Ω Rg). Also included: Coss/Qrr, the dead-time body diode, and gate drive (Qg(V)·V·fsw).
- **Board-level losses previously missing:** VBAT/GND plane I²R, the current shunt, and the 1 oz phase bridge (from the generator geometry; the bottom-face neck is narrower).
- **Thermal network read back from the KiCad file:**
  - junction-to-case (0.88 K/W) and SAC305 joint (0.08 K/W);
  - HS: 16 barrels down 0.58 mm to the nearest VBAT plane (5.9 K/W), in parallel with prepreg (20.3 K/W), giving 4.6 K/W;
  - LS: 8.3 K/W through prepreg;
  - radial spreading in the 4 × 2 oz planes: 0.81 K/W per site;
  - board-to-air on both faces with fin efficiency;
  - battery and motor leads as pin fins **with their own I²R generation**;
  - a lumped board capacitance and wire lags for bursts.
- **Via array ablation at the legacy 35 A point:** with the vias, the HS junction sits 4.3 K above its site; with prepreg only it would be 16.4 K. The array buys **12.2 K** per FET.

**8. Ratings corrected, and AM32 changed.**
- **The model does not support the old 20 A / 35 A claim under pessimistic cooling.** At h = 50 W/m²K and 20 A, the board reaches 118 °C, which trips AM32's 115 °C cutoff.
- **Published ratings (per motor, all four loaded, 4S, D = 0.9, 24 kHz, Tamb 35 °C, 12 AWG battery / 20 AWG motor leads):**
  - Typical prop wash (h = 80): **20 A continuous, 28 A for 10 s.**
  - Pessimistic (h = 50): **17 A, 22 A.**
  - 35 A is allowed for about 4 s (2 s in the pessimistic case).
  - Static bench: about 8 A.
- **`am32_esc/am32_target.h`:**
  - `PWM_FREQ_KHZ 48 → 24`. 48 kHz adds +2.6 W at the 20 A point, and the 28 A burst then ends at 118.9 °C, past the cutoff; at 24 kHz it ends at 109.2 °C.
  - The RDS(on) comment is corrected.
  - The 38 A over-current limit is documented as a stall/short trip, not a thermal rating.

### Architecture improvements

- **Generator-first PCB.** Following the Rev 7 schematic, the footprints, DRC rules, project rules and the filled board are all generated by code. The board is **byte-deterministic**: pcbnew's random KIIDs are replaced with content-derived uuid5 values, the board is reloaded, re-saved and rewritten, and the root sheet UUID in `.kicad_pro` is pinned to the schematic. A hand edit therefore shows up as a diff against regeneration. All footprints link to the `nano-h743-aio` library, so KiCad's library-parity DRC actually runs.
- **Planes stay continuous under the via arrays.** Via pads use `remove_unused_layers`, so each GND plane sees a 0.7 mm hole antipad instead of a 0.9 mm pad antipad. Before this fix, the 1.0 mm pitch columns merged into **3.9 mm slots in In1/In4 under every high-side array**. The review found this, and the verifier now samples between every adjacent via pair. GND plane fill went from 83 % to 85 %.
- **Power budget, from the model.** At 20 A typical, the board dissipates 12.3 W:
  - FETs 5.6 W
  - shunt 2.6 W
  - phase bridges 1.6 W
  - planes 1.4 W
  - gate drive 0.5 W
  - MCU and other 0.6 W

  So **at the rated point the via arrays aren't the bottleneck; board-level heat removal is.** The next gains are a lower-value shunt, 2 oz outer copper on the bridges, and harness/airflow.

### Validation status

| Check | Result |
|---|---|
| `python3 tools/verify_power_stage_pcb.py` (P1–P11, see the script header) | **PASS (0 failures)** |
| P1 stackup / planes | Matches JLC061612-1080, 1.614 mm, ENIG. In1/In4 GND 85 %, In2/In3 VBAT 87 % filled. No slots between thermal vias |
| P2–P6 population, thermal arrays, safety, topology | 24 FETs, 12 per side. 6 × 16 VBAT vias, each inside both high-side drains. 12 low-side FETs with ≥ 7 GND vias each. None of the 138 plated holes lands in a pad of another net. 12 phases, 24 unique gates, 12 motor pads |
| P8 `kicad-cli pcb drc --severity-all` | **0 violations, 0 unconnected**, with custom rules live. A planted 0.25 mm thermal via is flagged. The DRU parses in the pcbnew engine |
| P9 / P10 reproducibility and fab outputs | Board byte-identical to regeneration with a fresh fill. Exactly 18 fab files, each content-identical to a fresh `kicad-cli` re-plot. Job file: 6 layers, 1.614 mm, ENIG, copper 35/61/61/61/61/35 µm. 138 PTH hits plus 4 × 3.2 mm NPTH |
| P11 thermal | All five published-rating scenarios PASS. The committed report matches a re-run field by field |
| Verifier mutation tests (12 runs against a repo copy) | **11/11 defects caught**, no-op control passes. The defects: thermal-via pad resized; via made blind; one thermal via deleted; Q pad set to thermal relief; inner copper 0.035 mm; stale F.Cu Gerber; missing B.Mask; drill moved; hand-edited thermal report; rating raised to 25 A; DRU units dropped |
| Independent 4-lens adversarial review (geometry/DFM, thermal/electrical, verifier mutation, KiCad tooling), each finding re-verified by a refuter | 15 confirmed findings, all fixed or modelled; 1 refuted. Critical: wire fins credited for cooling while their own I²R heating was ignored. Major: the burst verdict ignored the AM32 cutoff; P9/P10 never checked pad, drill or Gerber content; stale fills; no library nickname on footprints; non-deterministic UUIDs. Also fixed: the GND plane slots, a narrow bottom phase bridge (widened to the gate-clearance limit and modelled), and a mismatched mounting-hole library zone |
| `python3 tools/verify_mcu_core_sch.py` (Rev 7 gate) | **PASS** with the Rev 8 project file |
| Hardware | **Not built.** Bring-up: IR camera at 20 A × 4 in a prop-wash rig (expect a board of about 87 °C at 35 °C ambient); confirm AM32 reads 24 kHz; keep bench runs to ≤ 8 A per motor without airflow |

**Open items (blocking a releasable board):**
- **MCU placement.** The 15.7 mm reserve can't hold an LQFP-100 land pattern with its decoupling. Either move to STM32H743**VIH6** (TFBGA-100, 8 × 8 mm), which needs a Rev 7 pin-map regeneration, or accept a 38 mm outline.
- **Unplaced parts.** Gate drivers, gate resistors, bulk and ceramic capacitors, shunt, current sensor, battery pads, IMUs and the rest. The 24 gate nets have reserved 0.7 mm channels but no routing. ESC schematic sheets are still to be drawn.
- **Shunt.** It needs a ≥ 4 W part, or 0.25 mΩ, which saves 1.3 W.
- **Package variant.** Confirm the TPHR8504PL reel is SOP Advance (2-5Q1S), not (N) 2-5W1A.
- **Unconfirmed driver data.** FD6288Q output resistance (6 / 4 Ω assumed) and the RDS(on) temperature slope are taken from typical curves. Re-run the analysis when confirmed.
- **Release prerequisite.** Confirm with JLC that their stackup with POFV supports 2 oz inner copper.

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
