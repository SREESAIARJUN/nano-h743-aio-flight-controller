# Low-Noise IMU Power Supply Architecture Using the TI TPS7A4700 (Cycle 26)

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail noise | < 10 µVrms (10 Hz–100 kHz) | Keep the supply contribution below the IMU's intrinsic noise floor |
| Switching ripple at IMU pins | < 50 µVpp | Prevents aliasing into gyro/accel bandwidth |
| PSRR at buck f_sw | > 60 dB combined (LDO + filter) | Attenuates converter fundamental and harmonics |
| Isolation | Galvanic (option A) or noise-domain (option B) | Breaks ground loops from vehicle or system bus |
| Load | 50–200 mA per IMU (e.g., ADIS16470 ~44 mA, tactical-grade up to ~200 mA) | Sized with margin |
| Input bus | 9–36 V (12 V / 24 V / 28 V systems) | Typical avionics and robotics bus |

---

## 2. Top-Level Architecture

```
 VBUS (9–36 V)
   │
 [Protection: TVS + reverse-polarity FET + eFuse (TPS2660)]
   │
 [EMI input filter: CM choke + π LC]
   │
 ┌─┴──────────────────────────────────────────────┐
 │ OPTION A: Galvanic isolation                    │
 │ Pre-reg buck → 5 V → SN6505B push-pull driver   │
 │ + isolation transformer → rectifier → ~6 V iso  │
 ├─────────────────────────────────────────────────┤
 │ OPTION B: Noise-domain isolation only           │
 │ Sync buck (LMR33630 / TPS62130) → 4.0 V         │
 │ spread-spectrum OFF, f_sw synced/fixed          │
 └─┬───────────────────────────────────────────────┘
   │
 [Post-filter: ferrite bead (high-Z @ f_sw) + 10 µF + 2nd-stage LC]
   │
 ┌─┴─────────────────┐        ┌─────────────────────┐
 │ TPS7A4700 #1      │        │ TPS7A4700 #2 (or     │
 │ IMU VDD (analog)  │        │ TPS7A20) IMU VDDIO   │
 │ 3.3 V, ANY-OUT    │        │ 1.8 V / 3.3 V        │
 └─┬─────────────────┘        └─┬───────────────────┘
   │ Final RC/π filter            │
   ▼                              ▼
 IMU VDD ─────────── IMU ─────── VDDIO
                       │
          [ISO7741 digital isolator on SPI + DR/SYNC] (Option A)
                       │
                  Host MCU/FPGA
```

**Isolation strategy:**
- **Option A (galvanic):** Use this when the IMU chassis ground differs from the host ground, for example in long harnesses, vehicle frames, or motor-drive environments. The isolated return is AGND_ISO. All SPI, data-ready, and sync lines cross through ISO7741 or ISO6741.
- **Option B (noise-domain):** Use this for a single-board design. Isolation is achieved through separate regulation, filtering, and a partitioned ground, with a single-point tie at the LDO GND.

---

## 3. TPS7A4700 Configuration

### 3.1 Key Device Parameters (verify against the current datasheet)
- V_IN: 3 V–36 V; V_OUT: 1.4 V–34 V; I_OUT up to 1 A
- Output noise: ~4.17 µVrms (10 Hz–100 kHz)
- PSRR: ~82 dB at low frequency, still >55 dB in the hundreds of kHz with adequate headroom
- Dropout: ~307 mV at 1 A (much lower at IMU currents)
- Package: 5 × 5 mm VQFN-20 (RGW) with thermal pad

### 3.2 ANY-OUT Voltage Programming (no external resistors, so no resistor noise)
V_OUT = 1.4 V + Σ(grounded ANY-OUT pins)

| Rail | Pins tied to GND | Result |
|---|---|---|
| 3.3 V (IMU VDD) | 1P6V + 0P2V + 0P1V | 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V** |
| 5.0 V (some tactical IMUs) | 3P2V + 0P4V | 1.4 + 3.2 + 0.4 = **5.0 V** |
| 1.8 V (VDDIO) | 0P4V | 1.4 + 0.4 = **1.8 V** |

Leave unused ANY-OUT pins floating. Do not connect them to OUT.

### 3.3 External Components

| Component | Value | Notes |
|---|---|---|
| C_IN | 10 µF X7R + 0.1 µF, 50 V rated | Place close to the IN pins |
| C_OUT | 10–47 µF X7R ceramic (≥ 10 µF required) | Low ESR; use a voltage rating ≥ 2× V_OUT to limit DC-bias derating |
| C_NR/SS | 10 µF X7R | Sets the noise-reduction corner low. Soft-start ≈ tens of ms, which is acceptable for an IMU. |
| C_BYP | 10 µF (per datasheet guidance) | Lowers 1/f noise from the error amplifier |
| EN | Driven by the sequencer or PG of the upstream rail, with a 100 kΩ pull-down | Prevents an undefined start |
| PG | 10 kΩ pull-up to VDDIO | Routed to the MCU and the IMU RST gating logic |
| SENSE | Kelvin trace to the IMU-side output cap | Compensates IR drop |

### 3.4 Headroom and Thermal Budget
- V_IN(LDO) = 4.0 V for a 3.3 V output, giving ~700 mV of headroom. This keeps the pass device out of dropout so PSRR is preserved at f_sw.
- P_D = (4.0 − 3.3) × 0.2 A = **0.14 W**.
- ΔT ≈ 0.14 W × ~35 °C/W ≈ 5 °C. This is negligible and limits thermal gradients near the IMU, which matters because gyro bias is temperature sensitive.

---

## 4. Pre-Regulator and Filtering

### 4.1 Buck Converter Selection
- **Fixed frequency, spread spectrum disabled.** A known spur is easier to filter and verify than a smeared one.
- **f_sw ≥ 2 MHz** where efficiency permits. This places the fundamental far above the IMU sensor bandwidth (typically < 1 kHz) and in the region where the ferrite bead attenuates strongly.
- **Optional synchronization.** Sync to a multiple of the IMU sample clock so any residual spur lands at a deterministic, filterable frequency.

### 4.2 Inter-Stage LC Filter (buck → LDO)
- Ferrite bead: e.g., BLM18KG601 (600 Ω @ 100 MHz, low DCR), followed by a 10 µF capacitor.
- Second stage: 2.2 µH + 22 µF gives a corner of ~23 kHz.
- Damping: add an RC (1 Ω + 22 µF) to suppress the LC Q peak.
- Combined attenuation at 2 MHz: ~40 dB (LC) + ~55 dB (LDO PSRR) ≈ **>90 dB**.

### 4.3 Final Point-of-Load Filter at the IMU
- 1–2.2 Ω series resistor + 10 µF + 0.1 µF at the IMU VDD pins.
- This decouples the IMU's own transient current (ADC/DSP bursts) from the LDO loop.

---

## 5. Noise Budget (3.3 V IMU Rail, 10 Hz–100 kHz)

| Source | Contribution |
|---|---|
| TPS7A4700 intrinsic | ~4.2 µVrms |
| Residual buck ripple after >90 dB attenuation (20 mVpp in) | < 1 µVrms equivalent |
| Ground/layout coupling (estimate) | ~2 µVrms |
| **RSS total** | **≈ 4.8 µVrms** (meets the < 10 µVrms target) |

---

## 6. Power Sequencing

1. VBUS is valid and the eFuse PG asserts.
2. The buck or isolated converter starts. Its PG enables TPS7A4700 #1 (VDD).
3. TPS7A4700 #1 PG enables the VDDIO LDO, or follow the IMU datasheet if it requires the opposite order.
4. Both PGs are ANDed to release IMU RST after the IMU-specified start-up delay (e.g., ~250 ms for the ADIS16470).
5. Power-down runs in reverse order. Hold the SPI lines low or high-Z to avoid back-powering the IMU through its I/O pins.

---

## 7. PCB Layout Rules

- **Ground partitioning.** AGND_IMU connects to the LDO GND at a single point. In Option A, AGND_ISO is fully separated by a ≥ 4 mm creepage gap under the transformer and isolators.
- **Buck placement.** Keep the buck ≥ 25 mm from the IMU. Orient the inductor's field axis away from the IMU, and use shielded inductors.
- **Thermal pad.** Use at least 9 thermal vias under the TPS7A4700 pad into an internal ground plane. Avoid a direct copper thermal path to the IMU footprint.
- **NR/SS and BYP capacitors.** Use the shortest possible loops. These nodes are high impedance and noise sensitive.
- **Mechanical.** Do not route switching traces under the IMU. Avoid high-mass components on the IMU mounting area, since vibration couples into the sensor.
- **Optional shielding.** Use a can over the buck stage. Guard-ring the LDO output trace.

---

## 8. Verification Plan (Cycle 26 Exit Criteria)

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise density | LNA (e.g., 40 dB) + FFT analyzer, 10 Hz–100 kHz | < 10 µVrms integrated |
| Spur at f_sw | Spectrum analyzer at the IMU VDD pin | < −100 dBV |
| Load transient | 10 → 200 mA step, 1 µs edge | < 10 mV deviation, no ringing > 2 cycles |
| Line transient | VBUS 12 → 28 V step | No measurable IMU output disturbance |
| IMU-level test | Allan variance with the IMU stationary, comparing a bench linear supply vs. this design | Bias instability and ARW within 5% of the bench-supply baseline |
| Thermal | IR camera, full load, 85 °C ambient | LDO T_J < 110 °C; gradient across the IMU < 1 °C |
| Isolation (Option A) | Hi-pot test | Per the system spec (e.g., 1.5 kVrms for 60 s) |

---

## 9. Bill of Materials Summary

| Function | Part | Qty |
|---|---|---|
| Ultra-low-noise LDO | TPS7A4700RGWR | 1–2 |
| Pre-regulator buck | LMR33630 or TPS62130 | 1 |
| Isolated supply (Option A) | SN6505B + Würth 750315240 transformer | 1 |
| Digital isolator (Option A) | ISO7741 | 1 |
| Input protection | TPS2660 eFuse + SMBJ36A TVS | 1 each |
| Ferrite bead | BLM18KG601SN1 | 2–3 |
| Precision ceramics | X7R/C0G per the tables above | — |

---

**Decision needed this cycle:** confirm Option A or Option B, based on whether the IMU's mounting ground is shared with the host. If the IMU sits on a separate mechanical structure or connects through a harness longer than ~0.5 m, Option A is recommended.