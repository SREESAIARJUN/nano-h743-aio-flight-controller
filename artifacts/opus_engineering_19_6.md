# Cycle 19: Low-Noise IMU Power Architecture Using the TPS7A4700

> **Note:** I don't have the outputs from Cycles 1–18. This design uses common defaults: 12 V system bus, 3.3 V IMU core, and 1.8 V I/O. Values that need confirmation against your earlier cycles are marked **[VERIFY]**. Check all part values against the current TI datasheet (SBVS204) before release.

---

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU rail noise (10 Hz–100 kHz) | < 10 µVrms | Keeps supply-induced bias instability below the gyro noise floor |
| Low-frequency PSRR (10 Hz–1 kHz) | > 70 dB | Rejects vibration-band and switcher subharmonic ripple |
| Ripple at the IMU pin | < 50 µVpp | Typical MEMS gyro supply sensitivity is around 0.1–1 °/hr per mV |
| Load | 10–100 mA | IMU plus local ADC/reference **[VERIFY]** |
| Isolation | Functional (noise/ground-domain) | Galvanic isolation is offered as an option in §7 |

---

## 2. Top-Level Topology

```
 12 V Bus ──► [Input EMI filter] ──► [Sync Buck, spread-spectrum] ──► 4.5 V
                                                                       │
                                     [π-filter: FB + C-L-C, f_c ≈ 5 kHz]
                                                                       │
                                                         ┌─────────────▼─────────────┐
                                                         │   TPS7A4700 (U1)          │
                                                         │   VIN=4.5V  VOUT=3.3V     │
                                                         │   ANY-OUT config          │
                                                         └─────────────┬─────────────┘
                                                                       │ 3.3V_QUIET
                                      ┌────────────────────────────────┼───────────────┐
                                      ▼                                ▼               ▼
                              [FB + 10µF] VDD_IMU          [RC] VDDA_REF     [TPS7A20 1.8V] VDDIO
                                 (core/analog)              (ADC ref)        (digital I/O)
```

**Why two stages:**
- The buck handles the efficiency drop from 12 V.
- The π-filter attenuates the switching fundamental. The TPS7A4700's PSRR rolls off above roughly 100 kHz, so this attenuation is needed.
- The LDO removes low-frequency ripple and reference noise, where its PSRR is about 80 dB or better.

---

## 3. Stage Design

### 3.1 Pre-regulator (Buck)
- **Suggested parts:** LMR33630 or TPS62130A with spread-spectrum enabled.
- **Switching frequency:** f_sw ≥ 2 MHz. This keeps harmonics well above the IMU sensor bandwidth (< 1 kHz) and makes post-filtering easier.
- **Output:** 4.5 V. This gives 1.2 V of headroom over the 3.3 V output. The TPS7A4700's PSRR degrades sharply below about 0.5 V headroom, and its dropout is about 307 mV at 1 A (much lower at IMU currents).

### 3.2 Inter-stage π-Filter
| Component | Value |
|---|---|
| FB1 | Ferrite bead, 600 Ω @ 100 MHz, R_DC < 0.1 Ω (e.g., BLM18KG601) |
| L1 | 4.7 µH shielded |
| C1, C2 | 22 µF X7R ceramic + 1 µF X7R each side |
| Damping | 1 Ω + 22 µF series R-C across C2 to suppress L-C resonance peaking |

- Expected attenuation at 2 MHz is more than 40 dB.
- The filter must have flat impedance from 1 to 10 kHz, so check that peaking is below 3 dB.

### 3.3 TPS7A4700 Core Regulator

**Output programming (ANY-OUT, no external resistors).** VOUT = 1.4 V + sum of the grounded pin weights.

| Pin | Weight | Connection |
|---|---|---|
| 6P4V | 6.4 V | Open |
| 3P2V | 3.2 V | Open |
| **1P6V** | 1.6 V | **GND** |
| 800mV | 0.8 V | Open |
| 400mV | 0.4 V | Open |
| **200mV** | 0.2 V | **GND** |
| **100mV** | 0.1 V | **GND** |

Result: 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V**.

The internal resistor matching avoids the noise contribution and temperature-coefficient mismatch of external resistors.

**Passives:**

| Ref | Function | Value | Notes |
|---|---|---|---|
| C_IN | Input | 10 µF X7R + 0.1 µF | Place within 2 mm of IN |
| C_NR | Noise-reduction / soft-start | 10 µF X7R | Gives the near-4.17 µVrms noise spec; soft-start becomes tens of ms |
| C_BYP | Feed-forward bypass | 10 µF X7R | Lowers noise gain in the 10 Hz–10 kHz band |
| C_OUT | Output | 22 µF X7R + 0.1 µF | Stable with ≥ 10 µF ceramic; size for derated value at 3.3 V bias |
| R_PG | Power-good pull-up | 100 kΩ to 3.3V_QUIET | Feeds MCU / IMU-reset gating |
| EN | Enable | From MCU GPIO via 10 kΩ, plus 100 kΩ pull-down | Allows controlled sequencing |

**Thermal check:**
- P_D = (4.5 − 3.3) × 0.1 A = 120 mW.
- With θ_JA ≈ 35 °C/W (RGW QFN with thermal pad), temperature rise is about 4 °C. This is negligible and helps IMU thermal stability.

### 3.4 Point-of-Load Branches
- **VDD_IMU (core/analog):**
  - Ferrite bead (≥ 1 kΩ @ 100 MHz, low R_DC).
  - 10 µF + 100 nF at the IMU pins.
  - Isolates IMU digital switching transients from the shared rail.
- **VDDA_REF:**
  - 10 Ω + 10 µF RC (f_c ≈ 1.6 kHz).
  - Use only if an external ADC or reference is present.
- **VDDIO 1.8 V:**
  - A secondary small LDO, e.g., TPS7A20, fed from 3.3V_QUIET.
  - If the IMU accepts 3.3 V I/O, tie directly via a ferrite bead **[VERIFY with IMU datasheet]**.

---

## 4. Noise and PSRR Budget

| Contributor | Estimate at IMU pin |
|---|---|
| TPS7A4700 intrinsic noise (10 Hz–100 kHz, C_NR = C_BYP = 10 µF) | ~4–5 µVrms |
| Buck residual at 2 MHz | 10 mVpp − 40 dB (π) − ~40 dB (LDO at MHz) − 20 dB (POL FB) ≈ **< 10 µVpp** |
| Low-frequency ripple from bus (100 Hz) | 50 mVpp − ~80 dB ≈ **5 µVpp** |
| Ground-return IR / coupling | Layout-dependent; target < 5 µV |
| **RSS total** | **≈ 7–9 µVrms**, meeting the < 10 µVrms target |

---

## 5. Grounding, Isolation, and Layout

1. **Ground split.**
   - The quiet analog ground (AGND_IMU) covers the LDO output, POL filters, and IMU.
   - It joins the power ground at a single star point under U1's GND/thermal pad.
2. **Placement.**
   - Keep the buck at least 25 mm from the IMU.
   - Do not route switch-node copper or its harmonics on the layers adjacent to the IMU.
3. **Critical capacitors.**
   - Place C_NR and C_BYP directly at the pins with a minimal loop. These nodes are high-impedance and easily pick up noise.
4. **Kelvin sensing.**
   - Route the OUT and SENSE connection as a dedicated trace from the C_OUT pad, not from the load plane.
5. **Thermal pad.**
   - Use a 3×3 via array to an internal ground plane.
   - Keep the IMU off this thermal path to avoid thermal gradients that drive gyro bias drift.
6. **Mechanical.**
   - Mount the π-filter inductor perpendicular to the IMU axis.
   - Prefer shielded parts, since magnetic coupling can affect co-packaged magnetometers.

---

## 6. Power Sequencing and Protection
1. The 12 V input feeds a TVS (SMBJ15A), then reverse-polarity protection (P-FET).
2. The buck comes up, then the MCU asserts EN on U1.
3. The C_NR soft-start ramps the output monotonically (no overshoot).
4. PG goes high, then the MCU releases the IMU reset after a ≥ 10 ms settle.
5. The TPS7A4700's internal current limit and thermal shutdown cover output faults.
6. Optionally add a polyfuse on the IMU branch.

---

## 7. Optional Galvanic Isolation Variant
If a later cycle requires true isolation, for example for a navigation unit on a noisy vehicle ground:
- Replace the buck with an **SN6505B transformer driver plus a 1:1.5 isolation transformer** (e.g., Würth 750315371), rectified to about 5 V.
- Feed that into the same π-filter and TPS7A4700 chain.
- Isolate data with an ISO7741 digital isolator (or ISO1042 for CAN).
- The LDO's high PSRR absorbs the SN6505's 410 kHz ripple. The rest of the architecture is unchanged.

---

## 8. Verification Plan (Cycle 19 Exit Criteria)

| Test | Method | Pass |
|---|---|---|
| Output noise | LNA (e.g., 60 dB gain) into a spectrum analyzer, 10 Hz–100 kHz | < 10 µVrms |
| PSRR | Injection transformer (Bode 100) at LDO input | > 70 dB at 100 Hz–10 kHz |
| Load transient | 10 → 100 mA step | < 5 mV deviation, no ringing |
| Switcher residual | 50 Ω coax probe, 500 MHz bandwidth, at IMU pin | < 50 µVpp |
| System-level | Allan variance with the IMU powered from this rail vs. a battery reference | Bias instability degradation < 5% |
| Thermal | IR camera at 100 mA, 85 °C ambient | U1 junction < 100 °C; IMU gradient < 1 °C |

---

## Open Items for Cycle 20
- Confirm the exact IMU part, its voltage rails, and its supply-sensitivity specification.
- Confirm the bus voltage range and whether galvanic isolation (§7) is mandatory.
- Decide the C_NR value as a trade-off between noise and start-up time against the system boot-time budget.