# Low-Noise IMU Power Supply Architecture Using the TI TPS7A4700 (Cycle 17)

I don't have the outputs of Cycles 1–16, so this design assumes a typical baseline. Where earlier cycles fixed a parameter (input bus voltage, IMU part, current budget, isolation requirement), substitute it in the values marked **[confirm]**.

---

## 1. Design Assumptions

| Parameter | Assumed value | Notes |
|---|---|---|
| System input bus | 12 V nominal (9–18 V) | **[confirm]** |
| IMU | MEMS IMU, e.g. ADIS16470-class (3.3 V, ~45 mA) or ICM-42688-class (1.8 V / 3.3 V VDDIO) | **[confirm]** |
| IMU rail current | ≤150 mA, including 3× margin | |
| Target rail noise | <10 µVrms (10 Hz–100 kHz) | |
| Target spur level | <5 µV pk at any switching frequency | |
| Isolation | Rail isolation from digital/motor noise (Option A). Galvanic isolation is optional (Option B). | **[confirm]** |

---

## 2. TPS7A4700 Key Characteristics

Verify all values against the current datasheet (SBVS204).

- Input range: 3 V to 36 V. Output: 1.4 V to 20.5 V via ANY-OUT pins, higher with external resistors.
- Output current: up to 1 A.
- Output noise: ~4.17 µVrms (10 Hz–100 kHz).
- PSRR: ~82 dB at 100 Hz, and still >55 dB around 1 MHz with adequate headroom.
- ANY-OUT programming: V_OUT = 1.4 V + sum of the grounded pins (0P1V, 0P2V, 0P4V, 0P8V, 1P6V, 3P2V, 6P4V).
- Other features: EN pin, open-drain PG output, thermal shutdown, current limit.
- Package: 20-pin 5×5 mm VQFN with thermal pad.

---

## 3. Top-Level Architecture

### Option A — Non-isolated, rail-isolated (baseline)

```
12 V bus ──► Input protection ──► Sync buck pre-regulator ──► π-filter ──► TPS7A4700 #1 (3.3 V) ──► FB + local C ──► IMU VDD (analog/core)
             (TVS, rev-pol,        (e.g. TPS62913 /           (FB + C)  │
              LC EMI filter)        LMR33630, set to 4.5 V,              └► TPS7A4700 #2 or small LDO (1.8/3.3 V) ──► IMU VDDIO
                                    spread-spectrum OFF                       (optional; separates digital I/O current)
                                    or fixed, sync'd freq)
```

### Option B — Galvanically isolated (if the IMU ground must float from system ground)

```
12 V ──► Isolated DC-DC (SN6505B push-pull + transformer, or isolated module) ──► rectifier ──► ~5 V ISO
     ──► π-filter ──► TPS7A4700 (3.3 V) ──► IMU
     IMU data crosses the barrier via digital isolator (e.g. ISO7741 for SPI) powered from the same isolated rail
```

### Why a two-stage design

The buck converter provides efficiency and a coarse drop. The π-filter attenuates high-frequency switching content, which is the range where LDO PSRR weakens. The TPS7A4700 then provides broadband rejection and an ultra-low intrinsic noise floor.

---

## 4. Stage Design

### 4.1 Pre-regulator

- **V_OUT = 4.5 V.** This gives 1.2 V of headroom above 3.3 V. PSRR of the TPS7A4700 degrades sharply below about 0.5–1 V of headroom, while too much headroom wastes power.
- **Switching frequency: ≥2 MHz**, set to a fixed value. Place it well above the IMU bandwidth and away from the IMU's internal sense/resonator frequencies (check the IMU datasheet). Avoid spread-spectrum unless EMI requires it, because it smears spurs into the noise floor.
- **Preferred part: TPS62913.** It has an integrated ferrite-filter topology aimed at low-noise LDO-less or post-LDO use.

### 4.2 Inter-stage π-filter

- **Topology:** C1 (10 µF X7R) → ferrite bead (e.g. BLM18KG601, 600 Ω at 100 MHz, low DCR) or 1–2.2 µH shielded inductor → C2 (10 µF X7R + 100 nF).
- **Damping:** add 0.5–1 Ω ESR (series resistor or electrolytic/polymer in parallel) to damp the LC resonance. Keep the resonance well below f_sw and away from the IMU sensitive band.
- **Target attenuation:** ≥30–40 dB at f_sw.

### 4.3 TPS7A4700 (main IMU rail, 3.3 V)

| Item | Value / setting |
|---|---|
| V_OUT programming | Ground 1P6V + 0P2V + 0P1V → 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V** |
| 1.8 V variant (if VDDIO) | Ground 0P4V → 1.4 + 0.4 = 1.8 V |
| C_IN | 10 µF X7R ceramic, close to IN |
| C_OUT | 10 µF X7R ceramic (≥ datasheet minimum), plus 100 nF near the IMU |
| C_NR (noise reduction) | 10 µF low-leakage ceramic per datasheet low-noise condition. A larger C_NR lowers noise but slows startup. |
| C_BYP (bypass) | 10 µF per datasheet typical-noise configuration **[verify pin naming and value in datasheet]** |
| EN | Tie to sequencing logic, or IN via RC for a delayed start |
| PG | Pull up to the MCU I/O rail (10–100 kΩ). Use it to hold IMU reset until the rail is in regulation. |
| Unused ANY-OUT pins | Leave floating per datasheet |

**Thermal check:**
P_D = (4.5 − 3.3) V × 0.15 A = 0.18 W. With θJA ≈ 35–40 °C/W, the temperature rise is about 7 °C. This is negligible, which also minimises thermal drift coupling into the IMU.

### 4.4 IMU local decoupling

- 100 nF + 1 µF at each VDD pin, within 2 mm.
- Optional ferrite bead between the LDO output and the IMU VDDIO to keep digital switching currents off the analog rail when both share one LDO.
- Follow the IMU datasheet's specific capacitor requirements (e.g. internal regulator caps such as REGOUT/VDDA).

---

## 5. Noise Budget (Cascade Estimate)

| Source | At buck output | π-filter | LDO PSRR | At IMU |
|---|---|---|---|---|
| Switching ripple @ 2.2 MHz | 10 mV pk | −35 dB | −40 dB | ~0.2 µV pk |
| Buck low-freq noise (10 Hz–100 kHz) | ~100 µVrms | ~0 dB | −70 dB | ~0.03 µVrms |
| LDO intrinsic | — | — | — | ~4.2 µVrms |
| **Total (RSS)** | | | | **≈4.2 µVrms, dominated by LDO intrinsic** |

Result: the output meets the <10 µVrms target with roughly 2× margin, and switching spurs are below 1 µV.

---

## 6. Power Isolation and Grounding Strategy

1. **Dedicated rail.** No other loads sit on the IMU TPS7A4700 output. MCU, radios, and motor drivers use separate regulators.
2. **Ground.** Use a solid unbroken ground plane under the IMU and LDO, with no split planes under signal traces. Keep the buck's hot loop (switch node, input cap) physically distant and on the opposite board side if possible. Return buck currents locally.
3. **Star distribution.** The pre-regulator output branches to the IMU path before any other loads, or the IMU gets its own pre-regulator tap.
4. **Option B (galvanic).** Keep the isolated domain island ≥2 mm creepage (more if the safety standard requires it). Use a stitching capacitor (e.g. 100 pF–1 nF Y-cap) across the barrier for EMI only if allowed.
5. **Mechanical/thermal.** Keep the LDO away from the IMU footprint (≥10 mm) so its heat doesn't create gradients that affect gyro bias. Do not place the buck inductor under or next to the IMU, since magnetic fields can couple into the magnetometer if present.

---

## 7. Sequencing and Protection

- **Startup order:** Pre-regulator → TPS7A4700 EN, delayed ~1–5 ms → PG asserted → MCU releases IMU reset/CS → IMU configured after its datasheet start-up time.
- **VDDIO ordering:** If VDDIO is separate, follow the IMU's required order. Typically VDD before or with VDDIO, and VDDIO must not exceed VDD + 0.3 V.
- **Protection:** Input TVS on the 12 V bus, reverse-polarity FET, and the LDO's internal current limit/thermal shutdown. An optional eFuse (e.g. TPS2595) protects against a shorted IMU harness.

---

## 8. BOM Summary (Option A)

| Ref | Part | Function |
|---|---|---|
| U1 | TPS62913 (or LMR33630) | 12 V → 4.5 V buck |
| L1/FB1 | 2.2 µH shielded / BLM18KG601 | π-filter |
| C1, C2 | 10 µF X7R 25 V/10 V | π-filter caps |
| R_damp | 1 Ω 0402 | Filter damping |
| U2 | TPS7A4700RGWR | 3.3 V ultra-low-noise LDO |
| C_IN, C_OUT, C_NR, C_BYP | 10 µF X7R/C0G-grade low-leakage | LDO network |
| R_PG | 47 kΩ | PG pull-up |
| U3 (opt.) | TPS7A4700 or TPS7A20 | VDDIO rail |
| D1 | SMBJ18A | Input TVS |

---

## 9. Verification Plan for This Cycle

1. **Noise:** Measure 10 Hz–100 kHz rms noise at the IMU pin using an LNA (e.g. 40 dB gain) and a spectrum analyzer. Target <10 µVrms.
2. **Spurs:** Measure with an FFT up to 10 MHz. The f_sw fundamental and harmonics should be <5 µV.
3. **PSRR:** Inject a signal at the LDO input with a line injector (e.g. Picotest J2120A) and confirm ≥60 dB at 1 kHz–100 kHz.
4. **Load transient:** Step IMU current 10→60 mA. Deviation should be <5 mV with no ringing (this checks C_OUT stability).
5. **IMU-level test:** Compare Allan deviation (bias instability, ARW) on this supply against a bench linear supply or battery. The difference should be within measurement noise. This is the definitive pass criterion.
6. **Thermal:** Use an IR camera at maximum load and ambient. Confirm the LDO temperature rise is <15 °C and there is no gradient across the IMU footprint.

---

## 10. Open Items to Close Before Cycle 18

- Confirm the IMU part number and actual VDD/VDDIO currents to finalise ANY-OUT settings and headroom.
- Decide between Option A and Option B (galvanic).
- Confirm the IMU's internal clock and resonant frequencies to finalise buck f_sw selection.
- Confirm C_NR/C_BYP values and pin names against the latest TPS7A4700 datasheet revision.

If you share the outputs or constraints from earlier cycles, I can align this design with them.