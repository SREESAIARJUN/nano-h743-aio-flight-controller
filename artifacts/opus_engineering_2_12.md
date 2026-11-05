# Low-Noise IMU Power Supply Architecture, Cycle 2

**Core regulator:** TI TPS7A4700 (36 V, 1 A, ultra-low-noise positive LDO)

> **Scope note:** I don't have the Cycle 1 baseline. This cycle states its assumptions explicitly so they can be reconciled with your earlier work. Any datasheet values marked *(verify)* should be confirmed against the current TI datasheet (SBVS204) before freeze.

---

## 1. Design Assumptions (Cycle 2 Baseline)

| Parameter | Assumed value | Rationale |
|---|---|---|
| Input bus | 9–36 V DC (12/24/28 V nominal) | Typical vehicle or avionics bus |
| IMU | Tactical-grade MEMS (e.g., ADIS1649x/1650x class) | 3.3 V supply, about 90–150 mA typ, transient peaks up to about 250 mA |
| IMU rail | 3.3 V ±2 %, < 10 µVrms (10 Hz–100 kHz) | Supply noise must sit well below the sensor noise floor and bias-instability contributors |
| Ripple at IMU pin | < 50 µVpp at the switching frequency | Prevents aliasing into the sensor's internal ADC/decimation |
| Isolation goal | Noise isolation from digital/RF loads, with optional galvanic isolation | Galvanic isolation applies if the IMU is on a separate chassis or long harness |

---

## 2. Top-Level Architecture

```
 28V BUS
   │
 [Protection]  TVS (SMBJ36A) + reverse-polarity ideal diode (LM74700-Q1)
   │           + input EMI filter (CM choke + π-LC)
   │
 [Stage 1: Pre-regulator]  Sync buck, LMR33630 (or LM61460)
   │           Vout = 4.5 V, fsw = 2.1 MHz fixed, spread-spectrum OFF*
   │
 [Stage 2: Post-filter]  2nd-order LC: ferrite/1 µH + 22 µF X7R + damping R-C
   │
   ├──► [Stage 3a: IMU ANALOG/SENSOR RAIL] TPS7A4700 → 3.3 V ──► IMU VDD
   │         (dedicated, star-fed, nothing else on this rail)
   │
   └──► [Stage 3b: Digital/Interface rail] TPS7A2033 or separate LDO → 3.3 V
             (MCU, SPI level shifters, logic) ──► IMU VDDIO (if split)

 OPTIONAL GALVANIC VARIANT (Stage 1 replacement):
   SN6505B push-pull + Würth 750315xxx transformer → rectifier → ~5.5 V
   → same LC post-filter → TPS7A4700 (headroom increases to ~2.2 V)
```

\*Spread spectrum spreads energy into a wideband floor that the LDO handles less effectively at high frequency. A fixed frequency above the IMU's internal sampling and anti-alias bandwidth is easier to filter deterministically. Revisit this if the EMI scan fails.

---

## 3. Stage Design Details

### 3.1 Pre-regulator headroom selection (Cycle 2 change)

The intermediate rail is raised from the minimum-dropout choice (about 3.8 V) to **4.5 V**, giving 1.2 V of headroom.

- **Reason:** TPS7A47 PSRR degrades significantly as V_IN – V_OUT approaches dropout (about 307 mV at 1 A *(verify)*). Operating at about 1 V or more of headroom keeps PSRR near its rated curve across the 100 kHz–2 MHz band.
- **Cost:** P_LDO = (4.5 − 3.3) × 0.25 A = **0.30 W worst case**. With θJA of about 35 °C/W (RGW QFN with a well-stitched thermal pad *(verify)*), the junction rise is about 11 °C. This is acceptable.

### 3.2 TPS7A4700 configuration

**Output voltage (ANY-OUT, no external divider, so no divider noise):**

V_OUT = 1.4 V + Σ(grounded pins)
3.3 V = 1.4 + 1.6 + 0.2 + 0.1 → **ground pins 1P6V, 0P2V, 0P1V**. Leave all others floating.

| Component | Value | Note |
|---|---|---|
| C_IN | 10 µF X7R 16 V + 100 nF 0402 | Placed at the IN pin |
| C_OUT | 22 µF X7R 10 V (≥10 µF effective after DC bias derating) + 100 nF | Low-ESR ceramic; meets the stability requirement |
| C_NR | 1 µF to 10 µF X7R (low-leakage) | Larger values give lower 10 Hz–1 kHz noise but slower soft-start. Start with 4.7 µF and trade against the start-up time budget |
| SENSE | Kelvin trace to the IMU VDD pin | Compensates IR drop in the feed |
| EN | RC-delayed from the 4.5 V rail, or MCU-controlled | Sequencing: IMU VDD before or with VDDIO, per IMU datasheet |
| Thermal pad | ≥9 vias to the internal GND plane | |

**Expected noise performance:** about 4.2 µVrms (10 Hz–100 kHz) per datasheet *(verify conditions)*. This is well inside the 10 µVrms budget.

### 3.3 Post-filter (between buck and LDO)

- L = 1 µH shielded (or ferrite bead rated ≥ 1 A, with low DC-bias derating) and C = 22 µF give f₀ of about 34 kHz.
- At 2.1 MHz, the filter attenuates by roughly 70 dB (ideal second order) and by about **35–40 dB realistically** once parasitics are included.
- **Damping:** add 10 µF + 0.5 Ω series RC in parallel with C. This keeps the filter Q below about 1 and prevents a peak near the LDO's control-loop crossover and load-step response.

---

## 4. Ripple / Noise Budget (at 2.1 MHz)

| Point | Level | Attenuation |
|---|---|---|
| Buck output ripple | ~10 mVpp | — |
| After LC post-filter | ~150 µVpp | ~36 dB |
| TPS7A4700 PSRR @ 2 MHz, 1.2 V headroom | — | ~40 dB *(verify vs. curve)* |
| **At IMU VDD** | **~1.5 µVpp** | Target < 50 µVpp: **~30× margin** |
| IMU internal PSRR (additional) | — | Further margin, not counted |

Low-frequency path: below about 10 kHz, LDO PSRR (about 80 dB at 100 Hz–1 kHz *(verify)*) dominates. Bus disturbances such as alternator ripple and pump PWM are suppressed by >70 dB.

---

## 5. Isolation Strategy

1. **Rail isolation:** The IMU has a dedicated TPS7A4700. No other load shares its output. Digital/IO loads are on a separate LDO, so their load-step transients never modulate the IMU rail.
2. **Ground strategy:** Use a single solid ground plane, partitioned by placement rather than by cuts. The IMU and LDO sit in a "quiet zone" away from the buck's hot loop. Return currents from the buck must not flow beneath the IMU.
3. **Ferrite isolation at the IMU:** Add an optional 0402 ferrite (about 600 Ω @ 100 MHz, low DCR) plus 1 µF at the IMU VDD pin. Place the SENSE tap on the LDO side of the ferrite to avoid loop instability. Only include this if RF/EMI testing shows it is needed.
4. **Mechanical/magnetic:** Keep the buck inductor at least 15 mm from the IMU, using a shielded inductor. This avoids magnetic coupling into any co-located magnetometer and reduces stray vibration from the inductor (acoustic or magnetostrictive).
5. **Galvanic option:** The SN6505B architecture breaks ground loops for remote IMU pods. The SPI then needs a digital isolator (ISO7741).

---

## 6. Layout Rules (TPS7A4700 Specific)

- Keep the C_NR loop tight: NR pin → cap → LDO GND pin, with the shortest possible path. Noise injected here is amplified directly to the output.
- Tie the ANY-OUT pins directly to the GND pad with short traces, and keep them away from switching nodes.
- Route the SENSE trace as a guarded Kelvin line, not near the SW node or the SPI clock.
- Make all C_IN/C_OUT ground returns go to the same GND via cluster as the thermal pad.
- Place the buck SW node on the opposite board side, or at least 10 mm laterally from the LDO and IMU.

---

## 7. Cycle 2 Verification Plan

| Test | Method | Pass criterion |
|---|---|---|
| Output noise density | LNA (≥40 dB gain, e.g., battery-powered) + FFT analyzer, 10 Hz–100 kHz | < 10 µVrms integrated |
| Switching ripple | 50 Ω coax tip-and-barrel, 1 GHz BW scope or spectrum analyzer | < 50 µVpp at fsw and harmonics |
| PSRR | Injected ripple (Picotest J2120A-type injector) 10 Hz–10 MHz | Matches budget within 6 dB |
| Load transient | 10→250 mA step | ΔV < 20 mV, no ringing beyond 50 µs |
| **System-level (key test)** | IMU Allan variance: battery-powered vs. this supply, 2+ hours static | Bias instability and ARW/VRW **statistically indistinguishable** |
| Thermal | 36 V in, max load, +85 °C ambient | T_J < 110 °C |
| EMI | CISPR 25 / DO-160 per program | Pass; if it fails, revisit spread spectrum |

---

## 8. Cycle 2 Changes and Open Items for Cycle 3

**Changes this cycle:**
- LDO headroom raised to 1.2 V to preserve high-frequency PSRR.
- Damped second-order post-filter added.
- IMU VDD and VDDIO split onto separate regulators.
- Fixed-frequency buck chosen (no spread spectrum) for deterministic filtering.

**Open items:**
1. Confirm the specific IMU part, its VDD/VDDIO split, current profile, and supply PSRR. These may relax or tighten the budget.
2. Close the C_NR value against the start-up time requirement.
3. Make the galvanic isolation decision based on the harness length and grounding architecture.
4. Assess cost/area: a TPS7A20 or LT3045 could serve as an alternative if the 36 V input capability of the TPS7A47 is unnecessary downstream of the buck. The LT3045 offers better PSRR at MHz frequencies, which is worth evaluating if the Cycle 2 measurements show margin shortfall.

If you share the Cycle 1 outputs (IMU part number, input bus spec, and any measured data), I can reconcile the assumptions and tighten the budget numbers.