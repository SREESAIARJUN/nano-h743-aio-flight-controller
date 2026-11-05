# Cycle 13: Low-Noise IMU Power Architecture Using the TPS7A4700

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail | 3.3 V ±2% | Typical MEMS IMU supply, such as ADIS1647x or ICM-4268x class |
| Integrated output noise (10 Hz–100 kHz) | < 10 µVrms | Keeps supply-coupled noise well below the sensor noise floor |
| Ripple at switcher frequency | < 50 µVpp at the IMU pin | Prevents aliasing into the IMU's internal ADC/decimation chain |
| Load current | 50–150 mA (design margin to 300 mA) | IMU plus local reference/heater if present |
| Input bus | 9–36 V (vehicle/platform bus) | Within the TPS7A4700 V_IN range of 3–36 V |
| Isolation | Noise/ground isolation from digital and motor domains (galvanic optional, see §7) | Prevents coupling of bias drift and vibration rectification |

## 2. Architecture Overview

```
 Platform Bus 9–36 V
        │
  [Reverse-pol FET + TVS + CM choke]
        │
  [Stage 1: Sync Buck Pre-regulator]  → 5.0 V @ 2.2 MHz (spread-spectrum OFF for IMU rail)
        │
  [Stage 2: 2nd-order LC/π filter]    → f_c ≈ 20–30 kHz, damped
        │
  [Stage 3: TPS7A4700 LDO]            → 3.3 V analog IMU rail (VDDA)
        │
  [Stage 4: Local ferrite + decoupling at IMU]
        │
       IMU ──── separate VDDIO from digital 3.3 V/1.8 V rail
```

**Partitioning principle:** The buck converter does the efficient step-down. The passive filter attenuates high-frequency content, where the LDO's PSRR falls off. The TPS7A4700 suppresses low-frequency ripple and provides an ultra-quiet output. The IMU's digital I/O supply comes from a separate rail, so SPI edge currents never return through the analog LDO.

## 3. Stage-by-Stage Design

### 3.1 Pre-regulator (Stage 1)
- **Example part:** TPS62913 (low-noise buck with integrated ferrite-filter compensation) or LMR33630 for wide V_IN.
- **Output: 5.0 V.** This gives about 1.7 V of headroom over the 3.3 V output. The TPS7A4700 dropout is about 307 mV at 1 A, and far less at 150 mA. The extra headroom is deliberate, because PSRR degrades sharply as the LDO approaches dropout. Keep at least 1 V of headroom for full PSRR.
- **Switching frequency:** 2.2 MHz. This places the fundamental above the AM band and well above the IMU bandwidth. Disable spread-spectrum on this channel, or verify that the spread band does not fold into the IMU's sample-rate harmonics.
- **Sync:** If the IMU or system ADC has a known sample clock, synchronize the buck to an integer relationship with it, so spurs land at deterministic, filterable locations.

### 3.2 Inter-stage Filter (Stage 2)
- **Topology:** π-filter: C1 (10 µF ceramic) – L (4.7 µH shielded, low DCR) – C2 (22 µF ceramic) with a damping branch (2.2 Ω + 47 µF tantalum/polymer in parallel with C2).
- **Corner frequency:** f_c = 1/(2π√(LC)) ≈ 1/(2π√(4.7 µH × 22 µF)) ≈ 15.7 kHz.
- **Attenuation at 2.2 MHz:** about 80 dB theoretical, about 40–50 dB realistic once ESL and parasitics are included.
- **Damping:** Required to prevent a Q-peak at f_c. An undamped LC can amplify load transients and destabilize the downstream LDO input impedance. Target Q ≤ 1.

### 3.3 TPS7A4700 Configuration (Stage 3)

**Output voltage via the ANY-OUT pins.** No external resistors are needed, which removes resistor noise and tempco error.

V_OUT = 1.4 V + Σ(grounded pin weights)

Target 3.3 V = 1.4 + 1.6 + 0.2 + 0.1, so:

| Pin | Weight | Connection |
|---|---|---|
| 1P6V | 1.6 V | **GND** |
| 200mV | 0.2 V | **GND** |
| 100mV | 0.1 V | **GND** |
| 400mV, 800mV, 3P2V, 6P4V | — | Float |

**Key components:**

| Ref | Value | Function |
|---|---|---|
| C_IN | 10 µF X7R, 25 V, plus 100 nF | Input bypass, placed at the IN pin |
| C_OUT | 10–22 µF X7R ceramic | Stability (≥10 µF required); lowers HF output impedance |
| C_NR | 10 µF X7R/film, low-leakage | Filters reference noise. Achieves ~4 µVrms; also sets soft-start |
| C_BYP (FB-to-OUT) | 10 nF C0G | Feed-forward. Improves noise and PSRR in the 1–100 kHz band |
| EN divider | Sized for ~4.5 V rising threshold | Holds the LDO off until the pre-regulator is in regulation |
| PG pull-up | 100 kΩ to MCU 3.3 V | Power-good flag gates the IMU reset/initialization |

**Expected performance:**
- **Noise:** About 4.2 µVrms (10 Hz–100 kHz) per the datasheet with C_NR = 10 µF.
- **PSRR:** About 80 dB at low frequency, about 50–60 dB at 10–100 kHz, and degrading above 1 MHz. Stage 2 covers that region.
- **Combined ripple rejection at 2.2 MHz:** Filter (≈45 dB) plus LDO (≈40 dB) gives about 85 dB. A 20 mVpp buck ripple is therefore reduced to roughly 1 µVpp.

**Thermal check:**
P_D = (5.0 − 3.3) × 0.15 A ≈ 0.26 W.
With θ_JA ≈ 35 °C/W for the QFN-20 with thermal pad, ΔT ≈ 9 °C. This is acceptable. Tie the pad to an analog ground pour with thermal vias.

### 3.4 Point-of-Load at the IMU (Stage 4)
- Use a ferrite bead with low DC resistance (for example, 600 Ω at 100 MHz, R_DC < 0.1 Ω) only if testing shows RF pickup on the trace. Otherwise, connect directly to avoid adding a resonance.
- Place 1 µF + 100 nF (X7R/C0G) at the IMU VDD pins, as recommended in the IMU datasheet.
- Keep the trace from LDO to IMU short and route it over a solid analog ground plane.

## 4. Noise Budget

| Source | Contribution at IMU VDD (10 Hz–100 kHz) |
|---|---|
| TPS7A4700 intrinsic | ~4.2 µVrms |
| Residual buck ripple (post-filter, post-LDO) | ~0.5–1 µVrms equivalent |
| Ground-bounce / layout coupling | < 2 µVrms (layout-dependent) |
| **RSS total** | **≈ 5 µVrms** (target < 10 µVrms ✓) |

Map this budget against the IMU's PSRR. Most MEMS IMUs have 40–60 dB internal supply rejection. Supply noise of 5 µVrms therefore contributes negligibly compared with sensor angle random walk and velocity random walk. Verify this with an Allan variance comparison (§8).

## 5. Grounding and Isolation Strategy
1. **Split domains:** AGND (LDO, IMU analog, reference) and DGND (MCU, SPI, motor drivers) planes.
2. **Single tie point:** Join them at one point near the LDO GND pin or the system power entry, not under the IMU.
3. **Return currents:** The buck's switching loop must stay entirely in the power ground region. Do not route it under the IMU or LDO.
4. **Separate VDDIO:** Supply the IMU logic from a separate digital LDO. Add series resistors of 22–33 Ω on SPI lines to slow edges and reduce injected current.
5. **Mechanical:** Place the IMU away from inductors to avoid magnetic coupling. Use a shielded inductor in Stage 2, and keep the buck at least 20 mm from the IMU.

## 6. Sequencing and Protection
- **Order:** Pre-regulator stable → EN threshold reached → TPS7A4700 soft-start. The ramp is set by C_NR (≈ tens of ms with 10 µF).
- **Reset gating:** PG asserted → MCU releases the IMU reset → IMU self-test.
- **Overcurrent and thermal:** The TPS7A4700 has internal current limit and thermal shutdown. Add a 0.5 A PTC or eFuse upstream for harness faults.
- **Transients:** Add a TVS on the bus input sized for the platform's surge standard (for example, ISO 7637 or MIL-STD-1275).

## 7. Galvanic Isolation (Optional Variant)
If the IMU must be galvanically isolated from the platform (for example, ground-loop-prone harnesses):
- Replace Stage 1 with a push-pull isolated converter: SN6505 + 1:1.3 transformer + rectifier, producing about 5.5–6 V.
- Keep Stages 2 and 3 unchanged. The TPS7A4700 then cleans the isolated converter's ripple, typically 410 kHz with its harmonics.
- Isolate data with a digital isolator such as ISO7741 for SPI. Note that the isolator adds its own supply transients, so power its IMU side from a separate secondary LDO.

## 8. Verification Plan (Cycle 13 Exit Criteria)

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise density | Low-noise preamp (e.g., 40 dB LNA) + FFT analyzer, 10 Hz–100 kHz | < 10 µVrms integrated |
| Switching spur | Spectrum analyzer at 2.2 MHz and harmonics | < 50 µVpp at IMU pin |
| PSRR | Inject a sine via line injector across 10 Hz–10 MHz | ≥ 60 dB at 100 kHz combined |
| Load transient | 10 → 150 mA step | < 10 mV deviation, no ringing |
| Stability across temperature | −40 °C to +85 °C, with ceramic capacitor derating | Phase margin > 45° (via load-step ringing analysis) |
| IMU impact | Allan variance: bench linear supply vs. this design | No measurable change in ARW, VRW, or bias instability |
| EMC | Conducted emissions on the bus | Meets the platform standard |

## 9. Risks and Mitigations

| Risk | Mitigation |
|---|---|
| Ceramic capacitor DC-bias derating lowers effective C_OUT | Use 25 V-rated parts for a 3.3 V rail; verify effective capacitance stays ≥ 10 µF |
| π-filter resonance interacting with the LDO input | Use the damping branch; measure input impedance with a VNA |
| C_NR leakage raising noise or offset | Use low-leakage X7R or film; avoid Y5V |
| Low headroom at cold crank (bus sag) | EN-based UVLO; the IMU tolerates a brief brownout via PG-driven reinitialization |
| Piezoelectric (microphonic) noise from ceramic capacitors under vibration | Use C0G, or tantalum/polymer for C_NR, in high-vibration installations, since vibration directly affects IMU data |

## 10. Cycle 13 Deliverables
1. Schematic of Stages 1–4 with the ANY-OUT strapping for 3.3 V.
2. Layout constraints document covering AGND/DGND tie, keep-out zones, and thermal vias.
3. Noise and PSRR simulation (TINA-TI / PSpice model of the TPS7A47) against bench measurements.
4. Allan variance comparison report as the gating criterion for Cycle 14.

If you share the specific IMU part number, the platform bus voltage, and the findings from earlier cycles, I can refine the headroom, filter corner, and noise budget against that sensor's actual supply sensitivity.