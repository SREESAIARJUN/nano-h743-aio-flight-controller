# Low-Noise IMU Power Architecture: TPS7A4700 (Design Cycle 24)

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail noise | < 10 µVrms (10 Hz–100 kHz) | Keeps supply-induced bias and ARW contribution below the IMU's intrinsic noise floor |
| Ripple at the IMU pin | < 50 µVpp at the switcher fundamental | MEMS sense electronics are sensitive to supply ripple that aliases into the band |
| Rail isolation | Galvanic (optional tier) plus filtered/star-grounded (baseline) | Blocks ground-loop and motor/actuator noise |
| Load | 50–300 mA typical (e.g., ADIS16470/ADIS1650x-class or a tactical-grade IMU) | Sets headroom and thermal limits |
| Input bus | 9–36 V vehicle/system bus | Within the TPS7A4700's 3–36 V V_IN range |

## 2. Key TPS7A4700 Characteristics Used

- **Input range:** 3 V to 36 V. **Output:** 1.4 V to 20.5 V, up to 1 A.
- **Output noise:** about 4.17 µVrms (10 Hz–100 kHz).
- **PSRR:** about 82 dB at 100 Hz, and more than 55 dB out to roughly 1 MHz. PSRR falls at higher frequencies, which drives the pre-filter requirement below.
- **Dropout:** about 307 mV at 1 A, and lower at IMU-level currents.
- **ANY-OUT™ resistorless programming:** V_OUT = 1.4 V + Σ(grounded pins). The pin weights are 0.1, 0.2, 0.4, 0.8, 1.6, 3.2, 6.4 and 6.4 V.
- **Pins:** EN, PG (power-good), and NR/SS (noise-reduction / soft-start).

## 3. Architecture Block Diagram

```
 9–36 V BUS
     │
 [TVS + reverse-polarity FET + input LC/CMC filter]
     │
 ┌───┴───────────────────────────────┐
 │ Stage 1: Sync buck pre-regulator  │  e.g. LMR33630 / LM61460
 │ Vout ≈ 5.0 V (for a 3.3 V IMU)    │  fsw fixed at 2.1 MHz, spread spectrum OFF
 │                                   │  (or synced to a system clock)
 └───┬───────────────────────────────┘
     │
 ┌───┴──────────────┐   OPTIONAL GALVANIC TIER
 │ SN6505B + xfmr   │   push-pull isolated converter → isolated 5–6 V
 │ (isolated domain)│   creates an isolated IMU ground (GND_IMU)
 └───┬──────────────┘
     │
 [Stage 2: π-filter]  ferrite bead (e.g. BLM31KN) + 2×10 µF X7R + 1 Ω damping
     │
 ┌───┴──────────────────────────────┐
 │ Stage 3: TPS7A4700               │
 │ VIN 5.0 V → VOUT 3.3 V (ANY-OUT) │
 │ PG → MCU, EN ← sequencer         │
 └───┬──────────────────────────────┘
     │
 [Local decoupling at the IMU: 10 µF + 100 nF + 1 nF]
     │
   IMU VDD (analog)        IMU VDDIO ← separate digital LDO / MCU rail
     │                                 SPI via digital isolator (ISO7741) if galvanic
   GND_IMU ── single star point ── system GND (or via 1 MΩ‖4.7 nF Y-cap if isolated)
```

## 4. Stage-by-Stage Formulation

### Stage 1: Pre-regulator
- **Headroom target:** V_IN(LDO) − V_OUT = 1.0–1.7 V. This is far above dropout, so the pass element stays out of saturation and PSRR is preserved.
- **Choice:** 5.0 V intermediate rail for a 3.3 V IMU, or 6.5 V intermediate for a 5.0 V IMU.
- **Switching frequency:** fix f_sw high (≥ 2 MHz) and outside the IMU's internal sample/demod frequencies. Disable spread spectrum or sync to a known clock, so residual spurs are deterministic and can be characterized and filtered.
- **Light-load mode:** use forced-PWM. PFM/skip mode produces low-frequency bursts that the LDO handles poorly near the IMU's band.

### Stage 2: Post-buck π-filter
- **Corner frequency:** f_c = 1/(2π√(L·C)). With a bead of roughly 1 µH effective and 20 µF, f_c ≈ 36 kHz, giving about 70 dB attenuation at 2 MHz from the second-order rolloff.
- **Damping:** add a 1 Ω series R with an electrolytic or polymer cap, or a lossy bead, to keep filter Q below 1. This prevents peaking and interaction with the LDO loop.
- **Bead current:** derate the bead's DC current. Bead inductance collapses with bias.

### Stage 3: TPS7A4700 configuration

**ANY-OUT programming (no resistor-divider noise or tolerance):**

| V_OUT | Grounded pins | Sum |
|---|---|---|
| 3.3 V | 1P6V, 0P2V, 0P1V | 1.4 + 1.6 + 0.2 + 0.1 |
| 5.0 V | 3P2V, 0P4V | 1.4 + 3.2 + 0.4 |

**Passive components:**
- **C_IN:** 10 µF X7R placed close to the pin.
- **C_OUT:** at least 10 µF ceramic, X7R, rated at 25 V or more so DC-bias derating is acceptable.
- **C_NR (NR/SS pin):** use the datasheet-recommended value, typically about 10 nF. Larger values reduce reference noise further and lengthen soft-start; this trade can be verified against the datasheet noise-vs-C_NR curve.
- **Adjustable mode (if used instead of ANY-OUT):** place a feed-forward capacitor (about 10 nF) across the upper feedback resistor to cut noise gain.

**Control and protection:**
- **EN:** drive from the sequencer so the IMU analog rail is up before or with VDDIO, per the IMU datasheet.
- **PG:** pull up to the MCU domain. Hold the IMU in reset until PG is asserted.
- **Current limit and thermal shutdown:** internal to the device.

### Thermal check
P_D = (V_IN − V_OUT) × I_LOAD = (5.0 − 3.3) × 0.3 A = 0.51 W.

The VQFN-20 (RGW) package has θ_JA of about 35–40 °C/W with a proper thermal pad and vias, giving ΔT ≈ 20 °C. This is acceptable up to about 85 °C ambient.

Keep the LDO thermally separated from the IMU. Temperature gradients drive IMU bias drift more than supply noise does.

## 5. Isolation Strategy

| Tier | Method | When to use |
|---|---|---|
| **A: Baseline** | π-filter + LDO, separate analog ground pour, star-tied at a single point near the LDO | Most systems |
| **B: Galvanic** | SN6505B isolated converter → π-filter → TPS7A4700, with ISO77xx digital isolator on SPI/DRDY/RST | Motor drives, long cable runs, ground potential differences |
| **C: Mechanical/EMI** | Shield can over the IMU and LDO, guard ring, no switching nodes under the IMU | High-vibration or high-EMI platforms |

For Tier B, the SN6505B's ripple at about 400 kHz falls where the LDO's PSRR is still about 55–60 dB. Combined with the π-filter, this yields better than 100 dB total rejection.

## 6. Noise Budget (3.3 V, 10 Hz–100 kHz)

| Contributor | Estimate |
|---|---|
| TPS7A4700 intrinsic | ~4.2 µVrms |
| Buck ripple residual (2 MHz, after π + PSRR) | < 1 µVrms equivalent (out of band) |
| Ground/layout coupling | ~2–3 µVrms (layout dependent) |
| **RSS total** | **≈ 5–6 µVrms**, within the < 10 µVrms target |

## 7. Layout Rules

1. Place the buck, π-filter and LDO in a linear flow, with the IMU at the far end. Do not route switching nodes under the IMU.
2. Connect the LDO's GND, C_NR and C_OUT returns to a quiet analog ground island, then star-tie it to system ground.
3. Use Kelvin routing from the C_OUT node to the IMU VDD, and place 100 nF and 1 nF capacitors directly at the IMU pins.
4. Use at least 9 thermal vias on the RGW pad.
5. Keep the high-impedance NR/SS trace short and guarded.

## 8. Cycle-24 Verification Plan

| Test | Method | Pass criterion |
|---|---|---|
| Output noise | LNA (e.g., 40–60 dB) + spectrum analyzer, 10 Hz–100 kHz | < 10 µVrms |
| PSRR / ripple | Inject via Bode 100 + injector; measure at the IMU pin | < 50 µVpp at f_sw |
| Load transient | 10 → 300 mA step | < 5 mV deviation, no ringing |
| IMU-level | Allan variance, supply on bench lab supply vs. this design | ARW and bias instability unchanged within 5% |
| Thermal | IR camera at max ambient/load | T_J < 125 °C, IMU gradient minimized |
| EMC | Conducted/radiated pre-scan with motors running | No IMU output spurs at f_sw or harmonics |

## 9. Decisions to Carry Forward

- 5.0 V intermediate rail (3.3 V IMU) and ANY-OUT programming.
- Fixed-frequency forced-PWM buck at 2 MHz or above.
- Damped π-filter between the buck and the LDO.
- Tier A isolation by default, with Tier B footprint provisions (SN6505B + ISO7741) populated as a DNP option until EMC results are in.

If you share the specific IMU part, the bus voltage, and the Cycle-23 findings, I can tighten the headroom, C_NR, and filter values to match.