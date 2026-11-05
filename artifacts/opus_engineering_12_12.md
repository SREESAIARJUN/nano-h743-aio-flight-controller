# Low-Noise IMU Power Architecture: TPS7A4700 Post-Regulated Isolated Rail (Cycle 12)

I don't have the context from Cycles 1–11, so I assumed a typical baseline: a 9–36 V vehicle or avionics bus and a tactical-grade MEMS IMU (ADIS1649x-class, 3.3 V, about 90–160 mA). Swap in your actual values where they differ. The architecture holds; only the numbers shift.

---

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU rail voltage | 3.3 V ±2 % | IMU VDD spec |
| Load current | ≤ 200 mA (peak), ~150 mA typ | Includes inrush at IMU boot |
| Integrated noise (10 Hz–100 kHz) | < 10 µV RMS at IMU pins | Keeps supply-induced bias instability below the IMU's intrinsic noise floor |
| Ripple at switcher fundamental | < 20 µV pp | Avoids aliasing into gyro/accel bandwidth |
| PSRR (system, 100 Hz–2 MHz) | > 80 dB combined | Rejects bus transients and buck ripple |
| Fault isolation | IMU rail survives faults elsewhere | Containment of shorts and brownouts |

---

## 2. Block Architecture

```
 VBUS (9–36 V)
   │
 [Stage 0] Input protection: TVS (SMBJ36A) + reverse-polarity ideal diode (LM74610) + CM choke
   │
 [Stage 1] Buck pre-regulator: LMR36015 (or TPS62912 if VIN ≤ 17 V)
   │         VOUT = 4.3 V, fsw = 2.2 MHz (above the IMU band, spread-spectrum OFF or synced)
   │
 [Stage 2] Passive π-filter: 10 µF ─ FB (BLM31KN601, 600 Ω@100 MHz) ─ 22 µF + 1 Ω/10 µF damping
   │
 [Stage 3] TPS7A4700 ultra-low-noise LDO → 3.3 V_IMU (dedicated, IMU-only)
   │
 [Stage 4] Local decoupling at IMU: 1 µF + 100 nF per VDD pin, ferrite bead optional
   │
  IMU (VDD)          Separate rail → MCU / VDDIO (do NOT share)
```

**Optional galvanic isolation**, used when the IMU ground must float from the vehicle chassis (noisy motor drives, ground loops):

```
VBUS → buck → SN6505B push-pull + 1:1.3 transformer (Würth 750315371) → rectify → π-filter → TPS7A4700 → IMU
SPI/DRDY crossing: ISO7741 (or ISO7763 for higher SPI speed)
```

The TPS7A4700's high PSRR at 400 kHz–1 MHz is what makes the noisy push-pull stage acceptable.

---

## 3. TPS7A4700 Configuration

### 3.1 Key device data (datasheet, typical)
- VIN: 3 V–36 V; VOUT: 1.4 V–20.5 V; IOUT up to 1 A
- Noise: 4.17 µV RMS (10 Hz–100 kHz)
- PSRR: 82 dB at 100 Hz, ≥ 55 dB at 1 MHz
- Dropout: ~307 mV at 1 A (much lower at 150 mA)
- Package: VQFN-20 (RGW, 5×5 mm) with exposed thermal pad

### 3.2 Output voltage (ANY-OUT, no external resistors)
The output equals the 1.4 V reference plus the weights of any ANY-OUT pins tied to GND:

**3.3 V = 1.4 + 1.6 + 0.2 + 0.1 V**, so tie the **1.6 V, 200 mV, and 100 mV** pins to GND and leave the rest open.

ANY-OUT avoids feedback-resistor thermal noise and tempco mismatch, which are both important for gyro bias stability over temperature.

### 3.3 Headroom selection
- The dropout at 200 mA is about 100 mV, but PSRR degrades sharply below about 500 mV headroom.
- The chosen VIN is 4.3 V, giving 1.0 V headroom. This preserves PSRR above 1 MHz with margin across buck tolerance (±2 %) and filter IR drop.

### 3.4 Passive components

| Ref | Value | Type | Purpose |
|---|---|---|---|
| C_IN | 10 µF + 1 µF | X7R 0805/0402, 16 V | Input bypass, placed close to IN pins |
| C_OUT | 10 µF + 100 nF | X7R, low ESL | Stability (≥ 10 µF ceramic) |
| C_NR | 10 µF | X7R or C0G-stack | Filters reference noise; sets soft-start |
| C_BYP | 10 nF | C0G | Further reference noise reduction (if using external FB) |
| R_PG | 100 kΩ | to VDDIO | Power-good pull-up to the MCU |
| EN | from MCU GPIO or RC delay | — | Sequencing |

Use DC-bias-derated X7R values. A 10 µF 0805 at 4.3 V retains about 6–7 µF, so check the manufacturer curves. Avoid Y5V entirely.

**Soft-start:** C_NR = 10 µF gives a startup of tens of ms. Confirm this is compatible with the IMU's VDD rise-time requirement (some IMUs specify a monotonic rise under ~100 ms) and use the PG output to gate IMU reset release.

---

## 4. Noise & Ripple Budget (at 2.2 MHz buck fundamental)

| Stage | Attenuation | Residual |
|---|---|---|
| Buck output ripple | — | ~10 mV pp |
| π-filter (2nd-order LC, fc ≈ 50 kHz) | ~50–60 dB at 2.2 MHz | ~20 µV pp |
| TPS7A4700 PSRR at 2.2 MHz (~40–45 dB, layout-limited) | ~40 dB | ~0.2 µV pp |
| **Result at IMU** | | **< 1 µV pp ripple**, broadband ≈ 4–5 µV RMS (LDO-dominated) |

In practice, layout coupling (magnetic pickup from the buck inductor, ground bounce) dominates over component limits. See §6.

---

## 5. Thermal

P_D = (4.3 − 3.3 V) × 0.2 A = **0.20 W worst case**

With θ_JA ≈ 35 °C/W on a 4-layer board with a via-stitched pad, the rise is about 7 °C. Even so, place the LDO **≥ 15 mm from the IMU** or on the opposite side of a thermal slot. Gyro bias is temperature-sensitive, and LDO self-heating gradients cause slow drift.

---

## 6. Layout & Grounding (the part that actually determines performance)

1. **Partitioning:** Keep the buck and its inductor at the board edge. The LDO sits between the buck and the IMU, and the IMU is in a quiet zone with no switching nodes beneath it on any layer.
2. **Ground:** Use a solid, unsplit ground plane. Route the buck's hot loop tightly on the top layer with its own local return. Avoid slot-split planes unless you are galvanically isolating.
3. **Kelvin sensing:** Run the OUT/SENSE connection as a dedicated trace from the IMU's VDD decoupling cap back to the LDO SENSE pin. This regulates at the load.
4. **NR cap:** Place C_NR immediately at the NR pin with its ground via adjacent to the LDO GND. This node is high impedance, so keep it short and guarded.
5. **Thermal pad:** Use ≥ 9 vias (0.3 mm) to inner ground planes.
6. **Inductor orientation:** Use a shielded inductor and orient its field axis away from the IMU. Keep a ≥ 20 mm distance where possible.
7. **Digital return:** Route SPI lines over continuous ground with series 33 Ω termination at the source. Keep the IMU VDDIO on a separate rail from IMU VDD where the IMU allows it.
8. **Mechanical:** Mount the IMU on a vibration-decoupled area. Keep heavy electrolytic caps (piezoelectric/microphonic) away from the IMU and prefer polymer or ceramic.

---

## 7. Protection & Fault Isolation
- **Upstream:** TVS plus ideal-diode reverse protection. An eFuse (TPS2596x) before the buck limits fault energy.
- **TPS7A4700 internal protection:** current limit and thermal shutdown.
- **Overvoltage clamp at the IMU** (optional): a TVS rated just above VDD max, e.g. a 3.6 V-class device, if the IMU abs-max is tight.
- **PG supervision:** The MCU holds the IMU in reset until PG is high, and logs PG drop events. These events correlate with data anomalies.
- **Brownout:** The buck UVLO is set above about 8 V, so the IMU rail drops cleanly instead of entering dropout and becoming noisy.

---

## 8. Verification Plan (Cycle 12 exit criteria)

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise spectrum | LNA (e.g., ×100 battery-powered) + FFT analyzer, 10 Hz–100 kHz | < 10 µV RMS integrated |
| Switching ripple | 50 Ω coax to spectrum analyzer, DC-blocked, at IMU cap | < 20 µV pp at fsw and harmonics |
| PSRR | Injection transformer (Picotest J2120A) + VNA (Bode 100) | > 80 dB system at 100 Hz–100 kHz |
| Load transient | 10 → 200 mA step, 1 µs edge | < 10 mV deviation |
| Startup/sequencing | Scope VIN, EN, VOUT, PG, IMU RST | Monotonic; PG precedes RST release |
| **System-level** | Allan deviation of a static IMU: LDO rail vs. bench linear supply/battery | Bias instability and ARW within 5 % of battery reference |
| Thermal | IR camera, worst-case VIN and ambient | LDO ΔT not coupling >0.5 °C into the IMU |

The Allan variance comparison against a battery-powered reference is the decisive test. It directly shows whether the power path degrades navigation performance.

---

## 9. Abbreviated BOM

| Function | Part |
|---|---|
| Reverse-polarity | LM74610-Q1 + N-FET |
| eFuse | TPS25963 / TPS2596x |
| Buck | LMR36015 (or TPS62912) |
| LDO | **TPS7A4700RGWR** |
| Ferrite | Murata BLM31KN601SN1 |
| Isolation (opt.) | SN6505B + Würth 750315371, ISO7741 |
| Caps | Murata GRM X7R / C0G series |

---

## 10. Open Items to Confirm (from prior cycles)
1. The actual IMU part, its VDD range, and its supply-sensitivity spec (some specify bias shift in °/hr per V).
2. Whether galvanic isolation is required, or only noise isolation.
3. The bus voltage range and transient standard (MIL-STD-1275, ISO 7637, DO-160). These may require a heavier Stage 0.
4. The board stack-up and IMU-to-switcher distance constraints.

Share the Cycle 11 outputs or IMU selection and I can tighten the noise budget, headroom, and thermal numbers to the actual design.