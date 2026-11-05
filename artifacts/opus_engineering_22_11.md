# Low-Noise IMU Power Supply Architecture (Cycle 22)
**Core regulator:** Texas Instruments TPS7A4700 (36 V, 1 A, ultra-low-noise LDO)

> **Assumptions:** I don't have your prior-cycle documentation, so this assumes:
> - A 12 V (9–18 V) platform bus.
> - A MEMS IMU with a 3.3 V analog/core rail and a separate VDDIO, such as the ADIS16470, ICM-42688-P or BMI088 class.
> - "Isolation" means both noise isolation (the primary goal) and optional galvanic isolation.
>
> Adjust the values marked ⚙ to match your actual IMU and bus.

---

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU supply rail | 3.3 V ±2 % ⚙ | Typical IMU VDD range is 3.0–3.6 V |
| Load current | ≤ 100 mA (IMU ~10–50 mA) | Leaves margin for the heater or ADC if present |
| Output noise (10 Hz–100 kHz) | < 10 µVrms | Keeps supply-induced bias/ARW contribution below the sensor noise floor |
| Ripple at buck fsw | < 1 µVrms | Prevents aliasing into the gyro/accel bandwidth |
| Low-frequency (0.1–10 Hz) drift | Minimized | Affects bias instability |
| Ground isolation | IMU analog ground star-connected; optional galvanic barrier | Prevents motor/servo return currents from flowing in the sensor reference |

---

## 2. Architecture Overview

```
 12 V BUS ──► [Input protection] ──► [Stage 1: Pre-regulator] ──► [Stage 2: Passive filter] ──► [Stage 3: TPS7A4700] ──► [Stage 4: Local POL filter] ──► IMU VDD
              TVS + reverse-pol.      Sync buck, 4.5 V            Ferrite + CLC π-filter       3.3 V ultra-low noise      Ferrite + MLCC at pins
              + PTC/eFuse             (or isolated SN6505B                                       │
                                       path, Option B)                                         └──► TPS7A02 / LP5907 1.8 V ──► IMU VDDIO
                                                                                                     (separate digital rail)
```

### Option B: Galvanic Isolation Path
Use this when the IMU must be isolated from noisy actuator or bus grounds.

```
 5 V ──► SN6505B push-pull driver ──► Transformer (e.g., Würth 750315371, 1:1.3) ──► Schottky full-wave rectifier ──► ~5.5 V ISO
      ──► π-filter ──► TPS7A4700 (3.3 V ISO) ──► IMU
 Data path: ISO7741 / ISO7762 digital isolator (SPI) across the barrier
```

The SN6505B runs at about 420 kHz with a spread-spectrum option. This is well above IMU bandwidths and easily attenuated by the π-filter plus LDO PSRR.

---

## 3. Stage-by-Stage Design

### Stage 1: Pre-Regulator (Non-Isolated Baseline)
- **Device:** TPS62130A or LMR33630 (synchronous buck, spread-spectrum capable).
- **Output:** 4.5 V. This gives about 1.2 V of headroom over 3.3 V.
  - The TPS7A4700 dropout is about 307 mV at 1 A, and much lower at 100 mA.
  - PSRR degrades sharply near dropout, so keep **≥ 1 V headroom**.
- **Switching frequency:** ≥ 2 MHz.
  - This places the fundamental far above the IMU sample/ODR bands.
  - It also suits the LC filter design.
  - Enable spread spectrum (or dithering) to lower peak spectral lines.

### Stage 2: Inter-Stage π-Filter

```
 4.5 V ─── C1 (10 µF X7R) ─┬─ L1 (2.2 µH shielded, or ferrite BLM18PG/BLM21) ─┬─ C2 (10 µF X7R + 100 nF) ─── TPS7A4700 IN
                           GND                                                  GND
```

- **Corner frequency:** f_c = 1/(2π√(LC)) ≈ 34 kHz. This gives about 70 dB of attenuation at 2 MHz in the ideal case; plan for about 40 dB in practice due to parasitics.
- **Damping:** Add an electrolytic or polymer cap (22 µF, ESR ~0.5 Ω) or a series 1 Ω + 10 µF RC damper. This prevents an LC resonance peak that the LDO would see as an input oscillation.

### Stage 3: TPS7A4700 Configuration

| Pin / Element | Setting | Notes |
|---|---|---|
| IN | 4.5 V filtered | Plus 10 µF X7R placed close to the pin |
| OUT | 3.3 V | **ANY-OUT:** V_OUT = 1.4 V + Σ(grounded pins). 3.3 − 1.4 = 1.9 V = **1P6V + 0P2V + 0P1V** tied to GND; all other ANY-OUT pins left floating |
| C_OUT | 10 µF X7R (≥ 0805, 10 V+) | Must satisfy the datasheet minimum and stability requirements; account for DC-bias derating |
| NR/SS | C_NR = 1 µF to 10 µF ⚙ | Larger C_NR gives lower noise (datasheet ~4.17 µVrms) but slower start-up. Verify the IMU power-on ramp spec |
| EN | RC-delayed from pre-reg PG, or MCU GPIO | Enforces sequencing (VDD before VDDIO, or per the IMU datasheet) |
| Thermal pad | Solid via array to GND plane | At 100 mA, P = 1.2 V × 0.1 A = 0.12 W, so ΔT is only a few °C |

Using ANY-OUT instead of external feedback resistors eliminates resistor thermal noise and tempco drift. This directly benefits the IMU's bias stability.

**Expected PSRR:** about 80 dB at 100 Hz and about 50–55 dB in the MHz range. Confirm these values against the datasheet curves at your headroom and load.

### Stage 4: Point-of-Load Filtering at the IMU
- Ferrite bead between the LDO output and the IMU VDD. Use a low-DCR type such as BLM18KG, with < 0.1 Ω DCR to avoid IR drop.
- Decoupling at the IMU pins, per the IMU datasheet: 100 nF + 1 µF (+ 10 µF bulk).
- **VDDIO:** Use a separate small LDO, such as a TPS7A02 or LP5907 at 1.8 V. This keeps SPI/I²C edge currents off the analog rail.

---

## 4. Noise Budget Estimate

| Contributor | Calculation | Result at IMU VDD |
|---|---|---|
| TPS7A4700 intrinsic noise | Datasheet value | ~4.2 µVrms |
| Buck ripple (10 mVpp at 2 MHz) | −40 dB (π-filter) − 50 dB (PSRR) | ~0.3 µV |
| Bus low-frequency ripple (100 mVpp at 1 kHz) | − ~75 dB (buck loop + LDO PSRR) | ~0.02 µV |
| POL ferrite/MLCC (HF residue) | Additional −20 dB above ~10 MHz | Negligible |
| **Total (RSS)** | | **≈ 4.3 µVrms ✔ (target < 10 µVrms)** |

Translate this to sensor error using the IMU's power-supply sensitivity (deg/s/V or g/V) to confirm it is below the angle random walk (ARW) and velocity random walk (VRW) floor.

---

## 5. Grounding and Layout Rules
1. **Split return paths.** Use AGND for the IMU and LDO and PGND for the buck. Join them at a single star point near the LDO ground. Do not route digital or motor returns under the IMU.
2. **Kelvin connection.** If you use the adjustable mode instead of ANY-OUT, take the sense or feedback tap directly at the IMU decoupling cap.
3. **Shielding.** Keep the buck inductor and switch node at least 15 mm from the IMU. Consider a guard ring or a shield can over the buck.
4. **Thermal stability.** Keep the LDO thermally decoupled from the IMU. Thermal gradients induce IMU bias drift.
5. **Mechanical.** Avoid large MLCCs directly under or beside the IMU. Piezoelectric "singing" and board flex couple into accelerometers. Use C0G for small values, and soft-termination X7R where possible.
6. **Galvanic option.** Maintain the creepage/clearance required across the SN6505B transformer and ISO77xx barrier. Add a small Y-cap (~100 pF) across the barrier only if needed for EMI.

---

## 6. Representative BOM

| Ref | Part | Function |
|---|---|---|
| U1 | TPS62130A / LMR33630 | 12 V → 4.5 V buck |
| L1, FB1 | 2.2 µH shielded / BLM21PG | π-filter |
| U2 | **TPS7A4700RGW** | 3.3 V ultra-low-noise IMU rail |
| U3 | TPS7A02 / LP5907 | 1.8 V VDDIO |
| U4 (opt.) | SN6505B + Würth 750315371 | Isolated supply |
| U5 (opt.) | ISO7741 | SPI isolation |
| D1 | SMBJ18A TVS + ideal diode (LM74610) | Input protection |

---

## 7. Cycle 22 Verification Plan
1. **Noise:** Measure output noise with an LNA (e.g., ×100 gain) plus a spectrum analyzer from 10 Hz to 100 kHz. Pass if < 10 µVrms.
2. **PSRR:** Inject ripple with a line injector (e.g., Picotest J2120A) and measure PSRR from 10 Hz to 10 MHz.
3. **Transient:** Apply 0–50 mA load steps. Overshoot should be < 1 % and settle within the IMU spec.
4. **IMU-level test:** Take an Allan variance capture with the IMU on the new supply versus a lab battery reference. Bias instability and ARW should match within measurement uncertainty.
5. **Thermal:** Sweep from −40 to +85 °C to confirm output drift and the IMU bias-vs-temperature correlation.
6. **EMI coupling:** Run actuators and motors while logging IMU output, and check for spectral spurs at the fsw or SN6505B frequency.

---

## 8. Open Items to Confirm
- Exact IMU part number and its VDD/VDDIO range and sequencing requirements.
- Whether galvanic isolation (Option B) is required or noise isolation alone is sufficient.
- Final C_NR value as a trade-off between start-up time and noise.
- Actual bus voltage range and transient profile (e.g., ISO 7637 or MIL-STD-1275 if vehicular).

If you share the specific IMU and the Cycle 21 findings, I can tighten the component values, ANY-OUT strapping, and noise-to-sensor-error conversion.