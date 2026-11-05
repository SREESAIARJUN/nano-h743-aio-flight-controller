# Low-Noise Power Supply Architecture for IMU Power Isolation (Cycle 28)

**Core regulator:** Texas Instruments TPS7A4700 (36 V, 1 A, ultra-low-noise positive LDO)

---

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail noise | < 10 µVrms (10 Hz – 100 kHz) | Keep supply-induced bias instability below sensor noise floor |
| Ripple at IMU pins | < 50 µVpp | Avoid aliasing into the gyro/accel bandwidth |
| PSRR at switcher fundamental | > 50 dB (regulator only), > 80 dB (with filter) | Suppress DC-DC switching residue |
| Ground isolation | Separate analog return, single-point star | Prevent motor/digital return currents in the IMU reference |
| Optional galvanic isolation | 1–2.5 kVrms | For platforms with noisy high-power buses (motors, actuators) |
| Load | 50–200 mA typical (e.g., ADIS16470/16500-class, ADXL355 + gyro) | Sized with ≥3× margin vs. 1 A LDO capability |

---

## 2. Top-Level Architecture

```
 System Bus (12–28 V)
        │
 [TVS + Reverse-Polarity FET + Input LC/EMI Filter]
        │
 ┌──────┴────────────────────────────────┐
 │ (Option B) Galvanic Isolation Stage   │
 │ SN6505B push-pull + 1:1.x transformer │
 │ → rectified isolated ~6 V             │
 └──────┬────────────────────────────────┘
        │
 Pre-Regulator: Sync Buck (spread-spectrum, e.g. TPS62913 / LMR33630)
   Vout = 4.5 V  (≈1.2 V headroom above 3.3 V)
        │
 Second-stage π-filter: Ferrite (e.g. BLM18PG) + 10 µF + 1 Ω/22 µF damping
        │
 ┌──────┴──────────────┬──────────────────────┐
 TPS7A4700 #1          TPS7A4700 #2 (opt.)    Low-noise LDO (e.g. TPS7A20)
 3.3 V IMU ANALOG      5.0 V ref/ADC rail     3.3 V/1.8 V IMU DIGITAL I/O
 (VDD sensor core)     (from ~6 V pre-reg)    (SPI, VDDIO)
        │                       │                     │
   Local decoupling        Local decoupling      Local decoupling
        │                       │                     │
      IMU AVDD               ADC/REF               IMU VDDIO ─► Digital isolator
                                                      (ISO7741) if Option B
```

**Option A (non-galvanic):** Power-domain isolation via dedicated pre-reg, filtering, LDO and star-ground — sufficient for most UAV/robotic platforms.
**Option B (galvanic):** Add SN6505B isolated stage + ISO774x digital isolators on SPI/DR/SYNC for platforms with heavy motor currents or ground potential differences.

---

## 3. TPS7A4700 Configuration

### 3.1 Key Device Characteristics (datasheet reference values)
- V_IN: 3 V – 36 V; V_OUT: 1.4 V – 20.5 V via ANY-OUT (higher with external divider)
- Output noise: ~4.17 µVrms (10 Hz – 100 kHz)
- PSRR: ~82 dB at 100 Hz, remains >50 dB into the MHz region
- Dropout: ~307 mV at 1 A
- Package: VQFN-20 (RGW, 5×5 mm) with thermal pad

### 3.2 ANY-OUT Programming (no external resistors → no added resistor noise)
V_OUT = 1.4 V + Σ(grounded pin weights)

| Rail | Pins tied to GND | Result |
|---|---|---|
| **3.3 V IMU analog** | 1.6 V + 0.2 V + 0.1 V | 1.4 + 1.9 = **3.3 V** |
| 5.0 V ADC/reference | 3.2 V + 0.4 V | 1.4 + 3.6 = **5.0 V** |
| 1.8 V core (if required) | 0.4 V | 1.4 + 0.4 = **1.8 V** |

Unused ANY-OUT pins are left floating. Using internal precision resistors avoids the thermal noise and tolerance stacking of an external divider.

### 3.3 Passive Components

| Pin / Function | Component | Notes |
|---|---|---|
| NR/SS | **10 µF X7R, 16 V**, low-leakage | Filters internal reference noise; sets soft-start (~tens of ms — verify against IMU power-up timing) |
| IN | 10 µF X7R + 100 nF | Place within 2 mm of pins |
| OUT | **10 µF X7R** (≥ datasheet minimum) + 100 nF | Ceramic stable; check DC-bias derating (use 1206 or 25 V rated 0805) |
| EN | RC from IN or MCU GPIO | Enables controlled sequencing |
| PG (open drain) | 100 kΩ pull-up to VDDIO | Gate IMU reset / MCU init on power-good |

### 3.4 Headroom Selection
- Dropout at 200 mA ≈ 60–70 mV, but **PSRR degrades sharply near dropout**.
- Choose **V_IN − V_OUT ≈ 1.0–1.2 V** (4.5 V in for 3.3 V out) to retain high-frequency PSRR while limiting dissipation.

### 3.5 Thermal Check (3.3 V rail)
P_D = (4.5 − 3.3) V × 0.2 A = **0.24 W**
With θ_JA ≈ 35 °C/W (JEDEC; board-dependent) → ΔT ≈ 8.4 °C. Ample margin even at 85 °C ambient.
*Note:* Keep the LDO thermally separated from the IMU (≥15 mm, or thermal relief/slot) — IMU bias is temperature-sensitive and LDO heat gradients appear as drift.

---

## 4. Pre-Regulator & Filtering

### 4.1 Buck Selection Criteria
- Switching frequency ≥ 2 MHz (pushes ripple well beyond IMU bandwidth and into the range where LC filtering is efficient)
- Spread-spectrum disabled **or** carefully evaluated: spread-spectrum reduces EMI peaks but spreads energy toward lower frequencies — characterize against IMU sample rate
- Forced-PWM mode (avoid PFM/burst mode; its low-frequency bursts fall inside the gyro band)
- Sync pin available to lock frequency away from IMU internal sample clock harmonics

### 4.2 π-Filter Between Buck and LDO
- Ferrite bead: high impedance at f_sw, rated ≥2× load current, check DC-bias impedance drop
- Damping: 22 µF with ~1 Ω ESR (or series resistor) to suppress LC resonance peaking
- Corner frequency target: < f_sw / 10

**Combined attenuation estimate at 2 MHz:** π-filter ~40 dB + LDO ~45–50 dB → **~85–90 dB** total ripple rejection.

---

## 5. Noise Budget (3.3 V IMU Analog Rail)

| Source | Contribution (10 Hz–100 kHz) |
|---|---|
| TPS7A4700 intrinsic (C_NR = 10 µF) | ~4.2 µVrms |
| Residual buck ripple after filter + PSRR (e.g., 20 mVpp in → ~1 µVpp) | ~0.4 µVrms equivalent |
| Ground bounce / layout coupling (target with star ground) | < 2 µVrms |
| **RSS total** | **≈ 4.7 µVrms** — meets < 10 µVrms target |

Cross-check against the IMU's supply sensitivity specification (bias shift per volt of supply) to confirm the resulting bias contribution is ≪ the sensor's in-run bias instability.

---

## 6. Grounding & Isolation Strategy

1. **Split ground regions:** AGND_IMU, DGND, PGND (buck). Connect at a single star point near the LDO GND pin/thermal pad.
2. **Buck return currents** confined to PGND island; never route under IMU or LDO.
3. **Option B galvanic:**
   - SN6505B + transformer (e.g., Würth 750315371-class) → isolated ~6 V → buck/LDO chain on isolated side
   - Signals: ISO7741 (SPI: SCLK, MOSI, CS out; MISO in) + separate channel for DRDY/SYNC
   - Keep isolation barrier creepage per required rating; stitching cap (~100 pF Y-rated) across barrier for EMI if needed
4. **Mechanical:** IMU on dedicated board region or daughterboard with vibration damping; power entry via low-inductance connector with paired ground pins.

---

## 7. PCB Layout Rules

- LDO input/output caps on the same layer as the IC, minimal loop area.
- NR/SS capacitor **directly** at NR pin, returned to the LDO's quiet GND — this node is high-impedance and noise-critical.
- Use SENSE (if remote-sense routing is used) taken directly at IMU supply pins as a Kelvin connection.
- Thermal pad: 9+ vias to an internal ground plane.
- Shield critical analog traces with ground pour; no digital traces crossing under the analog rail.
- Keep the buck inductor ≥20 mm from the IMU and oriented to minimize magnetic coupling (shielded inductor mandatory).

---

## 8. Sequencing & Protection

| Step | Event |
|---|---|
| 1 | Bus applied → TVS/reverse-protection → buck starts |
| 2 | Buck PG → enables TPS7A4700 #1 (analog) |
| 3 | Soft-start via C_NR ramps analog rail (monotonic, no overshoot) |
| 4 | Analog PG → enables VDDIO LDO (or simultaneous per IMU datasheet requirement) |
| 5 | Final PG → releases IMU RESET → MCU waits IMU start-up time before SPI access |

- TPS7A4700 internal current limit and thermal shutdown cover fault cases.
- Add reverse-current protection if the IMU rail can be back-driven via I/O during power-down (series Schottky not recommended on the analog rail — use the isolator/IO sequencing instead).

---

## 9. Verification Plan (Cycle 28 Exit Criteria)

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise spectral density | LNA (e.g., 40–60 dB gain) + FFT/spectrum analyzer, battery-powered setup | < 10 µVrms integrated; no spurs > 1 µV in IMU band |
| PSRR / ripple | Injection transformer (e.g., Picotest J2120A) + VNA | Matches combined attenuation estimate within 6 dB |
| Load transient | 10 → 200 mA step | < 10 mV deviation, no ringing > 2 cycles |
| IMU static Allan variance | 2–4 h static log, compare bench linear supply vs. this design | Bias instability and ARW within 5% of linear-bench reference |
| Motor-on stress | Run actuators at full load | No measurable change in IMU noise floor |
| Thermal | IR camera at max ambient | LDO-to-IMU gradient within allowed limit |

---

## 10. Cycle 28 Summary & Open Items

**Decisions locked:**
- TPS7A4700 with ANY-OUT 3.3 V, C_NR = 10 µF, ~1.2 V headroom from a forced-PWM ≥2 MHz buck
- π-filter with damping between buck and LDO
- Separate LDO for IMU digital I/O; star-ground topology

**Open for Cycle 29:**
1. Confirm galvanic isolation requirement (Option A vs. B) based on platform ground-shift measurements.
2. Evaluate spread-spectrum on/off against measured IMU spur results.
3. Validate soft-start timing against IMU-specific power-up specification.
4. Final BOM capacitor derating review (DC-bias effect on 10 µF output/NR caps).

If you share the specific IMU part number and input bus voltage, I can refine the rail voltages, headroom, noise budget, and sequencing to match that sensor's datasheet exactly.