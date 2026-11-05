# Low-Noise IMU Power Supply Architecture: Cycle 1
**Core regulator:** Texas Instruments TPS7A4700 (36 V, 1 A, ultra-low-noise positive LDO)
**Cycle 1 scope:** Architecture definition, rail budgeting, first-pass component selection, isolation strategy, and the verification plan.

**Datasheet caveat:** Specifications and pin connections below are typical values and design intent. Confirm every value against the current TPS7A47 datasheet (SBVS204) before schematic capture.

---

## 1. Design Objectives and Assumptions

| Parameter | Cycle 1 Assumption | Rationale |
|---|---|---|
| Input bus | 9–32 V DC (12/24/28 V vehicle or platform bus) | Typical UAV, robotic, or avionics source |
| IMU class | MEMS tactical-grade (e.g., ADIS1647x/ADIS1650x class), 3.3 V supply | Bias stability is sensitive to supply ripple and noise |
| IMU load | ≤ 100 mA typical, 150 mA design budget | Includes margin for heaters and sync logic |
| Optional rail | 5.0 V for IMU heater or auxiliary analog circuitry | TBD in Cycle 2 |
| Target output noise | < 10 µV RMS (10 Hz–100 kHz) at the IMU pins | Below IMU internal ADC and reference noise floor |
| Ripple at IMU | < 100 µV p-p at switching frequency | Avoids aliasing into gyro and accelerometer bandwidth |
| Isolation goal | Noise isolation (mandatory); galvanic isolation (optional, Section 6) | |

---

## 2. Key TPS7A4700 Characteristics Used in This Design

| Spec | Typical Value | Design Relevance |
|---|---|---|
| VIN range | 3 V – 36 V | Can tolerate direct bus connection if needed |
| VOUT range | 1.4 V – 20.5 V | Settable with resistors or ANY-OUT pins |
| Output noise | ~4.17 µV RMS (10 Hz–100 kHz) | Core reason for selection |
| PSRR | ~82 dB @ 100 Hz; > 55 dB to ~10 kHz | Rejects pre-regulator and bus ripple |
| Dropout | ~307 mV @ 1 A | Allows low headroom, but PSRR needs more headroom (Section 3.3) |
| Output current | 1 A | Large margin at a 150 mA load |
| Package | 20-pin 5×5 mm VQFN (RGW) with thermal pad | Thermal pad must be soldered to ground |
| Features | EN, PG (power-good), NR/SS pin | Supports sequencing and fault monitoring |

---

## 3. Architecture

### 3.1 Block Diagram

```
 BUS 9–32V
    │
 ┌──▼─────────────┐   ┌───────────────┐   ┌──────────────┐   ┌──────────────────┐   ┌──────────┐
 │ Input Protect  │──▶│ Sync Buck     │──▶│ 2-stage LC / │──▶│ TPS7A4700 (U1)   │──▶│ Local    │──▶ IMU VDD
 │ TVS, reverse-V │   │ Pre-regulator │   │ Ferrite π    │   │ 3.3V ANY-OUT     │   │ π filter │    (3.3V)
 │ eFuse, EMI LC  │   │ ~4.6V, 2.1MHz │   │ post-filter  │   │ low-noise LDO    │   │ at IMU   │
 └────────────────┘   └───────┬───────┘   └──────────────┘   └────────┬─────────┘   └──────────┘
                              │                                       │ PG → MCU / IMU RST
                              │ (optional 2nd branch)                 │
                              └──▶ LC filter ──▶ TPS7A4700 (U2) 5.0V ─▶ Heater / aux analog
 Digital logic (MCU, comms) ◀── separate buck/LDO branch — never shares U1 output
```

### 3.2 Topology Rationale

**Two-stage regulation (switcher followed by LDO).** A buck converter handles the large step-down efficiently. The LDO then removes switching ripple and provides an ultra-low-noise output. Without the buck, a direct 28 V → 3.3 V LDO conversion at 150 mA would dissipate (28 − 3.3) × 0.15 ≈ **3.7 W**, which is thermally unacceptable in a 5×5 mm package.

**Switching frequency of about 2.1 MHz.** At 2.1 MHz the ripple sits far above the IMU sensing bandwidth (typically < 1 kHz). It is also easy to filter passively and stays outside the AM band.

**Spread-spectrum disabled.** Spread-spectrum is commonly enabled for EMI compliance, but it smears energy toward lower frequencies, where the LDO's PSRR is a different and harder problem. It is disabled for Cycle 1 and will be re-evaluated in EMI testing.

**Dedicated sensor rail.** The IMU rail is a dedicated "quiet" rail. No digital loads share the LDO output.

### 3.3 Headroom Selection

PSRR collapses near dropout. Target VIN − VOUT ≈ **1.0–1.3 V**.

- Pre-regulator setpoint: **4.6 V**, giving 1.3 V headroom for the 3.3 V rail.
- Dissipation in U1: (4.6 − 3.3) × 0.15 A ≈ **0.20 W**.
- Temperature rise: with θJA ≈ 35 °C/W on a good 4-layer board, ΔT ≈ 7 °C. This is negligible.
- If the 5 V rail is needed, use a second buck output (or a single buck at ~6.3 V feeding both LDOs). Re-check U1 dissipation if it shares that input.

---

## 4. TPS7A4700 Configuration

### 4.1 Output Voltage via ANY-OUT (no external resistors, which reduces noise and drift)

VOUT = 1.4 V (reference) + the sum of the binary-weighted pins tied to GND.

| Rail | Pins tied to GND | Calculation |
|---|---|---|
| **3.3 V (IMU)** | 1.6 V, 0.2 V, 0.1 V | 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V** |
| **5.0 V (aux)** | 3.2 V, 0.4 V | 1.4 + 3.2 + 0.4 = **5.0 V** |

- Leave unused ANY-OUT pins floating, per the datasheet.
- Verify the required FB/SENSE pin connection for ANY-OUT mode in the datasheet application section.

### 4.2 First-Pass Bill of Materials (per LDO)

| Ref | Value | Type | Notes |
|---|---|---|---|
| C_IN | 10 µF + 100 nF | X7R MLCC, 50 V | Close to the IN pin |
| C_OUT | 10 µF + 100 nF | X7R MLCC | Datasheet minimum is about 10 µF; check stability |
| C_NR/SS | 1 µF | C0G/X7R, low leakage | Lowers the noise corner and gives a soft-start of ~tens of ms |
| C_BYP | 10 nF | C0G | Feed-forward capacitor, if used in the chosen configuration |
| R_PG | 100 kΩ pull-up | — | PG to the MCU GPIO and IMU reset gating |
| EN | Divider from VIN, or MCU-controlled | — | Sets UVLO so the LDO does not start before the buck is in regulation |

### 4.3 Sequencing

Bus valid → eFuse on → buck soft-start → buck PG asserted → LDO EN → NR/SS ramp → LDO PG asserted → IMU reset released by the MCU.

The IMU reset should be released at least 10 ms after LDO PG, to satisfy typical IMU power-on requirements.

---

## 5. Noise Isolation Strategy

### 5.1 Filtering Chain

**Buck output to LDO input:**
- Ferrite bead (≥ 600 Ω at 100 MHz, DCR < 0.1 Ω), followed by 10 µF.
- Then a second-stage LC: 2.2 µH shielded inductor with 22 µF.
- Damping is needed to avoid an LC resonance peak. Use a series-R electrolytic or polymer capacitor (~0.5 Ω ESR) or an RC snubber.
- Target attenuation: about −40 dB at 2.1 MHz before the LDO. Combined with the LDO's high-frequency PSRR, this gives more than 70 dB total.

**LDO output to IMU:**
- Local π filter at the IMU: 1 µF – ferrite – 1 µF + 100 nF.
- This decouples IMU load transients from the shared trace and blocks RF pickup.

### 5.2 Mechanical and Microphonic Consideration (IMU-specific)

Class-II MLCCs (X7R/X5R) are piezoelectric. Under vibration they can generate voltage noise correlated with motion, which corrupts IMU data in exactly the conditions of interest.

- Use **C0G** for C_NR/SS, C_BYP, and the small IMU decoupling capacitors.
- Use **tantalum-polymer** for bulk capacitance near the IMU where the value requires it.
- Keep the IMU and its supply on a rigid board region, or follow the IMU vendor's mounting guidance.

### 5.3 Grounding and Layout

- Use a 4-layer minimum stack-up: Signal / solid GND / Power / Signal.
- Use a single continuous ground plane with **zoned placement** rather than split planes: switcher zone, quiet analog zone, digital zone.
- Keep the switcher's hot loop (input capacitor, FET, and switch node) minimal and placed far from the IMU, ideally on the opposite board edge.
- Route the LDO output to the IMU as a short, wide trace with a ground return directly beneath it. Do not route under the switcher or inductor.
- Give the TPS7A4700 thermal pad ≥ 9 thermal vias to the inner ground plane.
- Place the NR/SS capacitor directly at its pin. The pin is high-impedance, so leakage or coupling there appears directly as output noise.
- Route the IMU SPI and data-ready lines away from the switch node. Add series resistors (22–33 Ω) at the source to limit edge rates.

---

## 6. Optional Galvanic Isolation (Decision Point for Cycle 2)

Galvanic isolation is required if the IMU must be isolated from the platform bus, for example because of ground loops, long harnesses, or safety requirements.

```
BUS → SN6505B push-pull driver + isolation transformer (e.g., 1:1.5) → rectifier
    → LC filter → TPS7A4700 3.3V → IMU
IMU data → digital isolator (ISO7741 / ISO7762) → MCU
```

- The push-pull frequency (~420 kHz) is lower than the buck's, so it needs a stronger post-filter. The LDO's PSRR remains good (~60 dB) at this frequency.
- Trade-offs: about 70–80% efficiency, more parts and board area, and added isolator latency on SPI (typically ~10–20 ns, which is acceptable).

---

## 7. Cycle 1 Error and Noise Budget (Estimate)

| Contributor | Estimated value at IMU |
|---|---|
| LDO intrinsic noise (with C_NR = 1 µF) | ~4–5 µV RMS |
| Residual switching ripple (2.1 MHz, after LC and PSRR) | < 30 µV p-p, outside IMU bandwidth |
| Low-frequency bus ripple (100 Hz–1 kHz, after ~80 dB PSRR) | < 5 µV for a 50 mV bus ripple |
| Load-transient deviation (IMU burst) | < 5 mV, recovers in < 50 µs |
| **Total in-band (10 Hz–100 kHz)** | **≈ 6–8 µV RMS**, meeting the < 10 µV target |

---

## 8. Verification Plan

1. **Output noise:** Measure with a low-noise preamplifier (≥ 40 dB gain) and a spectrum analyzer, 10 Hz–100 kHz. Compare against the datasheet curve.
2. **PSRR:** Inject a signal at the LDO input with a line-injector or Bode analyzer, sweeping 10 Hz–10 MHz.
3. **Ripple at the IMU pins:** Use a 50 Ω coax probe with the scope at 20 MHz bandwidth-limited and full bandwidth.
4. **Load transient:** Apply 10 → 150 mA steps and check stability, overshoot, and settling.
5. **System-level test:** Record IMU Allan variance with (a) the lab linear supply and (b) the designed supply. The bias-instability difference should be within measurement noise.
6. **Vibration (microphonics):** Shake-table sweep while logging supply noise and IMU output.
7. **Thermal:** −40 to +85 °C (or the platform range). Log rail drift and IMU bias.
8. **EMI pre-scan:** Conducted and radiated, to decide the spread-spectrum question.

---

## 9. Open Items for Cycle 2

| # | Item | Owner/Action |
|---|---|---|
| 1 | Confirm the exact IMU part, its supply range, and peak current | Systems