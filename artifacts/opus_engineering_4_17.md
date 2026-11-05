# Low-Noise IMU Power Architecture with the TPS7A4700 (Cycle 4)

**Scope and assumptions:** I don't have the outputs from Cycles 1–3, so this cycle is self-contained. It assumes:
- A 3.3 V MEMS IMU in the ADIS16470, ADXL355 or ICM-42688 class, drawing 10–60 mA.
- A noisy system rail from 5 V to 28 V that also feeds digital, RF or motor loads.

Substitute your actual IMU and input rail where noted.

---

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail (VDD) | 3.3 V ±1%, < 10 µVrms (10 Hz–100 kHz) | Keeps supply-coupled noise well below the sensor noise floor |
| Ripple at IMU pin | < 20 µVpp at the switching frequency | Avoids aliasing into the IMU's internal ADC and sample-rate bands |
| Low-frequency drift and noise (0.1–10 Hz) | Minimized | Bias instability and random walk are sensitive to supply wander |
| Digital IO rail (VDDIO) | Separate from VDD | Prevents SPI and clock edge currents from modulating the analog rail |
| Fault isolation | IMU rail survives upstream transients | Reverse polarity, load dump, ESD |

---

## 2. Architecture Overview

```
 V_SYS (5–28 V, noisy)
   │
   ├─► [TVS + reverse-polarity FET + input LC filter]
   │
   ├─► Stage 1: Synchronous buck pre-regulator (≈4.5 V, f_sw ≥ 2 MHz, spread-spectrum OFF)
   │        e.g., TPS62913 (low-noise buck) or LMR33630 / TPS62130
   │
   ├─► Stage 2: π-filter (C – FB/L – C), cutoff ≈ 50–100 kHz
   │
   ├─► Stage 3: TPS7A4700  →  3.3 V_IMU_A  (ultra-low-noise analog rail)
   │        │
   │        └─► Stage 4: local RC / ferrite post-filter → IMU VDD pins
   │
   └─► Separate small LDO (e.g., TPS7A20 / TLV755P) → 3.3 V or 1.8 V VDDIO → IMU VDDIO + MCU SPI bank
```

**Key principle:** each stage attenuates a different part of the spectrum.
- The pre-regulator handles efficiency and large-headroom conversion.
- The π-filter handles MHz switching ripple, where LDO PSRR collapses.
- The TPS7A4700 handles noise and ripple from DC to ~100 kHz.
- The local post-filter decouples IMU-generated current transients from the shared trace.

---

## 3. Stage-by-Stage Design

### Stage 1: Pre-Regulator

- **Output:** ~4.5 V. This gives ~1.2 V of headroom over 3.3 V, which:
  - keeps the TPS7A4700 well out of dropout (~307 mV max at 1 A, far lower at IMU currents);
  - preserves PSRR, which degrades as V_IN − V_OUT approaches dropout.
- **Switching frequency:** ≥ 2 MHz, so ripple sits well above IMU bandwidths and is easy to filter with small LC parts.
- **Mode:** forced PWM (FPWM).
  - Disable PFM/power-save mode. Its variable low-frequency ripple can land inside the LDO's high-PSRR band edge and inside the IMU's measurement band.
  - Disable spread-spectrum on this rail unless EMC demands it. Spreading moves energy down toward the region where the filter is weaker.
- **If V_SYS is already a clean 5 V:** skip the buck and feed the π-filter directly. The LDO dissipates (5 − 3.3) × 0.06 ≈ 0.1 W, which is trivial.

### Stage 2: Inter-Stage π-Filter

| Component | Value | Notes |
|---|---|---|
| C1 (buck side) | 10 µF X7R 0805 + 100 nF 0402 | |
| L / FB | 1–2.2 µH shielded inductor or ferrite bead | Example bead: BLM18KG601, ≥600 Ω at 100 MHz |
| C2 (LDO side) | 10 µF X7R + 100 nF | Doubles as the TPS7A4700 C_IN |
| Damping | 1 Ω series R with a 22 µF electrolytic/polymer, or the ESR of a polymer cap | Prevents an LC resonance peak |

- **Cutoff:** f_c = 1 / (2π√(2.2 µH × 10 µF)) ≈ 34 kHz.
- **Attenuation at 2 MHz:** roughly 70 dB theoretical, 40–50 dB realistic once parasitics are included.
- **Ferrite bead caution:** a bead's inductance collapses with DC bias. Check the bead's DC-bias curve at the actual load current.

### Stage 3: TPS7A4700 Core Regulator

**Key specs used** (verify against the current datasheet):
- V_IN: 3–36 V
- V_OUT: 1.4–20.5 V
- I_OUT: up to 1 A
- Noise: 4.17 µVrms (10 Hz–100 kHz)
- PSRR: ~82 dB at 100 Hz, still >50 dB into the tens of kHz

**Setting the output with the ANY-OUT pins.** No external resistors are needed, so there is no resistor noise or tempco error:

```
V_OUT = 1.4 V (reference) + sum of the ANY-OUT pins tied to GND
3.3 V = 1.4 + 1.6 (1P6) + 0.2 (0P2) + 0.1 (0P1)
→ Tie 1P6, 0P2, 0P1 to GND; leave 6P4, 3P2, 0P8, 0P4 floating.
```

- Tie **SENSE** to **OUT** at the load-side point of the output capacitor (Kelvin connection).
- Using ANY-OUT keeps the output noise at the reference-limited minimum and removes divider-resistor drift.

| Pin / Component | Value | Purpose |
|---|---|---|
| C_IN | 10 µF X7R (shared with the π-filter C2) | Input stability and transient response |
| C_OUT | 10 µF X7R/X5R ceramic plus optional 10 µF | Datasheet requires ≥10 µF ceramic. Add more for lower output impedance. |
| C_NR (NR pin) | 10 nF C0G/NP0 | Filters reference noise. Larger values lower low-frequency noise but slow start-up. |
| EN | Tie to IN via 100 kΩ, or drive from a sequencer | Lets the MCU power-cycle the IMU for recovery |
| Thermal pad | Solid via array to the GND plane | |

**Cap dielectric note:** use C0G for C_NR. X7R piezoelectric microphonics on the NR node become noise when the platform vibrates, which matters for an IMU.

**Thermal check:** P_D = (4.5 − 3.3) × 0.06 A = 72 mW. ΔT ≈ 72 mW × ~35 °C/W ≈ 2.5 °C. This is negligible, which is good because LDO self-heating near the IMU would add thermal gradients and bias drift.

### Stage 4: Local IMU Post-Filter

```
TPS7A4700 OUT ──[FB or 1–2.2 Ω]──┬── IMU VDD
                                  ├─ 1 µF X7R (at pin)
                                  └─ 100 nF X7R (at pin)
```

- **Purpose:** isolate the IMU's internal ADC and charge-pump current pulses from the shared rail, and block any coupling picked up along the trace.
- **Sizing the series resistance:** ensure the IR drop stays within the IMU's VDD tolerance. For example, 2.2 Ω × 40 mA = 88 mV, which is fine for a 3.0–3.6 V part.
- **Per-pin decoupling:** follow the IMU datasheet exactly. Some parts (e.g., the ADXL355) require specific internal-regulator caps on V1P8ANA and V1P8DIG. Do not substitute values.

### VDDIO Rail (Isolation Partner)

- Use a separate small LDO (TPS7A20, TLV755P) from the pre-regulator output.
- This keeps SPI edge currents (high di/dt) off the analog LDO.
- If the IMU has a single VDD pin with no separate VDDIO, add series resistors (22–47 Ω) on the SPI lines. These soften edges and limit return currents.

---

## 4. Noise Budget Verification

| Source | Contribution at IMU VDD | Method |
|---|---|---|
| TPS7A4700 intrinsic noise | ~4.2 µVrms (≈13 nV/√Hz average) | Datasheet, C_NR = 10 nF |
| Buck ripple (10 mVpp at 2 MHz) | ~10 mV × −45 dB (π) × −40 dB (LDO at MHz) × −10 dB (local) → < 0.1 µVpp | Cascaded attenuation |
| Low-frequency system-rail noise (100 Hz–1 kHz, 50 mVpp) | 50 mV × −80 dB → ~5 µVpp | LDO PSRR at low frequency |
| Total | ≈ 5–7 µVrms equivalent | |

**Translating to sensor error.** IMU power-supply sensitivity is usually in the mg/V or °/s/V range. As an example, assume 1 mg/V. Then 10 µV × 1 mg/V = 0.01 µg. This is negligible compared with an ADXL355 noise density of ~25 µg/√Hz.

**Conclusion:** supply noise is no longer the limiting error term. The design margin shifts to layout, thermal behavior and mechanical coupling.

---

## 5. Layout and Isolation Rules

1. **Physical partition.** Place the buck in one corner of the board, away from the IMU. Keep the switch-node loop minimal and the inductor shielded. Keep at least 15–20 mm between the buck and the IMU.
2. **Ground strategy.** Use one continuous ground plane; do not split it.
   - Route the buck's high-current return so it does not pass under the LDO or the IMU.
   - Make a single tie point between the "power" and "quiet analog" ground regions at the LDO's GND/thermal pad.
3. **Kelvin sense.** Route SENSE as a dedicated trace to the C_OUT pad nearest the IMU branch point.
4. **Keep the NR node short.** C_NR goes directly at the NR pin, with no vias in series. The NR node is high-impedance and picks up coupled noise easily.
5. **Thermal isolation.** Keep the LDO and buck out of the IMU's thermal path. Use copper keep-outs or slots around the IMU to reduce gradient-induced bias drift.
6. **Mechanical.** Avoid large ceramic caps directly under or adjacent to the IMU. They can be piezoelectric emitters and receivers.
7. **Shielding (optional).** Add a guard ring of stitched ground vias around the IMU and its post-filter. A can shield is worthwhile if RF sources are present.

---

## 6. Protection and Sequencing

- **Input:** use a TVS sized for the system transients (e.g., SMBJ33A for a 28 V bus), plus P-FET reverse-polarity protection or an ideal-diode controller (LM74700).
- **Inrush and start-up:** C_NR sets the soft-start, which takes milliseconds at 10 nF. Confirm the IMU's power-on ramp requirement. Some IMUs require a monotonic ramp within a time limit.
- **Sequencing:** bring VDD up before or with VDDIO, per the IMU datasheet. Use the MCU or a power-good signal to drive the TPS7A4700 EN pin, which also allows a hard IMU reset.
- **Back-powering:** while the IMU is unpowered, hold the MCU's SPI pins low or high-Z. Otherwise current flows through the IMU's ESD diodes, which "phantom powers" the IMU and corrupts its reset state.

---

## 7. Cycle 4 Verification Plan

| Test | Equipment / Method | Pass Criterion |
|---|---|---|
| Output noise, 10 Hz–100 kHz | LNA (e.g., 40–60 dB gain) + spectrum analyzer, or an AP analyzer | < 10 µVrms at the IMU VDD pin |
| Switching ripple | 50 Ω coax probe with 1:1 termination, 20 MHz BW off | < 20 µVpp at the switching frequency |
| PSRR | Inject on V_IN with a line injector and measure with a VNA (e.g., Bode 100) | Matches the casc