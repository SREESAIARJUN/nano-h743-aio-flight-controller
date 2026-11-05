# Low-Noise Power Architecture: TPS7A4700 for IMU Rail Isolation (Cycle 27)

> **Context note:** I don't have the outputs of earlier cycles, so this is a self-contained baseline. The assumed requirements are in §1. Swap in your actual IMU part, input bus, and noise budget, and the numbers will propagate.

---

## 1. Design Assumptions

| Parameter | Assumed value | Rationale |
|---|---|---|
| Input bus | 12 V nominal (9–16 V) | Typical vehicle or robotics bus |
| IMU class | Tactical/industrial MEMS (e.g., ADIS1649x-class, 3.3 V) or consumer (BMI088/ICM-42688-class) | Covers both cases |
| IMU analog rail | 3.3 V, ≤ 100 mA | Includes margin for heater or high-ODR modes |
| IMU digital/IO rail | 3.3 V or 1.8 V, ≤ 50 mA | Kept separate from the analog rail |
| Rail noise target | < 10 µVrms (10 Hz–100 kHz) at the IMU pin | Below the IMU's internal regulator rejection floor |
| Critical bands | DC–1 kHz (sensor bandwidth) and ~10–30 kHz (MEMS gyro drive/sense resonance) | These are the bands where supply ripple aliases into bias or ARW |

---

## 2. Architecture Overview

```
 12 V BUS
   │
   ├─[TVS + reverse-pol + π-filter (EMI)]
   │
   ▼
 ┌──────────────────────────┐
 │ Stage 1: Sync Buck        │  12 V → 4.3 V
 │ (e.g., TPS62913 / LMR33630)│  fsw ≥ 2 MHz, spread-spectrum OFF
 └──────────┬───────────────┘
            │
   [Post-filter: Ferrite (600 Ω@100MHz) + 10 µF polymer + 2nd-stage LC]
            │
     ┌──────┴───────────────────────┐
     ▼                              ▼
 ┌─────────────────┐        ┌──────────────────┐
 │ Stage 2A:        │        │ Stage 2B:         │
 │ TPS7A4700 #1     │        │ TPS7A4700 #2      │
 │ 4.3 V → 3.3 V    │        │ (or TPS7A20 for   │
 │ IMU ANALOG (VDD) │        │  cost) → VDDIO    │
 └────────┬────────┘        └────────┬─────────┘
          │                          │
   [FB + local C, star to AGND]   [FB + local C]
          ▼                          ▼
       IMU VDD                    IMU VDDIO / MCU-side level IO
```

**Principle:** The buck handles efficiency. The TPS7A4700 handles rejection and noise. Analog and digital IMU domains never share a regulator, so digital edge current cannot modulate the analog reference.

---

## 3. Stage 1: Pre-Regulator

**Choosing the switching frequency:**
- Run at **≥ 2 MHz**. This places the fundamental far above both the IMU bandwidth and the MEMS resonance band (10–30 kHz).
- **Disable spread-spectrum** if it is offered. FM dithering spreads energy into the lower bands, where LDO PSRR is high but the IMU is sensitive to sidebands. A fixed tone at 2 MHz is easier to filter.
- **Synchronize** the buck to the system clock if possible. This keeps intermod products with the IMU's ADC sample rate deterministic.

**Setting headroom:**
- The TPS7A4700's dropout is about 307 mV at 1 A, and much lower at 100 mA. However, **PSRR collapses near dropout**.
- Target **V_in − V_out ≈ 1.0 V** (4.3 V → 3.3 V). This keeps high-frequency PSRR near datasheet levels while limiting dissipation.

**Post-filter:**
- Use a ferrite bead followed by a second-order LC. Choose a corner of about 20 kHz, for example 4.7 µH with 22 µF.
- Add damping with an ESR of about 0.1–0.3 Ω in a polymer capacitor, or with an RC snubber. This prevents a resonant peak landing in the gyro band.
- Attenuation needed at 2 MHz is about 40 dB from the LC, plus about 55 dB of LDO PSRR at 1 MHz or higher.

---

## 4. Stage 2: TPS7A4700 Configuration

### 4.1 Setting the output with ANY-OUT (no external divider)

V_OUT = 1.4 V (reference) + selected pin weights.

**For 3.3 V:**

| Pin | Weight | Connection |
|---|---|---|
| 1P6V | +1.6 V | GND |
| 200mV | +0.2 V | GND |
| 100mV | +0.1 V | GND |
| All others | — | Float |

1.4 + 1.6 + 0.2 + 0.1 = **3.3 V**

Using ANY-OUT instead of resistors avoids the divider's thermal noise and gain-up of the reference noise. This is the main reason to prefer it here.

**For 5.0 V** (if the IMU or a heater needs it): use 3P2V + 400mV, giving 1.4 + 3.2 + 0.4 = 5.0 V.

### 4.2 Passive components

| Node | Component | Notes |
|---|---|---|
| IN | 10 µF X7R + 1 µF X7R, at the pin | Low-ESL; place the bulk polymer upstream |
| NR/SS | 10 nF C0G/NP0 (datasheet-characterized value) | Sets the noise-filter corner and soft-start. Larger values lower 10–100 Hz noise at the cost of slower startup. **Use C0G**, because X7R is microphonic under IMU vibration. |
| OUT | ≥ 10 µF (stability minimum), e.g., 22 µF polymer/tantalum + 1 µF C0G | See §6 for why MLCC-only is avoided |
| EN | RC from IN, or MCU-controlled | Lets you sequence VDD before VDDIO when the IMU requires it |
| PG | Pull-up to VDDIO | Hold the IMU in reset or the MCU in wait until power is good |

> Verify the exact NR/BYP capacitor recommendations and output-cap ESR/stability range against the current TPS7A47 datasheet revision before layout freeze.

### 4.3 Key performance figures (datasheet typicals)
- Output noise: **4.17 µVrms** (10 Hz–100 kHz)
- PSRR: **82 dB at 100 Hz**, **> 55 dB at 1 MHz**
- I_out max: 1 A, which is far above the IMU's needs. The large margin keeps the pass device in a low-noise, well-regulated region.

---

## 5. Noise and Ripple Budget

| Source | At LDO input | LDO rejection | At IMU pin |
|---|---|---|---|
| Buck ripple at 2 MHz (after LC) | ~100 µVpp | ~55 dB | **~0.2 µVpp** |
| Buck low-frequency noise (100 Hz–10 kHz) | ~50 µVrms | ~70–80 dB | **< 0.02 µVrms** |
| LDO intrinsic noise | — | — | **~4.2 µVrms** |
| Ground-path IR/inductive pickup | Layout-dependent | — | Target < 2 µVrms |
| **Total (RSS)** | | | **≈ 5 µVrms**, meeting the < 10 µVrms target |

**Conclusion:** The LDO's own intrinsic noise dominates. Upstream sources are effectively eliminated, which confirms the cascade is correctly partitioned.

---

## 6. IMU-Specific Isolation Measures

1. **Microphonics.** Under vibration, Class II MLCCs (X5R/X7R) generate voltage through the piezoelectric effect. That voltage couples directly onto the rail and appears as vibration-correlated bias. Use **C0G, tantalum, or polymer** for NR and for the last-stage output capacitors.

2. **Thermal isolation.** The LDO dissipates (4.3 − 3.3 V) × 0.1 A ≈ 0.1 W. Place it **≥ 15–20 mm from the IMU**, or across a slotted copper boundary. Thermal gradients across the IMU drive bias drift more strongly than electrical noise does.

3. **Grounding.**
   - Use a single AGND island under the IMU and the LDO output network.
   - Tie it to PGND/DGND at **one point**, near the LDO GND pin.
   - Do not route buck switch-node return current under the IMU.

4. **Local decoupling at the IMU.**
   - Place a ferrite bead (low DCR, e.g., ~0.05 Ω) followed by 1 µF C0G and 100 nF C0G at the VDD pin.
   - Check the combination for an LC peak. If the bead-capacitor resonance falls near the gyro drive frequency, add damping resistance.

5. **Mechanical decoupling.** If the IMU sits on an isolator or damped sub-board, route its power through a flex or soft cable. The power trace should not form a rigid vibration path.

6. **Sequencing.** Bring up VDD (from LDO #1) first, then VDDIO (from LDO #2) once PG asserts. This prevents back-powering through IO protection diodes. Check this against the specific IMU's datasheet requirements.

---

## 7. Layout Checklist

- [ ] Buck switch node is minimal-area and on the opposite board region from the IMU
- [ ] TPS7A4700 thermal pad has a via array to a GND pour, isolated from the IMU's copper
- [ ] NR capacitor is placed directly at the pin and returned to the LDO's quiet GND, not shared
- [ ] Output sense and feedback are Kelvin-connected at the load if a long trace is unavoidable
- [ ] No digital traces (SPI clock especially) run under or adjacent to VDD
- [ ] SPI lines have series 22–33 Ω resistors to slow edges into the IMU
- [ ] Guard pour is stitched around the analog rail

---

## 8. Verification Plan

| Test | Method | Pass criterion |
|---|---|---|
| Rail noise | LNA (e.g., 40–60 dB gain) into a spectrum analyzer, 10 Hz–100 kHz | < 10 µVrms; no spurs in 10–30 kHz |
| PSRR | Inject on the buck output with a line-injection transformer; Bode analyzer | ≥ 50 dB to 1 MHz in-circuit |
| Load transient | 10 → 100 mA step | < 5 mV deviation, no ringing |
| IMU-level | 2–4 h static Allan variance; compare battery-powered vs. production rail | ARW and bias instability within 5% of battery baseline |
| Vibration | Shaker sweep with and without the C0G substitution | No vibration-correlated rail content above the noise floor |
| Thermal | IR camera at max ambient | IMU gradient < 0.5 °C across the package |

**The decisive test is the Allan deviation comparison.** If the production rail matches the battery-powered baseline, the supply is no longer a contributor to IMU error.

---

## 9. Cycle 27 Open Items

1. Confirm the IMU part number. This sets VDD/VDDIO levels, current, and sequencing.
2. Decide between TPS7A4700 and TPS7A20 for VDDIO, based on cost and noise needs. The digital rail rarely needs 4 µVrms.
3. Measure the actual MEMS drive frequency, then verify the post-filter and bead resonance avoid it.
4. If the bus can drop below about 5 V, re-check buck headroom. Alternatively, consider a TPS7A4700 → TPS7A20 cascade.

If you share the prior cycle's requirements or the specific IMU, I can tighten the component values and the noise budget.