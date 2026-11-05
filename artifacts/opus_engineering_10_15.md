# Low-Noise IMU Power Supply Architecture: TPS7A4700 (Cycle 10)

> **Assumption:** I don't have the outputs of Cycles 1–9. This design is therefore self-contained, with explicit assumptions. Replace the bracketed values with your locked requirements from earlier cycles.

---

## 1. Design Requirements and Assumptions

| Parameter | Assumed Value | Notes |
|---|---|---|
| System input | 9–32 V (vehicle/robotic bus) or 5 V USB/battery | Front end adapts to either |
| IMU class | MEMS tactical/industrial (e.g., ADIS16470, ICM-42688-P, BMI088) | |
| IMU analog rail (VDD) | 3.3 V | |
| IMU digital I/O rail (VDDIO) | 1.8 V or 3.3 V | Must match host MCU logic level |
| IMU supply current | 1 mA (consumer MEMS) to 50 mA (ADIS-class) | |
| Target rail noise | < 10 µVrms (10 Hz–100 kHz) | Keeps supply noise well below the IMU's own noise floor |
| Ripple at IMU pins | < 10 µVpp at the switching frequency | |
| Isolation goal | Keep switching noise, MCU digital currents, and motor/radio transients out of the IMU supply and ground | Galvanic isolation is an optional add-on (Section 8) |

---

## 2. Top-Level Architecture

```
 VBUS (9–32 V)
    │
 [TVS + reverse-polarity FET + input LC/EMI filter]
    │
 [Sync Buck Pre-Regulator]  → 4.0 V @ ~2.2 MHz (fixed freq, spread spectrum OFF)
    │                          (e.g., LMR33630 / TPS62130 / LMR36015)
    │
 [π-Filter: 10 µF – FB (600 Ω@100 MHz, low DCR) – 10 µF + 1 Ω/22 µF damping]
    │
    ├──► [TPS7A4700 #1]  3.3 V  → IMU VDD   (analog / sensor core)
    │         └─ local FB + 1 µF + 100 nF at IMU pin
    │
    └──► [TPS7A4700 #2]  1.8/3.3 V → IMU VDDIO (digital interface)
              └─ local 100 nF + 1 µF at IMU pin

 MCU / digital loads → separate buck or LDO branch (NOT shared with IMU LDOs)
```

**Key principles:**
1. Two-stage regulation. The buck provides efficiency. The LDO provides PSRR and a low noise floor.
2. Dedicated regulators. The IMU analog and IO rails are not shared with the MCU or radios.
3. Frequency planning. The buck frequency is placed above the IMU bandwidth and filtered passively, so the LDO only has to reject a small residual.

---

## 3. TPS7A4700 Configuration

### 3.1 Key Device Parameters (verify against the current datasheet)

| Parameter | Value |
|---|---|
| V_IN range | 3 V – 36 V |
| V_OUT range | 1.4 V – 20.5 V |
| I_OUT max | 1 A |
| Output noise | ~4.17 µVrms (10 Hz–100 kHz) |
| PSRR | ~82 dB at low frequency; >55 dB at 1 MHz |
| Dropout | ~307 mV at 1 A (much lower at IMU currents) |
| Package | 5×5 mm VQFN-20 (RGW) with thermal pad |

### 3.2 Output Voltage via ANY-OUT (no external divider, so no divider noise)

V_OUT = 1.4 V + Σ(grounded ANY-OUT pin weights). Ground the selected pins and leave the rest floating.

| Rail | Pins tied to GND | Calculation |
|---|---|---|
| **3.3 V** (VDD) | 1P6V, 0P2V, 0P1V | 1.4 + 1.6 + 0.2 + 0.1 = 3.3 V |
| **1.8 V** (VDDIO) | 0P4V | 1.4 + 0.4 = 1.8 V |

Using ANY-OUT keeps the noise gain at unity. An external resistor divider would amplify reference noise unless it is bypassed with a feed-forward capacitor.

### 3.3 Pin-Level Component Selection

| Pin | Component | Purpose |
|---|---|---|
| IN | 10 µF X7R (≥25 V) + 100 nF, placed close | Input decoupling; low source impedance for PSRR |
| OUT | 10 µF X7R ceramic + 100 nF | Stability (≥10 µF required) and transient response |
| NR/SS | 10 µF X7R (low leakage) | Filters reference noise; also sets soft-start (~tens of ms) |
| SENSE | Kelvin trace to the OUT cap at the IMU end | Removes trace IR drop |
| EN | Tied to IN through 100 kΩ, or driven by MCU/sequencer | Sequencing control |
| PG | 10–100 kΩ pull-up to VDDIO, routed to MCU | IMU-ready / fault flag |
| Thermal pad | Solid GND with vias | Thermal path and stable ground reference |

**Capacitor caution:** Specify X7R/C0G in 0805 or larger, derated for DC bias. Avoid Y5V. Class-II MLCCs can also be piezoelectric (microphonic) under vibration, which matters on an IMU board. For NR/SS and the IMU-adjacent capacitors, prefer C0G where values allow, or use soft-termination or polymer-tantalum parts.

---

## 4. Headroom and Thermal Budget

**Headroom:** At ≤50 mA, dropout is roughly ≤50 mV. However, PSRR degrades sharply near dropout, so the target is **V_IN − V_OUT ≥ 0.5–0.7 V**. The buck output is therefore set to **4.0 V** for both LDOs.

**Dissipation (worst case, ADIS-class load):**
- LDO #1: (4.0 − 3.3) × 0.05 A = **35 mW**
- LDO #2 at 1.8 V: (4.0 − 1.8) × 0.01 A = **22 mW**
- Junction temperature rise ≈ 0.035 W × ~38 °C/W ≈ **1.3 °C**, which is negligible.

Low dissipation also limits local heating of the IMU, which matters because MEMS bias is temperature sensitive. Even so, keep the buck and LDOs ≥15–20 mm from the IMU, or thermally separate them with a copper keep-out.

---

## 5. Noise and Ripple Budget

| Stage | Ripple at 2.2 MHz | Notes |
|---|---|---|
| Buck output | ~10 mVpp | Typical with ceramic output capacitors |
| After π-filter (~40 dB) | ~100 µVpp | LC corner around 50–100 kHz, damped |
| After TPS7A4700 (~45–50 dB at 2 MHz) | ~0.3–0.6 µVpp | PSRR is reduced at high frequency, but the π-filter has already done most of the work |
| After local FB + capacitors at IMU | < 0.3 µVpp | |

**Broadband noise:**
- The LDO contributes about 4.2 µVrms.
- The filtered residual ripple is negligible by comparison.
- **Total ≈ 4.5 µVrms**, which meets the < 10 µVrms target with about 2× margin.

**Why 2.2 MHz?** It sits well above IMU internal sampling and anti-alias bands (typically ≤ a few kHz ODR) and above the AM band. It also keeps the π-filter components small.

**Spread spectrum:** Disable it, or sync the buck to a fixed clock. Dithering spreads energy downward in frequency, toward bands where LDO PSRR is high but where aliasing into the IMU ADC is more likely. A fixed tone is easier to filter.

---

## 6. Ground and Layout Isolation Strategy

1. **Partitioned ground, single plane.** Use one continuous GND plane. Partition by placement, not by slots: the buck sits in one corner and the IMU in the opposite region, with no digital return currents crossing under the IMU.
2. **Star point.** Bring the LDO GND, NR capacitor ground, and IMU GND to a quiet local region connected to the main plane at one point near the LDO.
3. **Buck hot loop.** Keep the input cap, switch node, and output cap loop minimal. Keep the switch-node copper small and route it away from the LDO and IMU.
4. **SENSE routing.** Route SENSE as a Kelvin connection from the IMU-side capacitor so the regulation point is at the load.
5. **Guard copper.** Surround the IMU supply traces with GND pour and stitching vias at ≤λ/20 spacing for the highest harmonic of concern.
6. **Signal lines.** Add series 22–33 Ω resistors on SPI lines at the MCU to soften edges and reduce ground bounce coupled into the IMU.
7. **Mechanical.** Avoid placing large MLCCs directly under the IMU footprint. Board flex and capacitor microphonics can couple into the sensor.

---

## 7. Sequencing and Supervision

| Step | Action |
|---|---|
| 1 | The buck comes up. Its PG enables the LDOs, or the LDOs self-enable via an EN pull-up. |
| 2 | Bring up VDD (LDO #1) first, or simultaneously with VDDIO. Check the IMU datasheet: most MEMS IMUs tolerate either order, but VDDIO must not exceed VDD + 0.3 V on some parts. |
| 3 | The NR/SS capacitor gives a slow, monotonic ramp, which avoids inrush and IMU POR glitches. |
| 4 | Combine both PG outputs (wired-AND open-drain) to an MCU GPIO. Gate the IMU init and SPI traffic until PG is high plus the IMU's start-up time. |
| 5 | On a PG-low event, the MCU flags the IMU data as invalid and re-initializes after recovery. |

---

## 8. Optional: Galvanic Isolation Variant

If the IMU must be galvanically isolated (high-voltage motor drives, long-cable ground loops):

```
VBUS → isolated DC-DC (e.g., SN6505B push-pull + transformer, or UCC12050 module)
     → TPS7A4700 (post-regulation; PSRR handles the 400 kHz–8 MHz isolator ripple)
     → IMU
SPI  → digital isolator (e.g., ISO7741) between the MCU and IMU domains
```

- Allow more headroom (≥1 V), because isolated converter outputs vary with load.
- Isolated converters generate common-mode noise. Add a common-mode choke and a small Y-cap (if allowed) across the barrier.

---

## 9. Verification Plan

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise | LNA (e.g., 40–60 dB) plus spectrum analyzer, 10 Hz–100 kHz | < 10 µVrms |
| Switching ripple | 50 Ω coax probe at the IMU pin, 20 MHz BW off | < 10 µVpp at f_sw |
| PSRR | Injection transformer on the LDO input, sweep 10 Hz–10 MHz | Matches budget; no peaking from π-filter resonance |
| Load transient | IMU current steps (e.g., gyro enable) | < 5 mV deviation, no ringing |
| IMU-level noise | Allan variance, static 2–12 h: supply from the design vs. bench linear/battery reference | Angle random walk and bias instability within 5% of the battery reference |
| Vibration/microphonics | Shaker sweep with the IMU zeroed | No supply-correlated spurs in the gyro/accel PSD |
| Thermal | IR camera at full load | IMU-region ΔT < 2 °C from regulators |

The **Allan-variance comparison** against a battery-powered reference is the decisive test. It shows directly whether the supply limits IMU performance.

---

## 10. Cycle 10 Bill of Materials (Core)

| Ref | Part | Qty |
|---|---|---|
| U1 | Sync buck (LMR33630 / TPS62130 class), 4.0 V out | 1 |
| U2, U3 | TPS7A4700RGWR | 2 |
| FB1 | Ferrite bead, 600 Ω @ 100 MHz, ≤0.1 Ω DCR | 1 |
| FB2, FB3 | Ferrite bead at the IMU pins (lower Z, e.g., 120 Ω) | 2 |
| C_NR | 10 µF X7R, low leakage | 2 |
| C_OUT | 10 µF X7R 0805 + 100 nF C0G | 2 sets |
| C_IN | 10 µF X7R 25 V + 100 nF | 2 sets |
| R_damp | 1 Ω in series with 22 µF (π-filter damping) | 1 |

---

### Suggested Inputs for Cycle 11
- Lock the actual IMU part number and its VDD/VDDIO sequencing constraints.
- Decide between the non-isolated and galvanically isolated variants.
- Run a SPICE/PSRR simulation of the π-filter + TPS7A4700 using the TI PSpice model to confirm there is no resonance peaking.
- Do a layout review focused on the hot-loop area and the IMU keep-out zone.