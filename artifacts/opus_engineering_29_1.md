# Low-Noise IMU Power Architecture: TPS7A4700 (Cycle 29)

*I don't have the earlier cycles, so this design states its assumptions explicitly. Adjust where they differ from your baseline.*

**Assumptions:** 12–28 V system bus, MEMS IMU (e.g., ADIS16470-class or ICM-42688-class) needing a 3.3 V analog/core rail at ≤150 mA, plus a separate VDDIO rail.

---

## 1. Architecture Overview

```
 VBUS (12–28 V)
   │
   ├─[TVS + reverse-pol FET + input LC]
   │
   ▼
 Stage 1: Sync Buck Pre-Regulator  (e.g., LMR33630 / TPS62130, fsw ≈ 2 MHz, spread-spectrum OFF or characterized)
   │  Vout = 4.5 V
   ▼
 Stage 2: Damped LC π-filter  (ferrite + 2.2 µH + 22 µF ceramic + 47 µF polymer damping)
   │
   ├────────────────────────────┐
   ▼                            ▼
 Stage 3a: TPS7A4700          Stage 3b: Digital LDO (e.g., TPS7A20)
   3.30 V  →  IMU VDD (analog)    3.3 V / 1.8 V → IMU VDDIO + MCU interface
   │
   ▼
 Stage 4: Local ferrite + 0.1 µF / 1 µF at IMU pins
```

**Design principle:** The buck handles efficiency. The π-filter removes the MHz ripple where LDO PSRR rolls off. The TPS7A4700 sets the noise floor in the IMU's sensitive band (DC to ~1 kHz).

---

## 2. TPS7A4700 Key Parameters

| Parameter | Value (datasheet typ.) | Relevance |
|---|---|---|
| Vin range | 3 V – 36 V | Accepts 4.5 V post-filter rail |
| Output noise | ~4.17 µVrms (10 Hz–100 kHz) | Well below IMU ADC LSB-referred supply sensitivity |
| PSRR | ~82 dB @ 100 Hz; ~55 dB @ 10 MHz | Rejects buck residuals |
| Dropout | ~307 mV @ 1 A | Ample margin at IMU currents |
| Iout max | 1 A | Large headroom; low self-heating |
| Package | 5×5 mm VQFN-20 (RGW) w/ thermal pad | Thermal pad to GND plane |

**Verify all values against the current datasheet revision before release.**

---

## 3. Output Voltage Setting (ANY-OUT, No External Resistors)

Vout = 1.4 V (reference) + the sum of grounded ANY-OUT pins.

For **3.3 V**: tie **1P6V + 0P2V + 0P1V** to GND, and leave the others open.
1.4 + 1.6 + 0.2 + 0.1 = **3.30 V**

Using ANY-OUT avoids resistor noise and tolerance stack-up. The internal network also keeps noise approximately independent of Vout.

---

## 4. Headroom & Thermal

- Vin − Vout = 4.5 − 3.3 = **1.2 V**. This is well above dropout and keeps the pass device out of the region where PSRR degrades.
- At 150 mA: Pd = 1.2 × 0.15 = **0.18 W**. With θJA ≈ 35 °C/W, the rise is about 6 °C. That is negligible, which matters because thermal gradients near the IMU cause bias drift.

---

## 5. Component Selection

| Ref | Value | Notes |
|---|---|---|
| C_IN | 10 µF X7R + 0.1 µF | At IN pins, ≥10 V rating |
| C_OUT | 10–22 µF X7R ceramic | Stable with low-ESR ceramics; check DC-bias derating |
| C_NR (NR/SS) | 1 µF low-leakage (C0G/film if feasible) | Larger values lower low-frequency noise and lengthen soft-start. Confirm the range in the datasheet |
| π-filter L | 2.2 µH shielded, Isat > 0.5 A | fc ≈ 23 kHz with 22 µF |
| π-filter damping | 47 µF polymer (ESR ~50–100 mΩ) or RC snubber | Prevents LC peaking near fc |
| Local IMU decoupling | FB (600 Ω @100 MHz) + 1 µF + 0.1 µF | Placed at IMU VDD pin |
| EN | RC from VDDIO rail or MCU GPIO | Enforces the IMU sequencing requirement |

---

## 6. Isolation Strategy

**Noise isolation (baseline):**
- Use separate analog (TPS7A4700) and digital (VDDIO LDO) rails, so IMU I/O switching currents don't modulate the sensor supply.
- Use a single solid ground plane with partitioned placement (no split). Keep buck return currents physically away from the IMU, with the LDO between them.
- Place the IMU on the far side of the board from the buck inductor, and orient the inductor so its field is not coupled into the IMU.

**Galvanic isolation (if required by the system):**
- Insert an isolated DC-DC stage (e.g., SN6505B transformer driver plus rectifier) ahead of the π-filter, producing about 5 V isolated.
- Use a digital isolator (e.g., ISO7741) on the SPI lines.
- The TPS7A4700 then cleans the isolated converter's ripple. This is the dominant motivation for a high-PSRR LDO in this topology.

---

## 7. Layout Rules

1. Use a Kelvin connection from FB/sense to the IMU supply pin if using an external-resistor mode. With ANY-OUT, keep the OUT trace short and wide.
2. Place the C_NR return directly at the LDO GND pin. Keep the NR node very short and guarded, because it is high impedance.
3. Stitch the thermal pad with at least 9 vias to the internal GND plane.
4. Keep the switch node of the buck at least 15 mm from the IMU, with no traces routed under the IMU.
5. Mount the IMU symmetrically relative to copper to reduce thermo-mechanical stress.

---

## 8. Noise Budget (Indicative)

| Source | Contribution at IMU VDD |
|---|---|
| TPS7A4700 intrinsic | ~4.2 µVrms (10 Hz–100 kHz) |
| Buck ripple (~20 mVpp @ 2 MHz) after π-filter (~−40 dB) and LDO (~−55 dB) | ≪ 1 µV |
| Digital coupling (with rail separation) | Layout-dependent; target < 2 µV |
| **Target total** | **< 10 µVrms** |

Compare this against the IMU's specified PSRR / supply sensitivity (°/s/V, mg/V) to confirm the margin.

---

## 9. Verification Plan

- **Output noise:** Use an LNA plus spectrum analyzer from 10 Hz to 100 kHz, battery-powered, in a shielded box.
- **PSRR:** Inject with a line injector (e.g., Picotest J2120A) and use a Bode analyzer from 10 Hz to 10 MHz.
- **Transient:** Apply a 0–100 mA load step at IMU wake-up and confirm < 10 mV deviation.
- **System test:** Measure IMU Allan deviation with the bench linear supply vs. this design. Bias instability should be indistinguishable.
- **Thermal:** IR image at full load, checking the gradient across the IMU footprint.

---

**Deliverables for next cycle:** schematic capture, the π-filter damping simulation (Bode of the LC network), and the IMU-specific supply-sensitivity check once the exact sensor part number is confirmed.