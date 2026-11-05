# Low-Noise IMU Power Supply Architecture (Cycle 8)
**Core regulator:** Texas Instruments TPS7A4700 (36 V, 1 A, ultra-low-noise positive LDO)

> **Note on continuity:** I don't have the outputs of Cycles 1–7. This design states its assumptions explicitly so you can reconcile it with prior decisions such as the IMU part number, input bus voltage, and board stack-up.

---

## 1. Design Assumptions

| Parameter | Assumed Value | Rationale |
|---|---|---|
| System input bus | 12 V (9–16 V range) | Typical vehicle/robot/avionics bus |
| IMU | MEMS IMU, e.g., ADIS16470 / ICM-42688 / BMI088 class | 3.3 V analog/core supply |
| IMU VDD current | ≤ 50 mA typical, ≤ 100 mA peak | Includes heater/self-test transients |
| VDDIO | 3.3 V (or 1.8 V), separate rail | Isolates digital I/O current from sensor core |
| Noise target on VDD | < 10 µVrms (10 Hz–100 kHz) | Keeps supply-induced bias/noise below the IMU's intrinsic noise floor |
| Galvanic isolation | Optional (Section 7) | Only if the system ground is noisy or shared with motors |

---

## 2. Architecture Overview

```
 12 V BUS
   │
   ▼
[Input Protection]  TVS (SMBJ18A) + reverse-polarity PFET + bulk 47 µF
   │
   ▼
[EMI Filter]        Common-mode choke + π-filter (10 µF – 2.2 µH – 10 µF)
   │
   ▼
[Pre-Regulator]     Synchronous buck, 12 V → 4.0 V, fsw ≥ 2 MHz
   │                (e.g., TPS62913 low-ripple buck, or TPS62162)
   ▼
[Post-Buck Filter]  Ferrite bead (600 Ω @100 MHz) + 22 µF ceramic
   │
   ├──────────────────────────────┐
   ▼                              ▼
[TPS7A4700  #1]              [TPS7A4700 #2 or TPS7A20]
 3.3 V IMU_VDD (analog/core)   3.3 V/1.8 V IMU_VDDIO (digital)
   │                              │
 [Local LC / FB + 10 µF]       [FB + 4.7 µF]
   │                              │
   ▼                              ▼
  IMU VDD pin                   IMU VDDIO pin
```

**Why two stages:** The buck provides efficient step-down from 12 V. The TPS7A4700 then provides ripple rejection and ultra-low noise. Dropping 12 V → 3.3 V directly in the LDO would be thermally acceptable at 50 mA (~0.44 W). However, a pre-regulator keeps LDO headroom in the optimal PSRR region and makes the design scalable.

---

## 3. TPS7A4700 Key Specifications

*Verify all values against the current TI datasheet (SBVS204).*

| Spec | Value |
|---|---|
| Input range | 3 V – 36 V |
| Output range | 1.4 V – 20.5 V (ANY-OUT™ or resistor-set) |
| Output current | Up to 1 A |
| Output noise | ~4.17 µVrms (10 Hz–100 kHz) |
| PSRR | ~82 dB at 100 Hz; ≥ 55 dB out to ~10 MHz |
| Dropout | ~307 mV at 1 A |
| Package | 5 mm × 5 mm VQFN-20 (RGW), thermal pad |
| Output cap | Stable with ≥ 10 µF ceramic |

---

## 4. Output Voltage Programming (ANY-OUT, No External Resistors)

V_OUT = 1.4 V (reference) + the sum of the ANY-OUT pins tied to GND.

ANY-OUT pin weights: 100 mV, 200 mV, 400 mV, 800 mV, 1.6 V, 3.2 V, 6.4 V.

**For 3.3 V:** 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V**
- Ground the **1.6 V, 200 mV, 100 mV** pins.
- Leave all other ANY-OUT pins floating.

**For 1.8 V (VDDIO option):** 1.4 + 0.4 = 1.8 V. Ground the 400 mV pin only.

Advantage: there is no resistor-divider noise gain or resistor tolerance stacking. Accuracy is set by the internal reference (~±1% over temperature).

---

## 5. Component Selection (LDO #1: IMU_VDD)

| Ref | Function | Value / Part | Notes |
|---|---|---|---|
| C_IN | LDO input | 10 µF X7R, 25 V, 1206 | Place within 2 mm of the IN pins |
| C_NR | Noise-reduction/soft-start | 10 µF X7R (≥ 1 µF minimum) | Larger C_NR lowers low-frequency noise and lengthens soft-start |
| C_OUT | Output | 10 µF X7R + 100 nF C0G | Low ESL, close to the OUT pins |
| SENSE | Remote sense | Kelvin trace to the IMU VDD pad | Compensates IR drop in the trace and ferrite |
| EN | Enable | Driven by the MCU GPIO or tied to IN via 100 kΩ | Enables supply sequencing (Section 8) |
| FB_LOCAL | Post-LDO filter | Ferrite, e.g., BLM18KG (low DCR) | Optional; adds HF isolation from IMU digital switching |
| C_IMU | IMU decoupling | Per IMU datasheet (e.g., 100 nF + 1 µF) | Directly at the IMU pins |

**Ferrite caution:** Ferrite DCR combined with the IMU's load step causes droop. Also check the LC resonance of the ferrite with C_IMU. Add a 1 Ω series damping resistor or a lossy bead if Q > 2.

---

## 6. Performance Budget

**Headroom:** V_IN(LDO) − V_OUT = 4.0 − 3.3 = 0.7 V. This is more than 2× dropout at this current, which keeps PSRR near its datasheet value.

**Ripple attenuation chain (at the 2 MHz buck switching frequency):**

| Stage | Attenuation |
|---|---|
| Buck output ripple | ~10 mVpp (low-ripple buck) |
| Ferrite + 22 µF | ~ −20 dB → 1 mVpp |
| TPS7A4700 PSRR at 2 MHz | ~ −55 dB → ~1.8 µVpp |
| Post-LDO ferrite + C | additional −10 to −20 dB |
| **Residual at IMU** | **< 1 µVpp switching ripple** |

**Thermal:** P_D = (4.0 − 3.3) V × 0.1 A = 70 mW. ΔT ≈ 70 mW × ~35 °C/W ≈ 2.5 °C. This is negligible.

**Thermal stability matters for IMUs.** The low dissipation prevents the LDO from creating a local thermal gradient that would drift the gyro/accelerometer bias. Even so, place the LDO at least 10 mm from the IMU, and never directly under it.

---

## 7. Isolation Strategy

### Level 1: Power-Domain Isolation (Baseline)
- Use a dedicated LDO for the IMU core and a separate rail for VDDIO.
- Create a split analog ground island (AGND_IMU) under the IMU and LDO #1.
- Join AGND_IMU to system GND at a single point near the LDO GND pin, using a 0 Ω resistor or net-tie for debug.
- Do not route any digital or switching traces across the AGND_IMU island. Keep layer 2 solid ground beneath it.

### Level 2: Galvanic Isolation (Optional: Motors or Long Harnesses)
```
12 V ─► Isolated DC/DC (e.g., SN6505B + transformer, or UCC12050)
         ─► 5 V_ISO ─► TPS7A4700 (3.3 V) ─► IMU
Digital: SPI through ISO7741 / ISO6741 digital isolator
```
- The TPS7A4700 is essential here. Isolated converters carry significant ripple and common-mode noise, and its high PSRR cleans that up.
- Add a Y-capacitor (~100 pF to 1 nF) across the isolation barrier to return common-mode noise.

---

## 8. Sequencing and Protection

1. The buck comes up first. The PGOOD output of the buck drives EN of LDO #1.
2. VDD rises with a soft-start set by C_NR, giving a monotonic ramp (check the IMU's ramp-rate requirement).
3. VDDIO is enabled on or after VDD, per the IMU datasheet. Many IMUs forbid VDDIO > VDD + 0.3 V.
4. Add an output TVS or Zener (3.6 V) on IMU_VDD only if the IMU absolute maximum is close to 3.6 V. Note that the LDO already has current limit and thermal shutdown.

---

## 9. PCB Layout Rules

- Keep the C_IN, C_OUT, and C_NR loops minimal on the same layer as the LDO, with multiple GND vias.
- Use a thermal pad with at least 9 vias to the inner GND plane.
- Route SENSE as a separate Kelvin trace from the IMU pad, away from the SPI clock.
- Place the buck at the opposite board edge, at least 20 mm from the IMU, and orient the inductor so its field axis points away from the IMU.
- Guard-ring the IMU_VDD trace with ground and stitching vias every ~λ/20 at relevant frequencies.
- Place the IMU at the board's mechanical neutral axis and away from mounting screws. This is a mechanical rather than electrical constraint, but it interacts with placement.

---

## 10. Verification Plan (Cycle 8 Exit Criteria)

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise | LNA (e.g., 40 dB) + spectrum analyzer, 10 Hz–100 kHz | < 10 µVrms at the IMU pin |
| Switching ripple | 50 Ω coax probe, 1 GHz BW | < 50 µVpp |
| Load transient | 0 → 100 mA step, 1 µs edge | < 10 mV deviation, no ringing > 2 cycles |
| PSRR in-circuit | Injection transformer (e.g., Picotest J2120A) | > 50 dB at 2 MHz |
| IMU noise impact | Allan deviation, LDO vs. bench battery supply | ARW/VRW within 5% of the battery reference |
| Thermal drift | Chamber −40 to +85 °C | No supply-correlated bias shift |

---

## 11. BOM Summary (IMU_VDD Path)

| Qty | Part | Description |
|---|---|---|
| 1 | TPS7A4700RGWR | Ultra-low-noise LDO |
| 1 | TPS62913 (or equivalent) | Low-ripple buck pre-regulator |
| 3 | 10 µF X7R 25 V 1206 | C_IN, C_NR, C_OUT |
| 2 | 100 nF C0G 0402 | HF decoupling |
| 2 | BLM18KG601 | Ferrite beads (pre- and post-LDO) |
| 1 | SMBJ18A | Input TVS |
| opt. | ISO7741 + UCC12050 | Galvanic isolation option |

---

**Recommended next step (Cycle 9):** Run a SPICE noise and PSRR simulation using TI's TPS7A47 PSpice/TINA model with your actual IMU load profile. Then build a single-board prototype to perform the Allan-deviation comparison against a battery-powered reference.