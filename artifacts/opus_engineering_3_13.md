# Low-Noise IMU Power Architecture: TPS7A4700 (Cycle 3)

## 0. Scope and Assumptions

I don't have your Cycle 1 and Cycle 2 records, so this assumes a typical baseline. Replace the assumptions below with your actual parameters.

| Parameter | Assumed value |
|---|---|
| System input bus | 12 V nominal (9–16 V) |
| IMU | MEMS IMU with separate analog VDD (3.3 V) and digital VDDIO (1.8 V or 3.3 V) |
| IMU current | VDD ≤ 30 mA, VDDIO ≤ 10 mA |
| Goal | Keep supply-coupled noise below the IMU's intrinsic noise floor (gyro ARW and accel VRW), with no visible spurs in the 0.1 Hz–10 kHz sensor band |
| Cycle 3 focus | Pre-regulator spur suppression, rail separation, ground and return isolation, and verification |

---

## 1. Top-Level Architecture

```
 12 V BUS
   │
 [Input protection: TVS + reverse-polarity PFET + bulk/EMI π-filter]
   │
 [Pre-regulator: sync buck, 12 V → 5.0 V, f_sw ≥ 2 MHz, spread-spectrum OFF or controlled]
   │
 [2nd-stage LC / ferrite π-filter, f_c ≈ 10–30 kHz, damped]
   │
   ├──► U1 TPS7A4700 → 3.3 V  IMU_AVDD   (analog / sensor core)
   │
   └──► U2 TPS7A4700 → 1.8 V (or 3.3 V) IMU_VDDIO (digital I/O)
             │
          [Ferrite bead + local decoupling at IMU pins]
             │
            IMU
```

**Design rationale:**
- **Two-stage regulation.** The buck handles efficiency and the bulk voltage drop. The LDO handles noise and PSRR.
- **Separate LDOs for AVDD and VDDIO.** This prevents digital switching current from the SPI or I²C bus from modulating the sensor-core supply. This is the main "isolation" mechanism.
- **Switching frequency of 2 MHz or higher.** This places the fundamental far above the IMU bandwidth. It is also in a region where the LDO's PSRR, combined with the LC filter, still gives more than 80 dB of total rejection.

---

## 2. TPS7A4700 Key Characteristics

Confirm these values against the current datasheet revision.

| Parameter | Value | Relevance |
|---|---|---|
| Vin range | 3 V – 36 V | Accepts the 5 V pre-regulated rail |
| Iout max | 1 A | Large margin over the IMU load (more than 25×) |
| Output noise | ~4.17 µVrms (10 Hz–100 kHz) | Low-noise core spec |
| PSRR | ~82 dB at low frequency; remains above ~55 dB out to MHz range | Rejects buck ripple |
| Dropout | ~307 mV at 1 A (far less at 30 mA) | Headroom is driven by PSRR, not dropout |
| Vout setting | ANY-OUT pins: Vout = 1.4 V + sum of grounded pins (0.1/0.2/0.4/0.8/1.6/3.2/6.4/12.8 V) | No external resistors, so no resistor noise or tolerance error |
| Package | 5×5 mm VQFN-20 (RGW) with thermal pad | |

---

## 3. Rail Design

### U1: IMU_AVDD = 3.3 V
- **ANY-OUT setting:** 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V**. Tie the 1P6V, 0P2V, and 0P1V pins to GND and leave the others open.
- **Vin:** 5.0 V, giving about 1.7 V of headroom. Keep headroom at or above ~1 V, because PSRR degrades sharply near dropout.
- **Dissipation:** (5.0 − 3.3) × 0.03 A ≈ 51 mW. This is thermally negligible, which also avoids temperature drift near the IMU.

### U2: IMU_VDDIO = 1.8 V
- **ANY-OUT setting:** 1.4 + 0.4 = **1.8 V**. Tie 0P4V to GND.
- **3.3 V option:** use the same configuration as U1.
- **Dissipation:** (5.0 − 1.8) × 0.01 A ≈ 32 mW.
- **Optional cost-down:** U2 could be a lower-cost LDO (TPS7A20 class), because VDDIO noise is less critical. For Cycle 3 I recommend keeping both rails on the TPS7A4700. Using the same part reduces BOM variants and removes VDDIO as a debug variable.

### Passive Components (Both LDOs)

| Ref | Value | Type | Purpose |
|---|---|---|---|
| C_IN | 10 µF + 0.1 µF | X7R, rated ≥ 16 V | Input bypass, placed at the IN pins |
| C_OUT | 10 µF + 0.1 µF | X7R, low ESR | Stability and transient response; follow the datasheet's minimum value and ESR range |
| C_NR/SS | 10 nF (start value; evaluate up to ~1 µF) | C0G/NP0 or film | Filters reference noise. A larger value lowers low-frequency noise but lengthens soft-start |
| R_PG | 10–100 kΩ to VDDIO or the MCU rail | — | Power-good pull-up |
| EN | Driven by sequencing logic, or resistor-divided from Vin | — | Enable |

**Capacitor selection notes:**
- **Avoid X5R/Y5V on NR/SS.** Use C0G/NP0 or film to avoid microphonic (piezoelectric) noise. This matters on vibrating platforms that carry an IMU.
- **Account for DC-bias derating.** Check the effective capacitance of the 10 µF ceramics at 3.3 V and 5 V. Use 1206 or 0805 sizes rated at 25 V where possible.

---

## 4. Pre-Regulator and Inter-Stage Filter

**Buck converter:**
- 12 V → 5.0 V synchronous buck (LMR33630 / TPS62130 class).
- Use a fixed switching frequency of at least 2 MHz, synchronized to a system clock if available. This puts the spurs at a deterministic location instead of letting them wander.
- Use spread-spectrum only if EMC requires it. If enabled, confirm that the modulation frequency does not fold into the IMU band.

**π-filter (buck → LDO inputs):**
- Topology: C1 (10 µF) – L (4.7–10 µH, shielded) – C2 (10 µF), plus a damping network (1–2 Ω in series with 22 µF electrolytic or polymer).
- Corner frequency f_c ≈ 1/(2π√(LC)) ≈ 16 kHz with 10 µH and 10 µF. This gives roughly 80 dB of attenuation at 2 MHz, before the LDO's PSRR.
- The combined rejection target at f_sw is above 120 dB, so the ripple at AVDD should sit below the LDO's own noise floor.

**Local decoupling at the IMU pins:**
- Place a ferrite bead (≥ 600 Ω at 100 MHz, low DCR) on each rail.
- Follow it with the IMU datasheet's recommended decoupling, typically 0.1 µF + 1–10 µF per pin, placed within 2 mm.
- Check that the bead plus capacitor does not resonate near the IMU's internal drive or sense frequency. MEMS gyro drive frequencies are typically 20–30 kHz.

---

## 5. Isolation Strategy

| Domain | Measure |
|---|---|
| **Supply** | Dedicated LDO per IMU rail. No other loads on IMU_AVDD. |
| **Ground** | Use a single solid ground plane. Do not split it, because splits create slot antennas and break return paths. Partition the placement so buck return currents never flow under the IMU. Tie the buck PGND to the plane at one point near its input capacitors. |
| **Layout** | Place the buck at least 25 mm from the IMU on the opposite PCB edge, with the inductor oriented away. Place the LDOs adjacent to the IMU zone. Do not route digital or switching nets under the IMU or LDO NR traces. Keep the NR/SS trace short and guarded by ground. |
| **Signal** | Add series resistors (22–33 Ω) on SPI lines at the MCU. Use a digital isolator (ISO7741 class) only if the IMU must sit on a separately grounded sub-board. |
| **Thermal** | Connect the LDO thermal pad to the plane with vias. Keep the LDOs out of the IMU's thermal footprint to limit bias drift. |
| **Mechanical** | If the IMU sits on an isolated or damped sub-board, feed its power through a flex cable or connector. Place the LDOs on the IMU side so the cable carries only the filtered pre-LDO rail, and the LDO absorbs any noise the cable picks up. |

---

## 6. Sequencing

1. The buck reaches regulation. Its PG signal enables U1 (AVDD).
2. U1's soft-start, set by C_NR/SS, ramps AVDD over about 1–10 ms.
3. U1's PG asserts and enables U2 (VDDIO). Alternatively, tie both EN pins together if the IMU datasheet allows simultaneous ramp.
4. The MCU waits for U2's PG plus the IMU's startup time before issuing SPI commands.
5. **Power-down:** confirm the IMU's specified ordering. Many IMUs require VDDIO ≤ VDD + 0.3 V at all times. Add a Schottky clamp if U2 could ever lead U1.

---

## 7. Noise Budget (Estimate)

| Source | Contribution at AVDD (10 Hz–100 kHz) |
|---|---|
| TPS7A4700 intrinsic noise | ~4–5 µVrms |
| Buck ripple after π-filter and PSRR | < 1 µVrms equivalent (spur at f_sw, out of band) |
| VDDIO digital coupling | < 1 µV (separate LDO plus ground partitioning) |
| **Total** | **≈ 5 µVrms** |

Compare this total with the IMU's power-supply sensitivity specification (often given in °/s/V or mg/V). Typical results put supply-induced error well below the sensor's intrinsic ARW.

---

## 8. Cycle 3 Verification Plan

1. **Output noise.** Measure each rail with an LNA (≥ 40 dB gain) and an FFT analyzer, using AC coupling with a sub-1 Hz corner. Pass criterion: within 20% of the datasheet curve, with no spurs above −120 dBV in the 1 Hz–10 kHz band.
2. **PSRR.** Inject a 100 Hz–5 MHz sweep onto the 5 V rail (Bode 100 plus injector). Verify more than 60 dB of rejection at f_sw.
3. **Load transient.** Apply a 0–30 mA step on AVDD. Deviation should be under 10 mV with no ringing, which confirms C_OUT stability.
4. **Correlation test.** Record IMU static data (Allan variance over at least 2 hours) under two configurations:
   - (a) the LDO path;
   - (b) a battery or bench linear reference.
   
   Pass criterion: ARW, VRW, and bias instability match within measurement uncertainty.
5. **Spur sweep.** Step the buck frequency or toggle the system loads, and check the IMU output FFT for injected tones.
6. **Thermal.** Run a −40 to +85 °C sweep with the IMU bias logged, to confirm the LDO adds no thermal drift artifacts.
7. **Vibration (if applicable).** Run a shaker test to confirm there is no microphonic contribution from the MLCCs.

---

## 9. Bill of Materials (Core Power Path)

| Qty | Part | Function |
|---|---|---|
| 2 | TPS7A4700RGWR | AVDD / VDDIO LDOs |
| 1 | LMR33630 (or equivalent) | 12 V → 5 V pre-regulator |
| 1 | 10 µH shielded inductor | π-filter |
| 2 | C0G 10 nF (0603) | NR/SS capacitors |
| 4+ | 10 µF X7R 25 V (1206) | LDO input/output capacitors |
| 2 | Ferrite bead (BLM18 class) | IMU-pin isolation |
| 1 | TVS + PFET | Input protection |

---

If you can share the specific IMU part number and the Cycle 2 findings (measured spurs, failure modes), I can tighten the filter corners, sequencing, and noise budget to that sensor.