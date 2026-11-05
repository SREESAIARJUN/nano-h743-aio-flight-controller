# Low-Noise IMU Power Supply Architecture (Cycle 14)

**Core regulator:** TI TPS7A4700 (36 V, 1 A, 4.17 µV RMS ultra-low-noise LDO)

> **Assumptions.** I don't have the records from Cycles 1–13, so this is a self-contained baseline built on these assumptions:
> - 24 V or 28 V vehicle/system bus
> - A MEMS IMU in the ADIS1647x/ADIS1649x class: 3.3 V, ≤150 mA peak, SPI interface
> - Galvanic isolation required between the system bus and the IMU domain
>
> Send me the prior cycle's open items and I'll reconcile them against this design.

---

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU rail | 3.3 V ±2% | IMU VDD spec |
| Output noise (10 Hz–100 kHz) | < 10 µV RMS | Keeps supply-induced gyro and accel noise below the sensor noise floor |
| Ripple at IMU pins | < 50 µV p-p | Suppresses DC-DC switching spurs that can alias into the sensor's internal sampling |
| PSRR at the switching frequency | > 50 dB from the LDO, > 90 dB total | Combines LDO PSRR with passive filtering |
| Isolation | ≥ 2.5 kV RMS (functional or basic) | Breaks ground loops and blocks motor/actuator transients |
| Load-step deviation | < 10 mV on a 0→150 mA step | IMU burst reads and heater transients |

---

## 2. Architecture Overview

```
 28V BUS ──►[Input protection]──►[Isolated DC-DC]──►[Pi filter]──►[TPS7A4700]──►[Local decoupling]──► IMU VDD
            TVS + reverse     SN6505B + 1:1.3    ferrite +      3.3 V out      IMU pins
            polarity + CMC    transformer        LC, f_c≈2kHz   ~1.2 V hdrm
            (non-iso side)    → ~4.5–5 V iso     
                                                                   │
            ────────── ISOLATION BARRIER ──────────                │
                                                                   ▼
 MCU SPI ◄──────────────[ISO7741 digital isolator]◄─────── IMU SPI / DRDY / RST
            (VCC1 from MCU rail; VCC2 from a separate quiet LDO or from the
             IMU 3.3 V rail through a ferrite)
```

### Stage-by-stage rationale

**Stage 1: Input protection (non-isolated side)**
- SMBJ33A TVS for load-dump and transient clamping
- Ideal-diode or P-FET reverse-polarity protection
- Common-mode choke, e.g., Würth 744272102
- A pre-buck (e.g., LMR36015) from 28 V down to 5 V to feed the isolated converter

**Stage 2: Isolated DC-DC**
- SN6505B push-pull transformer driver with Würth 750315240 or a similar 1:1.3 transformer
- Rectify with Schottky diodes to an unregulated ~5 V on the isolated side
- Use spread-spectrum mode (SN6505B) to smear switching spurs; this reduces peak ripple energy at any single frequency
- Alternative: UCC12050, if board area dominates. It needs heavier post-filtering because it radiates more EMI.

**Stage 3: Pi filter (isolated side)**
- Ferrite bead: BLM31KN601SN1, 600 Ω at 100 MHz, ≥1 A
- 10 µH shielded inductor + 2 × 22 µF X7R
- Corner frequency ≈ 7–8 kHz
- Add 1 Ω series damping, or an electrolytic/polymer cap with ESR, to avoid an LC resonance peak
- This attenuates the ~420 kHz SN6505 ripple by about 40 dB before it reaches the LDO

**Stage 4: TPS7A4700 LDO**
- Set to 3.3 V with about 1.2 V of headroom
- Datasheet dropout is roughly 307 mV at 1 A and much less at 150 mA
- Keep ≥1 V of headroom anyway: PSRR above about 10 kHz falls off sharply near dropout

**Stage 5: IMU local decoupling**
- 10 µF + 1 µF + 100 nF X7R, all within 2 mm of the IMU VDD pins
- Follow the IMU datasheet's pin-level recommendations

---

## 3. TPS7A4700 Configuration

### Output voltage via ANY-OUT (no external resistors)

V_OUT = 1.4 V (reference) + the sum of the ANY-OUT pins grounded:

| Pin | Weight | 3.3 V | 5.0 V (alt.) |
|---|---|---|---|
| 6P4V | 6.4 V | open | open |
| 3P2V | 3.2 V | open | **GND** |
| 1P6V | 1.6 V | **GND** | open |
| 0P8V | 0.8 V | open | open |
| 0P4V | 0.4 V | open | **GND** |
| 0P2V | 0.2 V | **GND** | open |
| 0P1V | 0.1 V | **GND** | open |
| **Result** | | **1.4 + 1.6 + 0.2 + 0.1 = 3.3 V** | **1.4 + 3.2 + 0.4 = 5.0 V** |

Using ANY-OUT instead of a feedback divider keeps resistor thermal noise out of the loop, which preserves the full ~4 µV RMS performance.

### Pin and component table

| Pin | Component | Notes |
|---|---|---|
| IN | 10 µF X7R 25 V + 1 µF + 100 nF | Place close to the pin; add a 0.1–0.5 Ω ESR element if the source impedance is inductive |
| OUT | 10 µF X7R (≥ the datasheet minimum) + 1 µF | The ceramic must stay ≥ 2.5 µF effective after DC-bias derating, so use a 1206 / 16 V part |
| NR/SS | **10 µF X7R**, low leakage | Sets the reference noise-filter corner very low, which minimizes 10 Hz–1 kHz noise. This is the band that matters most for IMU bias stability. Soft-start time becomes long (tens of ms), which is acceptable here. |
| FB / bypass | 10 nF C_BYP from OUT to FB, if using the datasheet's feed-forward option | Improves PSRR and noise in the mid band |
| EN | Driven by the isolated-side UVLO (resistor divider from VIN), or tied to IN | Use a divider to set enable around 4.2 V so the LDO never runs in dropout |
| PG | 100 kΩ pull-up to V_OUT | Route through the isolator to the MCU to gate IMU reset release |
| Thermal pad | Solid pour to the isolated GND with ≥ 9 vias | |

### Thermal check

P_D = (4.5 V − 3.3 V) × 0.15 A ≈ 0.18 W.

With θ_JA ≈ 35 °C/W (VQFN-20), the temperature rise is about 6 °C. This is negligible and limits thermal drift coupling into the IMU. Keep the LDO at least 15 mm from the IMU so its heat doesn't create a thermal gradient across the sensor.

---

## 4. Noise and PSRR Budget

| Frequency band | Source | Pi-filter attenuation | LDO PSRR | Residual at IMU |
|---|---|---|---|---|
| 100 Hz | Upstream ripple (~20 mV) | ~0 dB | ~80 dB | ~2 µV |
| 10 kHz | Converter subharmonics (~10 mV) | ~10 dB | ~70 dB | ~1 µV |
| 420 kHz | SN6505 ripple (~50 mV p-p) | ~40 dB | ~50 dB | ~0.16 µV |
| 1–10 MHz | Edge harmonics | ~60 dB | ~30–40 dB | <0.1 µV |
| 10 Hz–100 kHz | LDO intrinsic noise | — | — | ~4.2 µV RMS |

**Result:** total supply noise is about 5 µV RMS, which meets the < 10 µV target with about 2× margin.

---

## 5. Isolation and Grounding Strategy

1. **Two ground domains.**
   - GND_SYS: non-isolated side
   - GND_IMU: isolated side
   - Optionally stitch them with a 2.2 nF Y-cap rated for the isolation voltage. This gives EMI common-mode return without creating a DC loop.
2. **Star point.** Connect the IMU ground to GND_IMU at a single point near the TPS7A4700 GND/thermal pad.
3. **Creepage.** Keep ≥ 4 mm across the barrier and a slot under the transformer if required.
4. **Digital isolation.**
   - Use an ISO7741 (4-channel, 3 forward / 1 reverse) for SCLK, MOSI, CS → and ← MISO
   - Use a second ISO7710 or a spare channel for DRDY and RST
5. **No signal returns across the barrier** except through the isolator.
6. **Clocks.** If the IMU uses an external sync/clock, pass it through the isolator. Never reference it to GND_SYS.

---

## 6. Layout Rules

- Keep the LDO output loop (OUT cap → IMU → GND) on one layer with an unbroken GND_IMU plane underneath.
- Separate the switching node and transformer from the LDO and IMU by ≥ 20 mm. Shield with a ground pour if needed.
- Guard-ring the NR/SS trace. It is high impedance, so keep it short and away from switching nets.
- Use Kelvin-style routing for the OUT sense if the IMU is remote. Otherwise place the LDO within 30 mm.
- Do not place IMU mounting screws in the path of switching current returns.
- Mechanically isolate the IMU from the transformer and inductors, which can be magnetostrictive or acoustic noise sources.

---

## 7. Sequencing and Protection

- The LDO's PG output, routed through the isolator, gates IMU reset release in MCU firmware: wait for PG plus the IMU startup time (≥ 250 ms for the ADIS family).
- The long NR/SS soft-start, set by the 10 µF cap, provides inherent inrush limiting.
- The TPS7A4700's internal current limit and thermal shutdown cover IMU fault cases.
- Optionally add a 3.6 V TVS (or 5.6 V Zener + PTC) on the IMU rail for overvoltage protection.

---

## 8. Bill of Materials (Key Items)

| Ref | Part | Function |
|---|---|---|
| U1 | LMR36015 | 28 V → 5 V pre-buck |
| U2 | SN6505B | Isolated push-pull driver |
| T1 | Würth 750315240 | Isolation transformer |
| D1, D2 | PMEG4010 | Isolated-side rectifiers |
| FB1 / L1 | BLM31KN601 / 10 µH shielded | Pi filter |
| U3 | **TPS7A4700RGW** | Ultra-low-noise LDO |
| C_NR | 10 µF X7R 16 V 0805 | Noise reduction |
| U4 | ISO7741 | SPI isolation |
| D_TVS | SMBJ33A | Input transient protection |

---

## 9. Verification Plan (Cycle 14 Exit Criteria)

1. **Output noise.** Measure with an LNA (≥ 40 dB gain) and a spectrum analyzer from 10 Hz to 100 kHz. Pass: < 10 µV RMS.
2. **Ripple.** Use a 1:1 coax probe with a 50 Ω termination, 20 MHz BW. Pass: < 50 µV p-p at the IMU pins.
3. **PSRR.** Inject a 100 mV sine at the LDO input with a Bode 100 or equivalent, sweeping 10 Hz–10 MHz. Compare against the budget in Section 4.
4. **Load step.** Step 0→150 mA with 1 µs edges. Pass: < 10 mV deviation, no ringing beyond 2 cycles.
5. **IMU-level test.**
   - Capture an Allan variance with the system powered from (a) a lab linear supply and (b) this supply.
   - Pass: the bias instability and ARW/VRW differences are within measurement uncertainty, i.e., < 5%.
6. **Isolation.** Hipot at the rated voltage for 60 s.
7. **Conducted EMI.** Run CISPR 25 or the program standard on the 28 V input.
8. **Thermal.** Soak at −40 °C and +85 °C. Confirm V_OUT drift < 0.5% and no start-up failures with the long soft-start at cold.

---

## 10. Open Items and Risks to Carry Forward

- Confirm the actual IMU part and its supply sensitivity. Some tactical IMUs specify supply-rejection limits in mg/V or °/hr/V, which may tighten the noise target.
- Decide between the SN6505B and UCC12050, trading EMI performance against board area.
- Verify the DC-bias derating of the output and NR caps at temperature.
- Evaluate whether the IMU's digital I/O needs a separate VDDIO rail. If so, add a second TPS7A4700, or a TPS7A20 for lower cost.

If you share the Cycle 13 outcomes (measured noise, failures, IMU model, bus voltage), I can turn this into a delta document with specific changes.