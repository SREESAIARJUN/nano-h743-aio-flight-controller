# Low-Noise IMU Power Supply Architecture, Cycle 25

**Core regulator:** Texas Instruments TPS7A4700 (36 V, 1 A, ultra-low-noise positive LDO)
**Purpose:** Deliver a clean, isolated analog rail to the IMU and decouple it from switching, digital, and actuator noise on the system bus.

---

## 1. Design Requirements (Cycle 25 Baseline)

| Parameter | Target | Rationale |
|---|---|---|
| IMU supply | 3.3 V ±2% | Typical for MEMS IMUs such as ADIS164xx, BMI088, and ICM-4xxxx |
| IMU load current | ≤ 100 mA (design to 250 mA) | Peak plus heater and start-up margin |
| Output noise (10 Hz–100 kHz) | < 10 µVrms | Keeps supply-induced bias instability below the sensor noise floor |
| Ripple at buck f_sw | < 10 µVpp at the IMU pin | Prevents aliasing into gyro/accel bandwidth |
| Input bus | 9–32 V (12/24/28 V nominal) | Vehicle or avionics bus |
| Transient immunity | Load dumps, ISO 7637 / DO-160-style surges | Bus robustness |

**Key TPS7A4700 specs to verify against the current datasheet revision:**
- V_IN 3–36 V and V_OUT 1.4–20.5 V
- Noise ≈ 4.17 µVrms (10 Hz–100 kHz)
- PSRR ≈ 82 dB at 100 Hz, remaining high (>55 dB) into the hundreds of kHz
- Dropout ≈ 307 mV at 1 A
- ANY-OUT™ resistor-free output programming
- VQFN-20 (5 × 5 mm) package with a thermal pad

---

## 2. Top-Level Architecture

```
 28V BUS
   │
 [Stage 0] Protection: reverse-polarity ideal diode (LM74700-Q1) + TVS (SMBJ33A) + fuse
   │
 [Stage 1] Pre-regulator: sync buck, 28V → 5.0V
   │        (e.g., LMR33630 / TPS62912 @ ≥2 MHz, spread-spectrum OFF or synchronized)
   │
 [Stage 2] Passive π-filter: C – L(ferrite + 4.7µH) – C, with damping
   │
 [Stage 3] TPS7A4700, 5.0V → 3.3V  ◄── ultra-low-noise post-regulator
   │
 [Stage 4] Local split at the IMU:
   ├── FB + 10µF ──► IMU VDD (analog core)
   └── FB + 1µF  ──► IMU VDDIO (digital I/O)   [or a 2nd LDO, see §6]
   │
 IMU (on an isolated analog ground island)
```

**Why a two-stage design (buck followed by LDO)?** Running the LDO directly from 28 V to 3.3 V would dissipate about 2.5 W at 100 mA. The buck handles efficiency. The LDO and π-filter handle noise.

---

## 3. Stage-by-Stage Design

### Stage 1: Pre-regulator (Buck)
- **Output:** 5.0 V. This gives 1.7 V of headroom over 3.3 V.
- **Headroom rule:** keep the LDO's V_IN – V_OUT ≥ 1 V. TPS7A47 PSRR at high frequency degrades sharply near dropout.
- **Switching frequency:** ≥ 2 MHz. This places ripple far above the IMU bandwidth (typically < 500 Hz), and the π-filter attenuates it more easily.
- **Spread spectrum:** disable it, or sync the buck to a known clock. Spread spectrum smears energy toward lower frequencies, where the filter is less effective.

### Stage 2: π-Filter (Buck → LDO)
- **Topology:** C1 (10 µF X7R) → ferrite (e.g., BLM18KG, 600 Ω at 100 MHz) → L = 4.7 µH shielded → C2 (22 µF X7R) ‖ 47 µF polymer.
- **Corner frequency:** f_c = 1/(2π√(4.7 µH × 22 µF)) ≈ **15.6 kHz**.
- **Attenuation at 2.1 MHz (ideal):** (2.1 M / 15.6 k)² ≈ 85 dB. Plan for about 50–60 dB in practice, because layout parasitics and inductor SRF limit real performance.
- **Damping:** the polymer capacitor's ESR (about 20–50 mΩ), or a 0.5 Ω series R plus 47 µF branch, keeps filter Q below about 1. Without damping, a resonant peak at f_c can amplify buck load-step energy.

### Stage 3: TPS7A4700 Configuration

**Output programming via ANY-OUT:** 3.3 V = 1.4 V (reference) + 1.6 V + 0.2 V + 0.1 V.

| Pin | Connection |
|---|---|
| 1P6V | GND |
| 200mV | GND |
| 100mV | GND |
| 6P4V2, 3P2V, 800mV, 400mV | Float |

Confirm the pin-strap convention against the datasheet table. Alternatively, use an external divider with 0.1% resistors to set V_OUT.

**Passive components:**

| Pin / Function | Component | Notes |
|---|---|---|
| IN | 10 µF X7R + 0.1 µF | Placed at the pin |
| OUT | 10 µF X7R (≥ 10 µF required) + 0.1 µF | Low-ESR ceramic. Derate for DC bias: a 25 V-rated 1206 or 1210 part retains capacitance at 3.3 V |
| NR (noise reduction / soft-start) | 1 µF X7R (C0G preferred if available) | A larger value lowers 1/f noise but lengthens soft-start (~tens of ms). Check that the IMU power-on reset tolerates the ramp |
| FB bypass (feed-forward) | 10 nF C0G | Improves noise and PSRR in the 1–100 kHz region |
| EN | RC from 5 V rail, or MCU GPIO | Sequencing, see §5 |
| Thermal pad | Solid GND pour with 9+ vias | |

**Thermal check:** P_D = (5.0 – 3.3) × 0.25 A = 0.43 W worst case. With θ_JA ≈ 35 °C/W, the rise is about 15 °C, which is negligible. At 100 mA nominal the LDO dissipates only about 0.17 W. This matters: placing the LDO **away from the IMU** prevents thermal gradients from coupling into gyro bias.

### Stage 4: Point-of-Load Isolation at the IMU
- **Ferrite bead per rail:** low DCR (< 0.1 Ω), sized so the IR drop stays under 10 mV.
- **Local decoupling:** 10 µF + 0.1 µF + 10 nF stacked capacitors at each IMU supply pin, in the order the IMU datasheet specifies.
- **Separate analog and digital supplies:** VDD (analog) and VDDIO (digital) get separate ferrites, so SPI edge currents do not modulate the analog core.

---

## 4. Noise Budget (Estimate)

| Source | At LDO output | At IMU pin |
|---|---|---|
| TPS7A47 intrinsic noise (10 Hz–100 kHz) | ~4–5 µVrms | ~4–5 µVrms |
| Buck ripple (10 mVpp at 2.1 MHz) | after ~55 dB π-filter + ~40 dB LDO PSRR → < 0.5 µVpp | < 0.2 µVpp after POL ferrite/capacitors |
| Buck harmonics (> 10 MHz) | Limited by layout coupling, not PSRR | Controlled by layout (§7) |
| Bus transients | Absorbed by Stage 0/1 | n/a |
| **Total target** | **< 10 µVrms** | ✅ Meets requirement |

The next check is to compare this budget against the IMU's specified PSRR or supply sensitivity (°/hr/V or mg/V) to confirm that supply noise sits below the sensor's angle random walk (ARW) and velocity random walk (VRW).

---

## 5. Sequencing and Protection
1. The buck starts from the bus.
2. The TPS7A47 EN pin is tied to the 5 V rail through an RC (about 5 ms delay), or driven by the MCU. This ensures the LDO input is settled first.
3. The NR capacitor gives a controlled soft-start, which limits inrush into the IMU decoupling capacitors.
4. **Optional:** add a TPS3890 supervisor monitoring 3.3 V and holding the IMU or MCU in reset until the rail is within tolerance.
5. **Back-feed protection:** if the IMU's I/O can be driven while it is unpowered (MCU powered first), add series resistors (22–47 Ω) on SPI lines, or sequence the MCU after the IMU.

---

## 6. Isolation Options by Tier

| Tier | Method | When to use |
|---|---|---|
| **A (baseline)** | Single TPS7A4700 feeding the IMU analog and digital rails through split ferrites | Most designs |
| **B** | Dedicated TPS7A4700 for analog VDD, plus a small LDO (e.g., TPS7A20) for VDDIO | Noisy digital bus or long SPI traces |
| **C** | Galvanically isolated DC/DC (e.g., SN6505B + transformer, or UCC12050), then the π-filter, then TPS7A4700, with ISO7741 digital isolators on SPI | High-current motor/actuator ground currents, or long cable runs where ground potential differs |

Tier C adds roughly 100–200 mW of loss and a 400 kHz–1 MHz switching source. If you adopt it, retain Stage 2 and the LDO after the isolator.

---

## 7. PCB Layout Rules (Critical)
- **Ground:** use a single solid ground reference plane. Do not split it under signals; define an analog *zone* rather than a slotted plane. Star-connect the IMU zone to system ground at the LDO output capacitor ground.
- **Separation:** place the buck and its switch-node copper at least 25 mm from the IMU. Keep the switch-node area minimal, with no traces routed beneath it.
- **LDO return loops:** keep OUT, NR, and FB-bypass capacitor loops very tight, since the NR and FB nodes are high-impedance and noise-sensitive. Guard-ring the NR trace with GND.
- **Routing to the IMU:** run a wide trace or local pour from the LDO output to the IMU. Do not route it parallel to SPI clock lines or PWM lines.
- **IMU mounting:** do not place the LDO or other heat sources under or adjacent to the IMU. Keep the IMU away from board-flex regions. Follow vendor keep-out guidance, avoiding copper or vias directly under some MEMS packages.
- **Shielding (optional):** a stitched-via fence or shield can around the buck if radiated coupling appears in testing.

---

## 8. Reference BOM (Core Items)

| Ref | Part | Qty |
|---|---|---|
| U1 | TPS7A4700RGWR | 1 |
| U2 | LMR33630 (or equivalent buck) | 1 |
| U3 | LM74700-Q1 ideal diode controller + MOSFET | 1 |
| D1 | SMBJ33A TVS | 1 |
| L1 | 4.7 µH shielded, I_sat > 1 A | 1 |
| FB1–FB3 | BLM18KG601SN1 (or similar low-DCR) | 3 |
| C_NR | 1 µF X7R 0603 | 1 |
| C_BYP | 10 nF C0G 0402 | 1 |
| C_OUT | 10 µF X7R 25 V 1206 | 2 |
| C_POL | 47 µF polymer (π-filter damping) | 1 |

---

## 9. Cycle 25 Verification Plan
1. **Output noise:** measure with an LNA (e.g., 40–60 dB gain) and a spectrum analyzer, 10 Hz–100 kHz. Pass criterion: < 10 µVrms.
2. **Ripple at the IMU pin:** use a 50 Ω coax pigtail probe with a short ground (no long ground clip). Pass criterion: < 10 µVpp at f_sw and its harmonics.
3. **PSRR sweep:** inject onto the 5 V rail with a line injector (e.g., Picotest J2120A) and a network analyzer, 10 Hz–10 MHz.
4. **Load-transient response:** step 10 → 100 mA and check overshoot and settling.
5. **System-level test:** collect a static IMU Allan-variance run with the buck operating versus a battery-powered linear reference. ARW and bias instability should agree within measurement uncertainty.
6. **Thermal soak:** run –40 to +85 °C and confirm the LDO output drift is below the IMU supply-sensit