# Cycle 15: Low-Noise IMU Power Architecture Using the TPS7A4700

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail (VDD) | 3.30 V ±1% | Typical MEMS IMU supply (e.g., ADIS16470, ICM-42688-P, BMI088) |
| IMU digital rail (VDDIO) | 3.30 V or 1.8 V, isolated from VDD | Prevents SPI/I²C edge currents from coupling into the analog sense path |
| Output noise (10 Hz–100 kHz) | < 10 µV RMS at IMU pin | Keeps supply-induced bias instability below the sensor's intrinsic noise floor |
| Ripple at switcher frequency | < 50 µV p-p at IMU pin | Avoids aliasing into the IMU's internal ADC/decimation filters |
| Load current | 10–60 mA typical, 150 mA budgeted | Covers IMU, plus margin for a heater or second sensor |
| Input source | 9–28 V vehicle/system bus | Typical UAV, robotics, or avionics bus |

## 2. Architecture Overview

```
 VBUS (9–28 V)
    │
 [TVS + reverse-polarity FET + input EMI filter]
    │
 [Stage 1: Synchronous Buck Pre-Regulator] ──► 4.2 V, fsw ≈ 2.2 MHz (spread spectrum OFF)
    │
 [Stage 2: CLC π-filter]  (10 µF – 600 Ω@100MHz ferrite – 10 µF + 22 µF polymer)
    │
 [Stage 3: TPS7A4700 LDO]  ──► 3.30 V (ANY-OUT configured)
    │
    ├──► [Ferrite + 10 µF/100 nF local] ──► IMU VDD (analog)
    │
    └──► [Ferrite / 10 Ω + 4.7 µF/100 nF] ──► IMU VDDIO (if 3.3 V I/O)
                    or
         [Separate small LDO, e.g. TPS7A20 at 1.8 V] ──► VDDIO (if 1.8 V I/O)
```

Each stage has a specific job:

- **Buck:** handles efficiency and the large input-to-output voltage drop.
- **π-filter:** attenuates switching ripple above the LDO's PSRR corner.
- **TPS7A4700:** sets the noise floor and supplies low-frequency PSRR.
- **Post-filters:** isolate the IMU's analog and digital domains from each other.

## 3. Stage 3 Core: TPS7A4700 Configuration

### 3.1 Key Device Characteristics (check against the current datasheet)
- VIN range: 3 V to 36 V. Output: 1.4 V to 20.5 V. Maximum load: 1 A.
- Output noise: about 4.17 µV RMS (10 Hz–100 kHz).
- PSRR: about 82 dB at 100 Hz, and roughly 55 dB or better out to the MHz range.
- Dropout: about 307 mV at 1 A, and much lower at tens of mA.
- Internal reference: 1.4 V. The ANY-OUT pins let you set the output without external resistors.
- Package: 5 × 5 mm VQFN-20 (RGW) with an exposed thermal pad.

### 3.2 Setting 3.3 V with ANY-OUT (no feedback resistors)

The output voltage follows:

V_OUT = 1.4 V + Σ(grounded ANY-OUT pin weights)

For 3.3 V:

**3.3 V = 1.4 V + 1.6 V (1P6V) + 0.2 V (200mV) + 0.1 V (100mV)**

| Pin | Connection |
|---|---|
| 1P6V | GND |
| 200mV | GND |
| 100mV | GND |
| 6P4V2, 6P4V1, 3P2V, 800mV, 400mV | Leave floating |

Using ANY-OUT has two advantages. It removes resistor-divider thermal (Johnson) noise, and it removes the noise gain that external resistors would add. Accuracy is set by the internal matched network, about ±1% overall.

### 3.3 Pin-Level Component Selection

| Pin / Node | Component | Value | Notes |
|---|---|---|---|
| IN | C_IN | 10 µF X7R 25 V + 100 nF 0402 | Placed within 2 mm of the pins |
| OUT | C_OUT | 10 µF X7R 10 V (≥4.7 µF effective after DC bias) + 100 nF | Low-ESR ceramic meets the stability requirement |
| NR/SS | C_NR | 10 µF X7R | Filters reference noise to near the device floor. Soft-start becomes about 100 ms or more; confirm this fits your IMU power-on sequencing |
| FB / OUT feedforward | C_FF | 10 nF | Lowers noise and improves transient response. Verify phase margin |
| EN | RC from 4.2 V | 100 kΩ / 1 µF, or MCU GPIO | Lets you sequence the IMU after the MCU I/O is stable |
| PG | Pull-up | 100 kΩ to 3.3 V (VDDIO domain) | Routes to an MCU GPIO to gate IMU initialization |
| Thermal pad | GND | 9-via array | Connects to the quiet analog ground plane |

### 3.4 Headroom and Thermal Check

- **Headroom:** V_IN − V_OUT = 4.2 − 3.3 = 0.9 V.
  - The required dropout at 150 mA is under about 100 mV, so the margin is ample.
  - Keeping roughly 0.9 V of headroom preserves PSRR in the 100 kHz–2 MHz band. PSRR collapses as the device approaches dropout.
- **Dissipation:** P_D = 0.9 V × 0.15 A = 135 mW (worst case). With θ_JA ≈ 35 °C/W, the temperature rise is under 5 °C, which is negligible.

## 4. Stage 1 and 2: Pre-Regulation and Ripple Filtering

### 4.1 Buck Selection
- **Recommended:** TPS62913 or TPS62912. These are TI low-noise bucks with an integrated ferrite-bead compensation option and about 2.2 MHz switching.
- **Alternative:** LMR36015, if the input bus can exceed 17 V. Set fsw to 2.1 MHz.
- **Disable spread spectrum.** A fixed frequency gives a single, predictable ripple tone that the π-filter can target. Spread spectrum smears energy into the IMU's sensitive band.
- **Set the frequency well above the IMU's internal ADC rates.** Keep fsw and its harmonics clear of the IMU's ODR × decimation aliasing points. Above 2 MHz is typically safe.

### 4.2 π-Filter Design
- **Topology:** 10 µF ceramic → ferrite bead (e.g., BLM18KG601, 600 Ω at 100 MHz, rated at least 1 A) → 10 µF ceramic plus 22 µF polymer.
- **Damping:** the polymer capacitor's ESR (roughly 20–50 mΩ) damps the bead/capacitor resonance. Without damping, the resonance near 50–200 kHz could peak and amplify ripple.
- **Target attenuation:** at least 40 dB at 2.2 MHz.

### 4.3 Ripple Cascade Estimate

| Point | Ripple at 2.2 MHz |
|---|---|
| Buck output | ~10 mV p-p |
| After π-filter (−40 dB) | ~100 µV p-p |
| After TPS7A4700 (PSRR ≈ 45 dB at 2 MHz, conservative) | ~0.6 µV p-p |
| After post-filter at IMU | < 0.5 µV p-p (well below the target) |

## 5. IMU Domain Isolation (Stage 4)

### 5.1 Analog VDD Branch
- **Path:** LDO output → ferrite bead (BLM15AX102, about 1 kΩ at 100 MHz, low DCR) → 10 µF + 100 nF + 10 nF at the IMU VDD pin.
- **Purpose:** this branch filters high-frequency digital noise that couples backward from the IMU's own internal circuitry. It also isolates the IMU from other loads on the rail.

### 5.2 Digital VDDIO Branch
- **Option A (3.3 V I/O):** use a separate ferrite, or 10 Ω plus 4.7 µF, from the TPS7A4700 output. The resistor variant gives a 3.4 kHz RC corner. That is acceptable because VDDIO draws only about 1 mA, so the drop is about 10 mV.
- **Option B (1.8 V I/O):** use a dedicated TPS7A20 (1.8 V) fed from the 4.2 V intermediate rail. This fully decouples SPI switching current from the analog regulator's control loop.
- **Choose Option B** if the IMU shares SPI with high-speed peripherals or uses SCLK at or above 10 MHz.

### 5.3 Grounding Strategy
- Use one continuous ground plane on layer 2. Do not split it. Splits create return-path discontinuities that radiate more than they isolate.
- Partition the board by placement:
  - Keep the buck and its hot loop in one corner.
  - Place the LDO between the buck and the IMU.
  - Keep the IMU farthest from the switcher, with at least 20 mm separation if possible.
- Keep the buck's input/switch-node loop small, and keep its return currents in a local copper pour that is stitched to the main plane near the buck.
- Route SPI traces over unbroken ground. Add 22–33 Ω series termination at the MCU end to slow edge rates.

## 6. Noise Budget at the IMU VDD Pin (10 Hz–100 kHz)

| Contributor | Estimated RMS | Notes |
|---|---|---|
| TPS7A4700 intrinsic (C_NR = 10 µF, C_FF = 10 nF) | ~4.2 µV | Datasheet floor |
| Residual buck ripple (in-band component) | < 0.5 µV | After the π-filter and PSRR |
| Ferrite/resistor thermal noise | < 0.1 µV | Negligible |
| Load-transient-induced noise (IMU internal sampling) | ~1–2 µV | Absorbed by local capacitance |
| **RSS total** | **≈ 4.7 µV RMS** | Meets the < 10 µV target with about 2× margin |

**Sensor impact:** the IMU's own PSRR, typically 40–60 dB for the internal analog front end, further attenuates this. Supply-induced gyro bias noise therefore drops below the sensor's intrinsic angle random walk contribution.

## 7. Sequencing and Protection

1. The 4.2 V buck starts as soon as VBUS is present.
2. The MCU boots from its own rail. That rail can share the buck, but needs a separate LDO.
3. The MCU asserts EN on the TPS7A4700, or the RC delay releases it.
4. The PG signal goes high once 3.3 V is in regulation. Expect this roughly 100 ms or more after EN, because C_NR also sets soft-start.
5. If you use Option B, the VDDIO LDO enables at or after VDD. Respect the IMU's VDD/VDDIO ordering spec; most allow either order, but verify for your part.
6. Only then does firmware begin IMU SPI initialization.

Protection measures:
- **Input:** SMBJ33A TVS and an ideal-diode or PFET reverse-polarity stage.
- **LDO:** the TPS7A4700 has internal current limit and thermal shutdown, so no added parts are needed.
- **Optional:** a 3.6 V Zener or crowbar on the IMU VDD, if the IMU's absolute maximum (often 4.0 V) is tight relative to fault scenarios.

## 8. Verification Plan for Cycle 15

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise density | LNA (e.g., 40–60 dB gain) into an FFT analyzer, 10 Hz–100 kHz | < 10 µV RMS integrated; no spurs > 1 µV |
| Ripple at fsw | 50 Ω coax pickoff with DC block, spectrum analyzer | < 50 µV p-p at the IMU pin |
| PSRR | Inject signal on the 4.2 V rail via line injector (e.g., Picotest J2120A) | ≥ 50 dB at 2.2 MHz, at the system level |
| Load transient | 1 → 60 mA step, 1 µs edge | < 5 mV deviation, no ringing past 3 cycles |
| Stability | Bode via injection at the FB node | Phase margin > 45° |
| IMU-level validation | Allan deviation, buck enabled vs. powered from a bench linear supply | No measurable change in bias instability or ARW |
| Thermal | IR camera at maximum load and 85 °C ambient | T_J < 125 °C |

## 9. Cycle 15 Deltas and Open Risks

**Changes in this cycle:**
- Moved from external feedback resistors to ANY-OUT, removing divider noise.
- Raised C_NR to 10 µF for the lowest reference noise.
- Added a dedicated VDDIO LDO option.
- Fixed the buck frequency and disabled spread spectrum.

**Open risks to carry into Cycle 16:**
1. **Soft-start duration:** C_NR = 10 µF extends start-up. Confirm this against the system boot-time requirement.
2. **Ceramic DC-bias derating:** verify that the 10 µF/10 V output capacitors keep at least 4.7 µF effective at 3.3 V, using the vendor's derating curves.
3. **Microphonics:** MLCCs near a vibrating IMU mount can generate piezoelectric noise. Consider C0G, polymer, or tantalum parts for the IMU-local bulk capacitance if vibration testing shows supply-correlated spurs.
4. **Buck harmonic coupling:** confirm that the IMU's internal sampling clock does not beat against 2.2 MHz harmonics. Shift fsw if spurs appear in the IMU output spectrum.

---

**Note:** confirm all TPS7A4700 parameters against the current TI datasheet (SBVS204 family) before releasing the design. Typical values vary with conditions such as C_NR, C_FF, V_IN − V_OUT, and load. If you share the specific IMU part number and the prior-cycle constraints