# Cycle 21: Low-Noise IMU Power Architecture Using the TPS7A4700

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail noise | < 10 µV RMS (10 Hz–100 kHz) | Keeps supply noise below gyro/accel ADC noise floor |
| Switching ripple at IMU pin | < 100 µV p-p | Prevents aliasing into sensor bandwidth |
| PSRR at the preregulator switching frequency | > 60 dB combined (LDO + filter) | Needed to attenuate buck ripple to the target |
| Load transient deviation | < 5 mV | Handles IMU burst current during sampling |
| Isolation | Separate analog and digital domains | Keeps digital and comms noise off sensor references |
| Vibration sensitivity | Minimized | Reduces piezoelectric MLCC noise under platform vibration |

## 2. Top-Level Architecture

```
 VBUS (9–28 V)
    │
 [Input protection: TVS + reverse-polarity FET + π-filter]
    │
 [Stage 1: Synchronous buck, e.g., LMR33630, fsw ≈ 2.1 MHz]  → 4.0 V
    │
 [Stage 2: Passive LC post-filter (FB + 10 µF, damped)]
    │
    ├──► [Stage 3A: TPS7A4700 #1] → 3.3 V  IMU_AVDD   (analog/sensor core)
    │
    └──► [Stage 3B: TPS7A4700 #2, or a lower-cost LDO] → 3.3 V  IMU_VDDIO (digital I/O)
                                   │
                              [Ferrite + 100 nF at IMU pin]
```

**Key design choices:**

- **Separate LDOs for the analog and I/O rails.** SPI edge currents on VDDIO cannot couple into AVDD through a shared regulator's output impedance.
- **Preregulator with about 0.7 V of headroom.** This keeps the LDO above dropout (about 307 mV at 1 A, much lower at IMU load currents) while limiting dissipation.
- **A switching frequency above 2 MHz.** This pushes switching ripple far above the IMU bandwidth, where the LC filter is most effective.

## 3. TPS7A4700 Configuration

### 3.1 Key Device Characteristics

Confirm these values against the current datasheet revision:

- Input range: 3 V to 36 V
- Output current: 1 A
- Output noise: about 4.17 µV RMS (10 Hz–100 kHz)
- PSRR: high at low frequency (about 80 dB range), degrading at higher frequencies
- Output setting: ANY-OUT™ pins, from 1.4 V up to 20.5 V
- Package: 5 mm × 5 mm VQFN (RGW) with thermal pad

### 3.2 Output Voltage Programming (ANY-OUT)

The output voltage is 1.4 V plus the sum of the ANY-OUT pins tied to GND. Leave unused pins floating.

| Rail | Pins to GND | Result |
|---|---|---|
| 3.3 V | 1P6V + 200mV + 100mV | 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V** |
| 5.0 V (if the IMU needs it) | 3P2V + 400mV | 1.4 + 3.2 + 0.4 = **5.0 V** |
| 1.8 V | 400mV | 1.4 + 0.4 = **1.8 V** |

Use ANY-OUT rather than an external resistor divider. Divider resistors add thermal noise and gain noise to the output.

### 3.3 Passive Components

| Pin | Component | Value / Type | Notes |
|---|---|---|---|
| IN | C_IN | 10 µF X7R 50 V + 100 nF | Place close to the pin. Size for DC-bias derating. |
| OUT | C_OUT | 10 µF X7R + 22 µF polymer tantalum | Polymer reduces the piezo effect and adds ESR damping. |
| NR | C_NR | 1 µF–10 µF, film or C0G/polymer preferred | Larger values give lower noise but slower start-up (soft start). |
| SENSE | Kelvin trace | To the IMU AVDD pin or LDO output cap | Remote sense compensates for trace IR drop. |
| EN | RC from the sequencer | 100 kΩ / 100 nF | Provides controlled sequencing. |

**Vibration note specific to IMUs.** Class II MLCCs (X7R/X5R) are piezoelectric. On a vibrating platform they generate voltage noise at the vibration frequencies, which then appears directly in the sensor bands. To counter this:

- Use C0G, film, or polymer capacitors on NR and on the final AVDD node.
- Where X7R is unavoidable, use small case sizes such as 0402 or 0603.
- Consider soft-termination parts.

## 4. Preregulator and Post-Filter Design

**Buck stage, 4.0 V output:**

- A spread-spectrum option can be considered, but it is usually disabled for IMU systems. Choose a fixed frequency so that spurs stay in a known, filterable location.
- Synchronize the buck to the system clock if possible. This prevents beat frequencies with the IMU ADC.

**LC post-filter:**

- Use a ferrite bead such as BLM18PG (about 600 Ω at 100 MHz), or a 2.2 µH shielded inductor plus 10 µF.
- The corner frequency is roughly 34 kHz, giving about 60 dB of attenuation at 2.1 MHz.
- Add a damping network (1 Ω + 22 µF in series across the filter capacitor) to suppress the LC resonance peak.

**Combined rejection at 2.1 MHz:**

- LC filter: about 60 dB
- LDO PSRR at that frequency: about 40 dB
- Total: about 100 dB

With 20 mV of buck ripple, this predicts about 0.2 µV at the IMU, well below target.

## 5. Thermal Budget

Example load: an IMU AVDD drawing about 50 mA.

- Dissipation: P = (4.0 − 3.3) V × 0.05 A ≈ **35 mW**
- Temperature rise with θJA ≈ 35 °C/W: about 1.2 °C, which is negligible.

Thermal stability still matters, because LDO heat near the IMU causes bias drift. Mitigations:

- Place the LDO at least 15 mm from the IMU.
- Add a thermal relief break in the copper pour between them.

## 6. Grounding and Layout Isolation

1. **Split ground strategy.** Use a solid ground plane under the analog section. Join the AGND and DGND regions at a single point near the LDO ground and IMU ground pins. Do not slot planes under any signal crossing.
2. **Star point.** Bring the LDO GND, C_OUT return, and IMU AGND together at one via cluster.
3. **Guard ring.** Surround the AVDD trace and NR node with a grounded copper guard to limit capacitive pickup.
4. **Keep-out zone.** Keep the buck inductor and switch node at least 20 mm from the IMU. Orient the inductor so its field axis is perpendicular to the IMU.
5. **Layer stack.** Use L1 for components, L2 for a solid GND plane, L3 for power, and L4 for signal and GND. Never route the switch node under the IMU.
6. **Mechanical decoupling.** Mount the IMU on its own board island, or on the same board with vibration-damped mounting. Keep the power stage on the rigid section.

## 7. Noise Budget at the IMU AVDD Pin (10 Hz–100 kHz)

| Contributor | Estimate |
|---|---|
| TPS7A4700 intrinsic noise | ~4.2 µV RMS |
| Residual buck ripple (filtered) | < 1 µV RMS |
| Ground-bounce coupling (layout-dependent) | ~2 µV RMS |
| MLCC microphonics (vibration-dependent) | < 1 µV RMS with C0G/polymer |
| **Root-sum-square total** | **~5 µV RMS** (target < 10 µV) ✔ |

## 8. Sequencing and Protection

- **Power-up order:** buck → AVDD LDO → VDDIO LDO, with about 5 ms between stages. Check the IMU datasheet, because some parts require VDDIO to come up first or within a timing window.
- **Power-good:** monitor PG from the buck, and use a supervisor such as the TPS3890 on AVDD to hold the IMU in reset.
- **Fault protection:** the TPS7A4700's internal current limit and thermal shutdown cover these faults. Add a resettable PTC upstream.

## 9. Verification Plan for Cycle 21

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise spectral density | Spectrum analyzer with an LNA (×100) or a dedicated noise analyzer | < 10 µV RMS integrated |
| Ripple at fsw | Oscilloscope, 1:1 probe, short ground spring, 20 MHz BW off | < 100 µV p-p |
| PSRR | Bode analyzer injection (e.g., Picotest J2120A) | > 60 dB at fsw |
| Load transient | 10 → 60 mA step | < 5 mV deviation |
| IMU-level performance | Allan variance of the gyro, static, comparing bench linear supply vs. this design | No degradation in bias instability or ARW |
| Vibration test | Shaker table sweep (20 Hz–2 kHz), monitoring AVDD and IMU output | No spurs correlated with the supply |
| Thermal drift | −40 to +85 °C chamber | Bias drift within IMU spec |

The Allan variance comparison against a bench linear supply is the decisive test. If both curves overlay, the supply is transparent to the sensor.

## 10. Cycle 21 Deliverables and Open Items

- [ ] Schematic capture, including both LDO channels and the filter network
- [ ] Confirm the IMU part's AVDD/VDDIO current and sequencing requirements
- [ ] Verify C_NR selection against the start-up time requirement
- [ ] Layout review against the isolation rules in Section 6
- [ ] Decide whether to replace the VDDIO LDO with a lower-cost device (e.g., TPS7A20) if analysis confirms that noise on I/O is non-critical
- [ ] Execute the verification matrix and feed the results into Cycle 22

If you share the specific IMU part, input bus voltage, and enclosure constraints, I can refine the component values, ANY-OUT strapping, and noise budget to match.