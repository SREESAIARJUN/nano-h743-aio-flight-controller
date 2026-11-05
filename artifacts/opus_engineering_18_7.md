# IMU Low-Noise Power Supply Architecture: TPS7A4700-Based Isolation (Cycle 18)

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU rail noise (10 Hz–100 kHz) | < 10 µVrms | Prevents bias instability and ARW degradation in MEMS gyros and accelerometers |
| Switching ripple at IMU pins | < 50 µVpp | Avoids aliasing into the IMU's internal ADC and the sense-element resonance band |
| Rail isolation | Analog and digital IMU domains separated from the system bus | Stops processor, radio, and motor transients from coupling into sensor rails |
| Input bus | 9–36 V (12/24/28 V nominal) | Typical vehicle, avionics, or robotic bus |
| IMU load | 3.3 V @ ≤ 150 mA (analog + digital core) | Covers ADIS1647x/1649x-class and similar tactical-grade IMUs |

## 2. Top-Level Architecture

```
 VBUS (9–36 V)
    │
 [Input protection: TVS + reverse-polarity FET + CM choke]
    │
 [Stage 1: Sync buck pre-regulator, spread-spectrum OFF, fsw fixed]
    │  e.g., LMR36015 / TPS54360B → 5.0 V @ ~400 kHz
    │
 [Stage 2: Two-stage π / LC post-filter]
    │  Ferrite (high-Z @ fsw) + 10 µF ‖ 100 nF, then 2.2 µH + 22 µF
    │
    ├──► [Stage 3a: TPS7A4700 #1] → 3.3 V  IMU_AVDD (sensor/analog)
    │
    └──► [Stage 3b: TPS7A4700 #2] → 3.3 V  IMU_VDDIO / digital core
                (or a lower-cost LDO if the digital rail is not noise-critical)

 Optional galvanic tier (Section 6): isolated DC-DC before Stage 2.
```

**Principle:** the switcher provides efficiency, the passive filter attenuates high-frequency ripple where LDO PSRR rolls off, and the TPS7A4700 provides low-frequency PSRR and an ultra-low intrinsic noise floor.

## 3. TPS7A4700 Configuration

### Key device characteristics
Verify these against the current datasheet (SBVS204).

- VIN: 3 V – 36 V
- VOUT: 1.4 V – 20.5 V
- IOUT: up to 1 A
- Output noise: about 4 µVrms (10 Hz–100 kHz)
- PSRR: about 82 dB at 100 Hz, and roughly 55 dB or more out to around 1 MHz
- Dropout: about 307 mV at 1 A
- Output voltage set by ANY-OUT pins, so no external feedback resistors are needed. This removes resistor thermal noise and tolerance drift.

### ANY-OUT programming
VOUT = 1.4 V + Σ(grounded pin weights).

| Rail | Pins tied to GND | Result |
|---|---|---|
| 3.3 V IMU | 1P6V + 0P2V + 0P1V | 1.4 + 1.6 + 0.2 + 0.1 = **3.3 V** |
| 5.0 V (if IMU needs 5 V) | 3P2V + 0P4V | 1.4 + 3.2 + 0.4 = **5.0 V** |

Leave unused ANY-OUT pins floating, as the datasheet specifies.

### Headroom selection
- Dropout at 150 mA is about 50 mV, but PSRR collapses near dropout.
- Use VIN = 5.0 V for a 3.3 V output, giving 1.7 V headroom. This keeps full PSRR across the frequency range.
- Dissipation per LDO: (5.0 − 3.3) × 0.15 A ≈ 0.26 W. With θJA ≈ 35–40 °C/W on a good thermal pad, the temperature rise is about 10 °C. This is acceptable.

### Passive components
| Pin | Component | Purpose |
|---|---|---|
| IN | 10 µF X7R + 100 nF, placed close | Local input decoupling |
| OUT | 10 µF X7R (≥ 10 µF required for stability) + 100 nF at the IMU pin | Stability and HF bypass |
| NR/SS | 1 µF low-leakage (C0G or X7R) | Filters reference noise and sets soft-start (slow ramp limits inrush into IMU caps) |
| Bypass / FB-feedforward (per datasheet) | 10 nF | Lowers the noise floor further |
| EN | RC from Stage 1 power-good | Sequencing (Section 7) |

Account for DC-bias derating of ceramics: use 16 V or higher rated parts so effective capacitance stays above about 70 % of nominal.

## 4. Noise Budget (worst-case estimate)

| Source | At buck output | After LC filter (~40 dB @ 400 kHz) | After LDO PSRR (~55 dB) | At IMU |
|---|---|---|---|---|
| Buck ripple | 10 mVpp | 100 µVpp | ~0.18 µVpp | negligible |
| Buck switching edges (≥ 10 MHz) | ~50 mVpk spikes | ~0.5 mV (ferrite) | LDO is ineffective here, so rely on output 100 nF and layout | < 20 µVpp target |
| LDO intrinsic | — | — | ~4 µVrms | ~4 µVrms |
| **Total** | | | | **< 5 µVrms + ripple well under 50 µVpp** ✔ |

**Frequency planning:**
- Fix the buck fsw (≥ 400 kHz) well away from the IMU's internal sample and decimation rates and from the MEMS sense resonance (typically 15–30 kHz).
- Disable spread-spectrum unless EMI compliance requires it. Dithered spurs can fold into the sensor bandwidth.

## 5. Isolation Strategies (noise domain)

1. **Dedicated regulators.** IMU rails never share an LDO with the MCU, radio, or GNSS.
2. **Ferrite-bead islands.** Place a bead (e.g., 600 Ω @ 100 MHz, rated ≥ 3× load current) between each LDO output and the IMU pin cluster.
3. **Ground strategy.**
   - Use a solid, unsplit ground plane under the IMU.
   - Keep return currents of the switcher physically confined: place its hot loop on the far side of the board and stitch it with vias.
   - Avoid slots that force IMU return currents to detour.
4. **Placement.**
   - Put the LDOs within about 15 mm of the IMU.
   - Keep the switcher at least 30 mm away, with the inductor oriented so its field is away from the IMU (or use a shielded inductor).
5. **Mechanical decoupling.** Keep the switcher inductor off the IMU mounting island so magnetostriction and acoustic noise are not mechanically coupled.

## 6. Optional Galvanic Isolation Tier

Use this tier when the IMU must float relative to a noisy system ground, such as motor drives or long harnesses.

```
VBUS → buck 5 V → SN6505B push-pull driver + low-capacitance transformer (e.g., 1:1.3)
     → rectifier → ~6 V isolated → LC filter → TPS7A4700 → 3.3 V ISO
Data: ISO7741 / ISO7763 digital isolator for SPI + DRDY
```

- The TPS7A4700's PSRR at the SN6505 frequency (~410 kHz) cleans the rectifier ripple.
- Choose a transformer with low interwinding capacitance (< 10 pF) to minimize common-mode injection.

## 7. Sequencing & Protection

- **Startup order:** Stage 1 PG → EN of LDO #1 (AVDD) → about 1 ms delay → LDO #2 (VDDIO) → IMU reset release from the MCU. Match this to the IMU datasheet's sequencing requirements.
- **Startup ramp:** the NR/SS cap gives a soft start of roughly 10–20 ms. Confirm it meets the IMU's minimum ramp requirement.
- **Overcurrent:** the TPS7A4700 has internal current limit and thermal shutdown. Add a resettable fuse at VBUS.
- **Brown-out:** monitor the 3.3 V rail with a supervisor such as the TPS3890 and flag IMU data as invalid during undervoltage.

## 8. Verification Plan

| Test | Method | Pass criterion |
|---|---|---|
| Output noise density | Low-noise preamp (e.g., 40 dB) + FFT analyzer, 10 Hz–100 kHz | < 10 µVrms integrated, no discrete spurs > 1 µV |
| Ripple / spikes | 50 Ω coax tip-and-barrel probing, 500 MHz BW | < 50 µVpp at fsw, < 20 µVpp HF |
| PSRR | Injection transformer on LDO input | Within 3 dB of datasheet at 100 Hz, 10 kHz, 400 kHz |
| Load transient | 10→150 mA step | < 5 mV deviation, recovers < 50 µs |
| System-level | Allan variance of IMU, linear supply vs. this architecture | ARW and bias instability within 5 % of the linear-bench reference |
| Thermal | IR camera at max VIN and load | LDO TJ < 85 °C |

## 9. Cycle 18 Change Notes

I don't have the details of your previous cycles. The items below are recommended deltas to check against the prior revision, not a record of what changed:

- [ ] Confirm the ANY-OUT strapping in the schematic. Mis-strapped pins are the most common TPS7A4700 bring-up error.
- [ ] Re-check the effective output capacitance after DC-bias derating.
- [ ] Verify the buck fsw is not a harmonic or sub-harmonic of the IMU ODR or the SPI clock.
- [ ] Revalidate Allan variance with the final enclosure and harness installed.

If you share the IMU part number, bus voltage, and the open issues from Cycle 17, I can tighten the component values and noise budget to your specific configuration.