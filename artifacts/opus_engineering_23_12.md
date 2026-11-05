# Low-Noise IMU Power Supply Architecture Using the TPS7A4700, Design Cycle 23

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail (VDD) | 3.3 V ±2% | Typical for MEMS IMUs (ADIS1647x, BMI088, ICM-42688, etc.) |
| Integrated output noise | < 10 µVrms (10 Hz–100 kHz) | Keeps supply noise well below the IMU's internal ADC and bias-instability floor |
| Ripple at IMU pin | < 50 µVpp at the switcher frequency | Prevents aliasing into the gyro/accel bandwidth |
| PSRR at 100 kHz–1 MHz | > 60 dB system-level | Rejects buck switching and harmonics |
| Load | 20–100 mA typical, 250 mA design margin | Covers the IMU plus heater or local reference if present |
| Isolation | Separate analog and digital domains, optional galvanic isolation | Stops digital and motor return currents from corrupting the sensor |

---

## 2. Top-Level Architecture

```
 VIN (9–28 V system bus)
   │
   ├─[TVS + reverse-polarity FET + input LC EMI filter]
   │
   ▼
 Stage 1: Synchronous Buck Pre-Regulator (e.g., TPS62913 / LMR33630, spread-spectrum)
   │   VOUT = 4.3 V, fsw ≥ 2 MHz (well above IMU bandwidth)
   │
   ▼
 Stage 2: Passive π-Filter (ferrite + 2nd-stage LC), corner frequency ~10–20 kHz
   │
   ├──────────────────────────────┬──────────────────────────────┐
   ▼                              ▼                              ▼
 Stage 3A: TPS7A4700          Stage 3B: TPS7A20 (or        (Optional) Stage 3C:
 VOUT = 3.3 V                 TPS7A4700 #2)                Isolated DC-DC + LDO
 IMU ANALOG VDD               IMU VDDIO / digital, 1.8/3.3 V for galvanically isolated
   │                              │                        IMU island (SN6505B + LDO)
   ▼                              ▼
 Local post-filter            Local decoupling
 (ferrite + 10 µF + 100 nF)       │
   │                              │
   └──────────► IMU ◄─────────────┘
                 │
          SPI ──► ISO7741 digital isolator (optional) ──► MCU/FPGA
```

**Partitioning principle:** each noise-reduction stage targets a different frequency band.
- The buck provides efficiency.
- The π-filter attenuates frequencies above about 100 kHz, where LDO PSRR starts to fall.
- The TPS7A4700 provides low-frequency PSRR and a very low intrinsic noise floor.
- The local ferrite decouples the IMU's own switching (gyro drive and ADC sampling) from the regulator.

---

## 3. Stage Details

### 3.1 Pre-Regulator (Stage 1)
- **Output voltage:** 4.3 V. This gives about 1.0 V of headroom above 3.3 V.
  - The TPS7A4700 dropout is about 307 mV at 1 A, and lower at IMU currents.
  - Keeping ≥ 0.7–1 V of headroom keeps the pass device out of near-dropout, where PSRR collapses.
- **Spread-spectrum and frequency choice:** enable spread-spectrum, or sync to a clean clock, at ≥ 2 MHz. This keeps the fundamental and its harmonics far from the IMU's internal resonator frequencies (typically 10–30 kHz for MEMS gyros).
- **Reason for a pre-regulator:** dropping 24 V to 3.3 V directly in the LDO would dissipate about 2 W at 100 mA, which causes thermal drift near the IMU. That drift is itself an error source.

### 3.2 Inter-Stage π-Filter (Stage 2)
- **Topology:** C1 (10 µF X7R) → L (4.7 µH shielded, low DCR) or ferrite (600 Ω at 100 MHz, rated ≥ 1 A) → C2 (22 µF X7R) + damping (1 Ω series with 47 µF, or a polymer cap with ESR around 50 mΩ).
- **Corner frequency:** f_c ≈ 1/(2π√(LC)) ≈ 15 kHz. This gives about 40 dB/decade of attenuation above that point.
- **Damping:** required to avoid a Q-peak that would amplify noise or ring into the LDO input.

### 3.3 TPS7A4700 Configuration (Stage 3A, IMU Analog Rail)

**Output voltage via ANY-OUT (no external resistors, so no resistor noise or drift):**

V_OUT = 1.4 V + 1.6 V (1P6V) + 0.2 V (0P2V) + 0.1 V (0P1V) = **3.3 V**

- Tie the 1P6V, 0P2V and 0P1V pins to GND.
- Leave all other ANY-OUT pins floating.

| Pin / Component | Value | Notes |
|---|---|---|
| IN capacitor | 10 µF X7R + 1 µF X7R, 0805 | Placed within 2 mm of the IN pins |
| OUT capacitor | 10 µF X7R (≥ 4.7 µF effective after DC bias) + 100 nF | Ceramic is stable; check effective capacitance at 3.3 V bias |
| NR/SS capacitor (C_NR) | 10 nF C0G | Sets the reference noise filter and soft-start; larger values lower 10 Hz–1 kHz noise but slow start-up |
| Feed-forward / bypass capacitor | 10 nF C0G across the internal divider (per datasheet) | Improves noise and PSRR in the 1–100 kHz band |
| EN | Drive from the sequencer or the pre-regulator's PGOOD via RC | Ensures the buck is stable before the LDO starts |
| Thermal pad | Solder to the ground pour with ≥ 9 vias | RθJA ≈ 35 °C/W on a JEDEC board |

**Thermal check:**
- P_diss = (4.3 − 3.3) V × 0.1 A = 100 mW, giving ΔT ≈ 3.5 °C.
- This is negligible. Still, keep the LDO at least 15 mm from the IMU, or behind a thermal slot, to avoid local gradients that couple into gyro bias.

**Expected performance:**
- Intrinsic noise: about 4.2 µVrms (10 Hz–100 kHz) at its datasheet configuration.
- PSRR: about 80 dB at low frequency, falling to about 55 dB near 1 MHz. Combined with the π-filter, the result is > 90 dB at the switching frequency.

### 3.4 Digital Rail (Stage 3B)
- **Regulator:** a separate low-cost LDO (TPS7A20, about 7 µVrms) for VDDIO.
  - Use a second TPS7A4700 only if the IMU has a single supply pin and VDDIO shares internal circuitry.
- **Purpose:** prevents SPI edge currents and I/O switching from modulating the analog rail.

### 3.5 Optional Galvanic Isolation (Stage 3C)
Use this for motor-drive platforms or for long cable runs with ground shifts.
- **Power path:** SN6505B push-pull transformer driver → 1:1.5 transformer → rectifier → π-filter → TPS7A4700 on the isolated side.
- **Data path:** ISO7741 (4-channel, 100 Mbps) for SPI, plus a separate channel for DRDY/INT.
- **Return path:** a stitching capacitor (1–2 nF Y-rated) across the barrier gives the transformer's high-frequency common-mode currents a return path.

---

## 4. Noise Budget at the IMU Supply Pin

| Contributor | Level at IMU (10 Hz–100 kHz) |
|---|---|
| TPS7A4700 intrinsic | ~4.2 µVrms |
| Residual buck ripple (2 MHz, after filter, LDO and local LC) | < 1 µVrms in-band (out-of-band component < 20 µVpp) |
| Pre-regulator low-frequency noise × LDO PSRR | < 0.5 µVrms |
| Ground-return IR noise (with star grounding) | < 2 µVrms |
| **Root-sum-square total** | **≈ 5 µVrms**, meeting the < 10 µVrms target |

---

## 5. Grounding and Layout Rules

1. **Plane structure:** use a solid ground plane, partitioned by placement rather than by cuts.
   - Keep the buck in one corner and the IMU in the opposite region.
   - Route no digital or switching return current under the IMU.
2. **Star point:** tie the analog ground (AGND) and power ground (PGND) at the TPS7A4700 GND/thermal pad.
3. **Kelvin sensing:** the TPS7A4700 has an internal sense path. Place the output capacitor between the LDO and the IMU, and run a short, wide trace to the IMU VDD pin.
4. **Buck switch node:** keep the SW node area minimal, on the opposite board side from the IMU, with the shielded inductor oriented away from the sensor.
5. **IMU keep-out:** no vias, high-speed traces or copper pours that could induce mechanical stress or thermal gradients under the IMU, per the IMU vendor's guidance.
6. **Guard ring:** surround the analog rail traces with a grounded guard ring stitched with vias at a pitch of about 2 mm.

---

## 6. Power Sequencing

1. VIN reaches the UVLO threshold, and the buck starts.
2. The buck asserts PGOOD.
3. After about 1 ms (RC delay), the TPS7A4700 EN goes high, and soft-start is set by C_NR (about 1–5 ms).
4. The digital rail comes up after the analog rail, or at the same time, as the IMU datasheet requires.
5. The MCU holds the IMU in reset until both rails are in regulation plus 10 ms.

---

## 7. Verification Plan (Cycle 23 Exit Criteria)

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise spectral density | LNA (e.g., 40–60 dB gain, battery-powered) + FFT analyzer, 10 Hz–100 kHz | ≤ 10 µVrms integrated; no spurs > 1 µV |
| Switching ripple | 50 Ω coax + 1 GHz scope at the IMU pin, 20 MHz BW and full BW | < 50 µVpp |
| PSRR | Bode 100 / injection transformer at the LDO input | > 60 dB from 100 Hz to 2 MHz (system) |
| Load transient | 10–100 mA step, 1 µs edge | < 5 mV deviation, no ringing |
| IMU-level validation | Allan variance with the IMU static on both the bench linear supply and this design | Bias instability and ARW within 5% of the linear-bench baseline |
| Thermal | IR camera at full load | IMU ΔT from the regulator < 1 °C |

---

## 8. Key Bill of Materials

| Ref | Part | Function |
|---|---|---|
| U1 | TPS62913 / LMR33630 | Spread-spectrum buck pre-regulator |
| FB1, L1 | BLM31KN601 / XFL4020-472 | π-filter |
| U2 | **TPS7A4700RGW** | Ultra-low-noise IMU analog rail |
| U3 | TPS7A2018 / TPS7A2033 | VDDIO rail |
| U4 (opt.) | SN6505B + 760390015 transformer | Isolated power |
| U5 (opt.) | ISO7741 | SPI isolation |
| C_NR, C_FF | 10 nF C0G | Noise-reduction and feed-forward capacitors |

---

## 9. Open Items for Cycle 24

- Confirm the IMU part number. Adjust V_OUT, headroom and VDDIO architecture if the IMU needs a 5 V supply (ANY-OUT for 5.0 V: 1.4 + 3.2 + 0.4).
- Decide on galvanic isolation based on measured ground-shift amplitude at the integration platform.
- Evaluate whether replacing C_NR with 1 µF lowers 1/f noise, and whether the slower start-up is acceptable.
- Measure vibration sensitivity of the ceramic capacitors (microphonics) near the IMU. Consider C0G or film capacitors in the final post-filter if needed.