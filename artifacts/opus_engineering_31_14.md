# Low-Noise IMU Power Supply Architecture Using the TPS7A4700

This design covers the power chain, LDO configuration, isolation, layout and verification for a precision IMU rail built around the TI TPS7A4700. I don't have the context or constraints from "Cycle 31," so this treats it as a design iteration and assumes a 12 V or 24 V input bus. Adjust the voltages to your actual system.

---

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail noise | < 10 µV RMS (10 Hz – 100 kHz) | Supply noise couples into gyro and accelerometer bias and ARW |
| Ripple at switcher frequency | < 50 µV p-p at the IMU pins | Avoids aliasing into the sensor's internal ADC/filters |
| PSRR, chain total | > 100 dB at 1 kHz, > 60 dB at 1–2 MHz | Rejects bus and pre-regulator disturbances |
| Load | Typically 5–70 mA (IMU-dependent) | Well under the LDO's 1 A rating, so it runs cool |
| Isolation | Separate analog and digital rails, controlled return paths | Keeps MCU, SPI and radio transients off the sensor supply |

---

## 2. Top-Level Architecture

```
 VBUS (12/24 V)
     │
 [Input protection: TVS + reverse-polarity FET + EMI filter]
     │
 [Buck pre-regulator → 4.5 V, fsw ≥ 2 MHz, synced or spread-spectrum]
     │
 [Pi filter: 10 µF ─ ferrite bead (600 Ω @100 MHz, low DCR) ─ 10 µF + 22 µF polymer]
     │
     ├──► TPS7A4700 #1 → 3.3 V  IMU_AVDD (analog/sensor core)  ← quiet rail
     │
     └──► TPS7A4700 #2 (or a lower-cost LDO) → 3.3 V / 1.8 V  IMU_VDDIO
                                                     │
                                         [Digital isolator / series-R on SPI]
                                                     │
                                                   MCU domain (separate buck/LDO)
```

**Key principles**

- **Two-stage regulation.** The buck stage provides efficiency, and the LDO provides noise and PSRR performance.
- **Dedicated LDO for the IMU's analog supply.** No other loads share it.
- **Digital I/O supplied separately.** SPI edge currents then cannot modulate the analog rail.
- **Pi filter between buck and LDO.** It attacks the 1–10 MHz region, where LDO PSRR naturally rolls off.

---

## 3. TPS7A4700 Key Characteristics

These are typical datasheet values. Verify them against the current revision before freezing the design.

- V_IN: 3 V – 36 V; V_OUT: 1.4 V – 20.5 V; I_OUT up to 1 A
- Output noise: about 4.17 µV RMS (10 Hz – 100 kHz)
- PSRR: about 82 dB at 100 Hz, and still above 55 dB near 1 MHz
- Dropout: about 307 mV at 1 A. It is far less at IMU load currents.
- ANY-OUT™ programming: the output voltage is set by grounding internal-resistor pins, with no external divider. This avoids resistor noise and tolerance errors.
- Package: 5×5 mm QFN-20 (RGW) with an exposed thermal pad

---

## 4. LDO Configuration

### 4.1 Output Voltage via ANY-OUT

V_OUT = 1.4 V + Σ(grounded pin weights). The available weights are 0.1, 0.2, 0.4, 0.8, 1.6, 3.2 and 6.4 V.

| Target | Pins tied to GND | Result |
|---|---|---|
| 3.3 V | 1P6, 0P2, 0P1 | 1.4 + 1.6 + 0.2 + 0.1 = 3.3 V |
| 1.8 V | 0P4 | 1.4 + 0.4 = 1.8 V |
| 5.0 V | 3P2, 0P4 | 1.4 + 3.2 + 0.4 = 5.0 V |

Leave unused ANY-OUT pins floating, and keep their traces short to avoid noise pickup.

### 4.2 Headroom Selection

- Pre-regulator output: 4.5 V, giving V_IN − V_OUT = 1.2 V for a 3.3 V output.
- At least 1 V of headroom keeps the pass device out of its low-PSRR region. PSRR degrades sharply as the LDO nears dropout.
- Power dissipation at 70 mA is (4.5 − 3.3) × 0.07 ≈ 84 mW. This is negligible, and the thermal drift it causes near the IMU is minimal.

### 4.3 Capacitors

| Pin | Value | Type | Notes |
|---|---|---|---|
| IN | 10 µF + 0.1 µF | X7R, ≥ 16 V | Place close to the pin, plus the pi-filter bulk upstream |
| OUT | 10 µF minimum, often 22–47 µF | X7R plus optional polymer | Meets stability requirements. A larger value lowers high-frequency impedance. |
| NR/SS | 1–10 µF (per datasheet noise conditions) | Low-leakage X7R/C0G-class | Sets the reference filter corner and the soft-start time. A larger value gives lower noise but a slower start. |
| BYPASS | about 10 nF | C0G | Reduces error-amplifier noise |

**Vibration note specific to IMUs.** High-K MLCCs are piezoelectric and microphonic. On a vibrating platform they can convert mechanical energy into rail noise at vibration frequencies, which falls directly in the IMU's band of interest. Mitigations:
- Use C0G for the small reference and bypass caps.
- Use polymer tantalum or polymer aluminium for part of the output bulk capacitance.
- Orient MLCCs consistently and keep them away from board flex zones.

### 4.4 Enable and Sequencing

- Drive EN from the MCU or a supervisor. Bring IMU_AVDD up before or simultaneously with VDDIO, following the sensor datasheet's sequencing rules.
- The NR/SS capacitor gives a controlled ramp, which limits inrush into the output capacitors.
- If sequencing matters, add a voltage supervisor (for example, a TPS3840-class device) on IMU_AVDD to gate the IMU reset.

---

## 5. Isolation Strategy

### 5.1 Rail Isolation
- IMU_AVDD comes from its own TPS7A4700, with no shared loads.
- IMU_VDDIO is separate, because digital edge currents return through the VDDIO path.
- Add a ferrite bead plus 1 µF local decoupling at each IMU supply pin. This is a final, local filter stage.

### 5.2 Ground Strategy
- Use a solid, unbroken ground plane under the IMU and LDO. Avoid splitting planes under signal traces.
- Partition the board instead: place the switcher in one corner, the LDO in a quiet zone, and the IMU furthest from the switcher.
- Route the buck's high di/dt loop (input cap, switch, inductor) tightly. Keep it on a local pour stitched to the main plane at a single region.

### 5.3 Signal Isolation (optional, harsh environments)
- Use a digital isolator (ISO77xx-class) on SPI or interrupt lines if the MCU domain is noisy or ground potential differences exist.
- At a minimum, place 22–33 Ω series resistors at the SPI drivers to slow edges and reduce return-current spikes.

---

## 6. Pre-Regulator Selection Guidelines

- Use a synchronous buck such as the LMR33630 or TPS62130 class. Alternatively, use a low-EMI part such as the LMQ6xxxx series.
- A switching frequency of at least 2 MHz pushes ripple above the IMU's internal filter bandwidth and into the range where the pi filter is effective.
- Synchronize to a known clock, or use spread spectrum. Make sure no harmonic or beat frequency lands in the IMU's sample-rate or anti-alias band. Check the sensor's ODR and internal ADC clock.
- Use a shielded inductor, and place it away from the IMU and LDO.

---

## 7. Noise Budget (Example)

| Stage | Contribution at the IMU pin |
|---|---|
| Buck ripple at 2 MHz | about 10 mV p-p |
| After the pi filter (about −40 dB at 2 MHz) | about 100 µV p-p |
| After the LDO PSRR (about −50 dB at 2 MHz) | about 0.3 µV p-p |
| LDO intrinsic noise | about 4–5 µV RMS (10 Hz – 100 kHz) |
| **Total** | **Dominated by LDO intrinsic noise, about 5 µV RMS** |

This is well under the 10 µV RMS target, leaving margin for layout imperfections.

---

## 8. Layout Checklist

1. Place the LDO within about 10–20 mm of the IMU, but outside the IMU's thermal zone. Keep the buck at least 30 mm away.
2. Route the IN and OUT capacitor returns directly to the LDO's GND pad with vias at the pad.
3. Use a thermal-pad via array to the inner ground. This is mainly for mechanical and ground integrity, since the thermal load is minimal.
4. Use Kelvin routing of the output sense to the IMU. Treat the trace from LDO to IMU as a low-impedance pour, not a thin trace.
5. Keep NR/SS and BYPASS caps immediately adjacent to their pins. These are high-impedance, noise-sensitive nodes.
6. Run no digital traces under the LDO or the IMU, on any layer adjacent to the ground reference.
7. Mount the IMU mechanically according to its datasheet, avoiding board flex near the IMU and its decoupling caps.

---

## 9. Verification Plan

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise | LNA (for example, 40–60 dB gain) feeding a spectrum analyzer, 10 Hz – 100 kHz | < 10 µV RMS |
| Switcher ripple | 50 Ω coax tap or a probe with a short spring ground, 20 MHz BW off | < 50 µV p-p at the IMU pin |
| PSRR | Injection transformer on V_IN, plus a VNA or Bode analyzer | Meets the Section 1 targets |
| Load transient | IMU active and sleep transitions | Rail deviation < 1 mV |
| IMU-level validation | Allan variance comparing the LDO rail against a bench linear supply or battery reference | No measurable ARW or bias-instability degradation |
| Vibration | Shaker table sweep, observing rail noise and IMU output | No rail spurs correlated with vibration (confirms microphonics mitigation) |
| Thermal | IR camera and IMU temperature sensor | LDO self-heating contributes < 0.5 °C at the IMU |

---

## 10. Example BOM (Core Items)

| Ref | Part | Qty |
|---|---|---|
| U1 | Buck pre-regulator (LMR33630 / TPS62130 class) | 1 |
| U2 | TPS7A4700RGWR (IMU_AVDD) | 1 |
| U3 | TPS7A4700RGWR or a low-noise LDO (IMU_VDDIO) | 1 |
| FB1–FB3 | Ferrite bead, 600 Ω @ 100 MHz, DCR < 0.1 Ω | 3 |
| C_NR | 1–10 µF low-leakage | 2 |
| C_BYP | 10 nF C0G | 2 |
| C_OUT | 10–22 µF X7R plus 47 µF polymer | 2 sets |
| U4 (optional) | Voltage supervisor | 1 |
| U5 (optional) | Digital isolator for SPI | 1 |

---

## Open Items

- Confirm the target IMU part number, so its supply range, sequencing rules and current draw are pinned down.
- Confirm the actual input bus voltage and any EMC standard to be met (DO-160, MIL-STD-461, CISPR 25), since these drive the input filter design.
- Cross-check the NR/SS and BYPASS capacitor values against the current TPS7A47 datasheet's noise test conditions.

If you share the IMU model and what changed in Cycle 31, I can tailor the voltages, sequencing and noise budget to it.