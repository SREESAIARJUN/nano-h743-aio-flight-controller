# Low-Noise IMU Power Architecture: TPS7A4700 Isolation Stage (Cycle 20)

I don't have the earlier design cycles, so I've assumed a typical baseline: a 12–24 V system bus, a MEMS IMU in the ADIS16470 / ICM-42688 class drawing less than 100 mA, and a separate VDDIO for the digital interface. Adjust the values below if your baseline differs.

---

## 1. Architecture Overview

```
 VBUS (12–24 V)
    │
 [TVS + reverse-polarity FET + input EMI filter]
    │
 ┌──▼────────────────────────┐
 │ Stage 1: Sync buck         │  e.g. LMR33630 / TPS62130
 │ Vout = 4.5 V, fsw ≈ 2.1 MHz│  (spread-spectrum OFF near IMU band, or
 └──┬────────────────────────┘   fixed fsw far above IMU bandwidth)
    │
 ┌──▼────────────────────────┐
 │ Stage 2: Damped LC π-filter│  ~40 dB attenuation at fsw
 └──┬────────────────────────┘
    │ 4.4 V (quiet pre-rail)
    ├──────────────────────────────┐
 ┌──▼──────────────────┐   ┌──────▼─────────────────┐
 │ Stage 3a: TPS7A4700  │   │ Stage 3b: TPS7A4700 or │
 │ IMU ANALOG (3.3 V)   │   │ TPS7A20 → VDDIO (1.8 V)│
 └──┬──────────────────┘   └──────┬─────────────────┘
 [FB bead + local decoupling]  [FB bead + decoupling]
    │                              │
   IMU VDD                      IMU VDDIO / MCU SPI
```

The key principle is two-stage noise rejection. The passive filter handles high-frequency switching residue, where LDO PSRR rolls off. The TPS7A4700 handles low-frequency ripple and its own intrinsic noise floor.

---

## 2. TPS7A4700 Configuration

| Parameter | Value / Setting | Rationale |
|---|---|---|
| Vin | 4.4 V nominal | Gives about 1.1 V headroom. PSRR degrades sharply below about 0.5–1 V headroom, even though dropout is only about 300 mV at 1 A. |
| Vout | 3.3 V via ANY-OUT | 1.4 V (Vref) + 1.6 V + 0.2 V + 0.1 V. Ground pins **1P6V, 0P2V, 0P1V**; leave the others open. |
| Output noise | ≈4.2 µVrms (10 Hz–100 kHz) | Datasheet typical, with C_NR fitted. |
| C_NR (NR pin) | 10 µF ceramic, X7R | Filters reference noise. It also sets the soft-start ramp, which is slow; budget this in the boot sequence. |
| C_IN | 10 µF X7R + 100 nF, at the pin | Low-impedance local source. |
| C_OUT | 10 µF + 1 µF + 100 nF X7R | Stable with ceramics; covers broadband impedance. |
| SENSE | Kelvin-routed to the IMU VDD pin | Removes IR drop and trace-borne noise. |
| EN | Driven by buck PG (via RC, ~1 ms) | Ensures the pre-rail is settled before the LDO starts. |
| PG | To MCU GPIO (10 kΩ pull-up to VDDIO) | Hold IMU reset or ignore IMU data until PG is high. |
| Package | 5×5 VQFN-20, thermal pad to GND plane with ≥9 vias | |

**Thermal check:** P = (4.4 − 3.3) V × 0.08 A ≈ 90 mW. The resulting temperature rise is negligible (<5 °C). This matters because thermal gradients near the IMU cause bias drift.

---

## 3. π-Filter Design (Between Buck and LDO)

- **L:** 4.7 µH shielded inductor with low DCR, followed by a ferrite bead (≈600 Ω @ 100 MHz, rated >3× the load current).
- **C:** 22 µF X7R on each side of the inductor.
- **Damping:** add a 47 µF polymer cap with ESR ≈ 0.3 Ω in parallel with the output C. Alternatively, use a 1 Ω + 22 µF RC snubber.
- **Cutoff frequency:** f_c = 1/(2π√(4.7 µH × 22 µF)) ≈ 15.7 kHz.
- **Attenuation at 2.1 MHz:** about 40 dB (second-order rolloff above f_c).
- **Resonance:** verify the Q is below about 2. Undamped peaking at f_c can land inside the IMU's sensor bandwidth.

---

## 4. Ripple Budget (Cascade)

| Point | Ripple at 2.1 MHz |
|---|---|
| Buck output | ~10 mVpp |
| After π-filter (−40 dB) | ~100 µVpp |
| After TPS7A4700 (PSRR ≈ −40 dB at MHz) | ~1 µVpp |
| After output bead + caps | sub-µV, below the LDO's own noise floor |

Low-frequency content (<10 kHz) is dominated by the LDO's PSRR, which is greater than 70 dB in this region. Load-induced ripple from the IMU's internal sampling is handled by C_OUT and the low output impedance.

---

## 5. Isolation Options

**A. Noise isolation (default).** Use the topology above with a dedicated LDO per IMU rail. Do not share the IMU LDO with the MCU, radio, or motor drivers.

**B. Galvanic isolation (if required).** Use this when IMU ground must float from a noisy power or motor ground.
- Insert a push-pull isolated converter before Stage 2: SN6505B with a 1:1.5 transformer, producing ~5 V isolated.
- Use ISO7741-class digital isolators for SPI.
- Keep the same π-filter and TPS7A4700 chain on the isolated side. The SN6505 runs at ~420 kHz, so increase the π-filter L to 10 µH.

---

## 6. Layout and Grounding Rules

1. Place the IMU, LDO, and output caps in a quiet zone, at least 15 mm from the buck inductor and switch node.
2. Use a solid, uninterrupted GND plane under the IMU. Do not route digital traces under the IMU or the LDO NR network.
3. Use a single-point tie between the power ground and the IMU analog ground, located at the LDO GND pad.
4. Keep the NR and SENSE traces short and guarded by ground. They are the highest-impedance and most noise-sensitive nodes.
5. Use C0G/X7R dielectrics only, sized at 0805 or larger. Avoid Y5V, whose DC-bias derating collapses capacitance. Also watch for piezoelectric (microphonic) MLCC effects near a vibration sensor; consider polymer caps for the bulk positions.
6. Mount the IMU mechanically away from the inductors. Inductor magnetostriction can couple acoustically into MEMS sensors.

---

## 7. Verification Plan

| Test | Method | Pass Criterion (example) |
|---|---|---|
| Output noise | LNA (×100) + spectrum analyzer, 10 Hz–100 kHz | ≤ 6 µVrms |
| Switching residue | 50 Ω coax probe, 1–30 MHz | No spur > 5 µVpp at fsw or harmonics |
| PSRR | Inject on Vin (Bode 100 + injector) | > 60 dB at 1 kHz, > 40 dB at 2 MHz |
| Load transient | Step 10 → 80 mA | < 5 mV deviation, no ringing |
| IMU-level | Allan variance, static, 2 h, with the buck on vs. a lab linear supply | No measurable degradation in angle random walk (ARW) or bias instability |
| Thermal | IR camera at full load | IMU ΔT < 2 °C from ambient |

---

## 8. Cycle 20 Open Items

- Confirm the IMU's supply-sensitivity spec (mg/V or °/s/V) to tighten the noise target.
- Decide between Option A and Option B based on the system ground architecture.
- Check that the NR-cap-driven soft-start time fits the boot-time budget. Reduce C_NR to 1 µF if needed, at the cost of a small noise increase.
- Characterize MLCC microphonics under the vibration profile.

If you share the specific IMU part number and the input bus voltage, I can tighten the component values and the noise budget accordingly.