# Low-Noise IMU Power Supply Architecture: TPS7A4700 (Cycle 6)

> **Note:** I don't have the records from Cycles 1–5, so this is a self-contained baseline. Where earlier decisions (bus voltage, IMU part number, board stack-up) should override my assumptions, I flag them as **[CONFIRM]**.

---

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU analog rail | 3.3 V ±2% | Typical MEMS IMU range is 3.0–3.6 V (e.g., ADIS1647x/1649x) **[CONFIRM part]** |
| Rail noise (10 Hz–100 kHz) | < 10 µVrms | Keeps supply-induced bias instability below the sensor noise floor |
| Ripple at IMU pins | < 50 µVpp at the switcher frequency | Prevents aliasing into the gyro/accel bandwidth |
| Load current | 50–150 mA steady, 300 mA transient margin | IMU plus optional heater/sync circuitry |
| Isolation from digital/motor noise | > 80 dB at 100 Hz–10 kHz | Covers rotor/PWM harmonics and MCU clock subharmonics |

---

## 2. Top-Level Architecture

```
 VBUS (12–28 V) [CONFIRM]
     │
  [TVS + reverse-polarity FET + input LC EMI filter]
     │
  ┌──┴───────────────────────────────┐
  │ Pre-regulator: sync buck          │  e.g., LMR33630 or TPS62912 (low-ripple buck)
  │ VOUT = 4.2 V, fsw ≥ 2 MHz         │  spread spectrum DISABLED (keeps noise out of IMU band)
  └──┬───────────────────────────────┘
     │
  [Post-buck π-filter: 10 µF – FB (600 Ω@100 MHz, low DCR) – 10 µF + 1 Ω/22 µF damping]
     │
     ├──────────────────────────┐
     │                          │
 ┌───┴──────────────┐   ┌───────┴─────────┐
 │ TPS7A4700 (U1)    │   │ General LDO (U2) │  e.g., TPS7A20 / TLV757
 │ 3.3 V IMU_AVDD    │   │ 3.3 V IMU_VDDIO  │  digital I/O, level shifters
 └───┬──────────────┘   └───────┬─────────┘
     │ star point at IMU         │
  [Local decoupling]          [Local decoupling]
     │                           │
   IMU VDD (analog)          IMU VDDIO / SPI
```

**Key principle:** two-stage regulation. The buck handles efficiency and large input dynamics, and the TPS7A4700 handles noise and PSRR. The digital I/O rail is split onto its own LDO so SPI edge currents never modulate the analog rail.

---

## 3. TPS7A4700 Configuration

### 3.1 Key Device Characteristics (verify against current datasheet rev.)
- VIN: 3 V to 36 V; VOUT: 1.4 V to 34 V; IOUT up to 1 A
- Output noise: ~4.17 µVrms (20 Hz–20 kHz)
- High PSRR at low frequency (~80 dB class near 100 Hz–1 kHz), degrading at MHz frequencies, which is why the π-filter is needed
- Dropout: ~307 mV at 1 A, much lower at IMU currents
- Package: VQFN-20 (RGW) with exposed thermal pad

### 3.2 Output Voltage via ANY-OUT (no external resistors)

VOUT = 1.4 V + Σ(selected pins tied to GND):

| Pin | Weight | Connection for 3.3 V |
|---|---|---|
| 6P4V2 | 6.4 V | Open |
| 3P2V | 3.2 V | Open |
| **1P6V** | 1.6 V | **GND** |
| 0P8V | 0.8 V | Open |
| 0P4V | 0.4 V | Open |
| **0P2V** | 0.2 V | **GND** |
| **0P1V** | 0.1 V | **GND** |

**1.4 + 1.6 + 0.2 + 0.1 = 3.3 V.** ANY-OUT avoids the resistor thermal noise and tolerance drift of an external divider, which is a direct noise benefit. If the design requires 5.0 V instead, ground 3P2V and 0P4V, and raise the buck to about 5.8 V.

### 3.3 Passive Components

| Ref | Value | Type | Purpose |
|---|---|---|---|
| C_IN | 10 µF + 100 nF | X7R, 16 V+ (derated) | Input bypass, placed close to IN pins |
| C_NR/SS | 10 µF | X7R or film, low leakage | Noise-reduction filter on the reference; sets soft-start (~tens of ms) |
| C_BYP (if used per datasheet rev.) | 10 nF | C0G | Reference bypass |
| C_OUT | 10 µF + 1 µF + 100 nF | X7R/C0G | Stability (≥10 µF ceramic) and HF decoupling |
| EN | Tie to VIN via 100 kΩ, or MCU GPIO | — | Sequencing control |

**Ceramic caution:** use 0805/1206 X7R rated ≥2× the operating voltage to limit DC-bias capacitance loss. Avoid Y5V. MLCC piezo (microphonic) effects matter on a vibrating platform. Consider a polymer tantalum on C_NR or soft-terminated MLCCs.

### 3.4 Headroom and Thermal Budget

- Headroom = 4.2 V − 3.3 V = **0.9 V**. This is well above dropout at 150 mA, and keeping ≥0.5–1 V preserves PSRR. PSRR collapses near dropout.
- Dissipation at 150 mA: 0.9 V × 0.15 A = **135 mW**
- θJA ≈ 35 °C/W (with the thermal pad soldered and vias) gives ΔT ≈ 5 °C. This is negligible.
- **Thermal coupling to the IMU:** place U1 at least 15–20 mm away, or isolate it with a copper keep-out/slot. LDO self-heating gradients contribute to gyro bias drift.

---

## 4. Noise and Ripple Budget

| Source | Level at LDO input | Attenuation | Residual at IMU |
|---|---|---|---|
| Buck ripple @ 2 MHz | ~10 mVpp | π-filter ~40 dB + LDO ~40 dB | ~1 µVpp |
| Bus disturbances @ 100 Hz–10 kHz (motors/PWM) | ~100 mVpp after buck loop | LDO PSRR ~75–80 dB | ~10–20 µVpp |
| LDO intrinsic noise | — | — | ~4–5 µVrms |
| **Total (RSS est.)** | | | **< 10 µVrms ✔** |

---

## 5. Isolation and Layout Rules

1. **Grounding:** single solid ground plane. Do not split AGND/DGND under the IMU. Instead, partition placement so digital return currents do not flow under the IMU or U1. Bring the LDO GND and IMU GND to a common local point.
2. **Kelvin sense:** run a dedicated OUT trace from U1 to the IMU's VDD pin and keep it short. TPS7A4700 has a SENSE pin, so route it to the load point if the trace is longer than about 10 mm.
3. **Buck isolation:** keep the switch node and inductor on the opposite board side or ≥25 mm from the IMU. Use a shielded inductor.
4. **NR/SS node:** this is high impedance and noise-critical. Keep it small, guard it with ground copper, and keep it away from switching edges.
5. **Mechanical:** if Cycle 5 used vibration isolation for the IMU, use flexible routing or a stiffening approach consistent with that. Avoid tall caps near the IMU, since they resonate. **[CONFIRM]**
6. **Thermal pad:** use a 3×3 to 4×4 via array to an inner ground plane.

---

## 6. Sequencing and Protection

- **Power-up order:** buck → U1 (analog) → U2 (VDDIO) after VDD settles. Alternatively, drive EN of U2 from U1's PG output, or use an MCU GPIO. Check the IMU datasheet for VDDIO ≤ VDD + 0.3 V constraints.
- **Soft-start:** the 10 µF NR/SS cap gives a slow, quiet ramp. Confirm the IMU's power-on reset tolerates this ramp rate.
- **Protection:** the TPS7A4700 has internal current limit and thermal shutdown. Add a TVS at the bus input and reverse-polarity protection upstream of the buck.

---

## 7. Verification Plan (Cycle 6 Exit Criteria)

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise density | LNA (e.g., 40–60 dB) + FFT analyzer, 10 Hz–100 kHz | < 10 µVrms integrated; no spurs > 1 µV |
| PSRR | Inject a sine on VIN (injection transformer), sweep 10 Hz–10 MHz | Meets the budget in Section 4 |
| Load transient | 10 → 150 mA step, 1 µs edge | < 5 mV deviation, no ringing |
| Ripple at IMU pins | Short-ground spring probe, 20 MHz BW limit off | < 50 µVpp at fsw |
| System test | IMU static Allan variance, motors on vs. off | Bias instability delta < 5% |
| Thermal | IR camera at max load | IMU gradient < 1 °C from the supply |

---

## 8. Open Items for Cycle 7

1. Confirm the IMU part number and its VDD/VDDIO current and sequencing requirements.
2. Confirm the input bus range, which drives the buck selection.
3. Decide whether a negative rail is needed (e.g., analog front-end). If so, pair with the **TPS7A3301** as the negative counterpart.
4. Evaluate a buck frequency sync to an IMU-unrelated clock to pin spurs away from the sensor bandwidth.
5. Perform microphonics characterization of the output MLCCs under the vibration profile.

---

If you share the Cycle 5 outputs (IMU part, bus voltage, board constraints), I can tighten the component values and produce a schematic-level BOM.