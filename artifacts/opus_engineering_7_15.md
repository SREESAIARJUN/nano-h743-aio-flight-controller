# Low-Noise IMU Power Architecture: TPS7A4700 Isolated Rail (Cycle 7)

> **Context note:** I don't have your Cycle 1–6 documents, so this cycle is written as a self-contained baseline. The assumptions in §1 are where to reconcile it with your previous decisions, such as IMU part number, bus voltage, and isolation requirements.

---

## 1. Design Assumptions & Requirements

| Parameter | Assumed Value | Rationale |
|---|---|---|
| Input bus | 12 V nominal (9–32 V transient) | Typical vehicle/UAV/robotics bus |
| IMU | MEMS tactical-grade (e.g., ADIS16470/ADIS1649x class) | 3.0–3.6 V supply, ~40–100 mA |
| IMU rail | 3.3 V ±2 %, ≤ 200 mA budget | Includes margin for heater/aux |
| Rail noise target | < 10 µV<sub>RMS</sub> (10 Hz–100 kHz) | Keeps supply contribution well below IMU intrinsic noise |
| Ripple at IMU pins | < 50 µV<sub>pp</sub> at switcher fundamental | Avoids aliasing into gyro/accel bandwidth |
| Isolation | Functional (noise) isolation as baseline, galvanic as option B | See §6 |
| Thermal | LDO must not create a thermal gradient across the IMU | Gyro bias tempco sensitivity |

---

## 2. Architecture Overview

```
 12 V BUS
   │
   ▼
┌──────────────────────┐
│ Stage 0: Protection  │  Reverse-polarity FET, TVS (SMBJ33A), eFuse (TPS2660x)
└──────────┬───────────┘
           ▼
┌──────────────────────┐
│ Stage 1: EMI Filter  │  CM choke + π-filter (LC + damping RC)
└──────────┬───────────┘
           ▼
┌──────────────────────┐
│ Stage 2: Pre-Reg     │  Sync buck → 4.3 V (e.g., TPS62913 / LMR33630)
│ (efficiency stage)   │  Fixed freq ≥ 2 MHz, out of IMU band
└──────────┬───────────┘
           ▼
┌──────────────────────┐
│ Stage 3: Post-Filter │  Ferrite bead + 2nd-order LC, damped
└──────────┬───────────┘
           ▼
┌──────────────────────┐
│ Stage 4: TPS7A4700   │  3.3 V, 4.17 µVrms, high PSRR
│ Ultra-low-noise LDO  │
└──────────┬───────────┘
           ▼
┌──────────────────────┐
│ Stage 5: Local Decap │  Ferrite + bulk/HF caps at IMU VDD pins
└──────────┬───────────┘
           ▼
         IMU
```

**Why a cascade instead of LDO-from-12 V directly?**

- **Direct 12 V → 3.3 V:** P = 8.7 V × 0.1 A ≈ 0.87 W. With θ<sub>JA</sub> of roughly 35–40 °C/W on the RGW QFN, that is a **+30 °C hotspot** near the IMU. This is unacceptable for bias stability.
- **4.3 V → 3.3 V:** P = 1.0 V × 0.1 A = **0.1 W**, giving a temperature rise of about 4 °C. This is negligible.
- The switcher handles efficiency. The LDO handles noise and PSRR.

---

## 3. TPS7A4700 Configuration

### 3.1 Key Device Parameters (verify against current datasheet revision)
- V<sub>IN</sub>: 3 V – 36 V
- V<sub>OUT</sub>: 1.4 V – 20.5 V via **ANY-OUT™** pin strapping, or external resistors
- I<sub>OUT</sub>: up to 1 A
- Output noise: **4.17 µV<sub>RMS</sub>** (10 Hz–100 kHz)
- PSRR: ~82 dB at 100 Hz, and > 55 dB out to ~1 MHz (load and headroom dependent)
- Dropout: ~307 mV at 1 A, which is much lower at IMU currents
- Package: 5 × 5 mm VQFN-20 (RGW) with thermal pad

### 3.2 Output Voltage Programming: ANY-OUT (no external resistors)

V<sub>OUT</sub> = 1.4 V + Σ(grounded pin weights)

| Pin | Weight | 3.3 V setting |
|---|---|---|
| 6P4V2 | 6.4 V | open |
| 3P2V | 3.2 V | open |
| 1P6V | 1.6 V | **GND** |
| 0P8V | 0.8 V | open |
| 0P4V | 0.4 V | open |
| 0P2V | 0.2 V | **GND** |
| 0P1V | 0.1 V | **GND** |

1.4 + 1.6 + 0.2 + 0.1 = **3.30 V**

Using ANY-OUT avoids external feedback resistors. That removes their thermal noise, their tempco mismatch, and a source of drift. This is the preferred approach for an IMU rail.

### 3.3 Passive Components

| Ref | Function | Value | Type / Notes |
|---|---|---|---|
| C<sub>IN</sub> | LDO input | 10 µF + 100 nF | X7R, ≥ 16 V, placed at IN pins |
| C<sub>OUT</sub> | LDO output | 10 µF (min) + 100 nF | X7R, ≥ 10 V; check DC-bias derating, since effective C must stay ≥ datasheet minimum |
| C<sub>NR</sub> | NR/SS pin | 1 µF – 10 µF | Larger values give lower 1/f noise but a longer soft-start; use low-leakage X7R or film |
| C<sub>BYP</sub> | Feed-forward/bypass | 10 nF – 10 µF per datasheet | Improves noise and PSRR at mid-band |
| R<sub>EN</sub> / C<sub>EN</sub> | Enable sequencing | 100 kΩ / 100 nF | Or drive from a PG/sequencer |

**Soft-start caution:** a large C<sub>NR</sub> gives a slow ramp. Confirm the ramp is compatible with the IMU's power-on reset requirements. Some IMUs specify a maximum supply rise time.

### 3.4 Headroom for PSRR
- PSRR degrades sharply near dropout.
- Target **V<sub>IN</sub> − V<sub>OUT</sub> ≥ 1.0 V**, which is why the pre-regulator is set to 4.3 V.
- Validate PSRR at the actual load (50–100 mA) and headroom. The headline datasheet curves are taken under specific conditions.

---

## 4. Noise and Ripple Budget

| Source | At LDO input | Attenuation | At IMU |
|---|---|---|---|
| Buck ripple @ 2.2 MHz | ~10 mV<sub>pp</sub> | LC post-filter ~40 dB, then LDO ~45 dB, then local FB/decap ~10 dB | **< 10 µV<sub>pp</sub>** |
| Buck switching spikes (HF) | ~50 mV | Post-filter plus local π handles most of it; LDO PSRR is weak above 10 MHz | Depends on layout, so measure |
| LDO intrinsic noise | — | — | ~4–5 µV<sub>RMS</sub> |
| Thermal noise, ferrite/caps | — | — | negligible |
| **Total estimate** | | | **≈ 5–8 µV<sub>RMS</sub>**, meeting the < 10 µV target |

**Key insight:** the LDO does not protect you from HF switching edges (> 5–10 MHz). The **passive post-filter and layout** are what handle those. The LDO's job is the 10 Hz–1 MHz band, which is where IMU sensitivity matters most.

### Post-filter design (Stage 3)
- L = 2.2 µH (shielded, low DCR), C = 22 µF X7R, giving f<sub>c</sub> ≈ 23 kHz
- **Damping:** add a series RC (1 Ω + 47 µF) across C, or use a lossy ferrite. This prevents a resonant peak that could amplify a load-transient harmonic.
- Check the Q of the filter combined with the LDO input impedance. LDOs can present negative input impedance at some frequencies.

---

## 5. Pre-Regulator (Stage 2) Selection Guidance

| Requirement | Reason |
|---|---|
| f<sub>sw</sub> ≥ 2 MHz, **fixed** | Keeps the fundamental far above IMU bandwidth and inside the region where passive filtering is effective |
| **Disable spread-spectrum** (or verify) | Spread spectrum smears energy downward and can create in-band products |
| Forced PWM (no PFM/skip mode) | PFM at light load produces low-frequency, load-dependent ripple. This is the most common IMU noise bug. |
| Sync input (optional) | Lock to the system clock to avoid beat frequencies with other converters |

Example candidates: **TPS62913** (low-noise buck with integrated ferrite-filter compensation) or **LMR33630** with FPWM. Choose based on Cycle 6 BOM decisions.

---

## 6. Isolation Strategy

### Option A: Functional (noise) isolation, which is the baseline
- Dedicated IMU power island fed only by the TPS7A4700. No other loads share it.
- **Split ground pour** with a single-point tie (star) at the LDO GND/thermal pad, under the IMU island boundary.
- Digital lines to the IMU (SPI, DR, RST) go through **series 22–33 Ω resistors** to slow edges and limit return current across the boundary.
- A guard ring around the IMU island stitches to the IMU GND only.

### Option B: Galvanic isolation (for high-noise motor/actuator systems)
```
12 V ─► SN6505B push-pull + transformer ─► rectifier ─► LC ─► TPS7A4700 ─► IMU
                                                                    │
                          SPI ◄──► ISO7741 / ISO6741 digital isolator ◄──► MCU
```
- The SN6505B runs at ~420 kHz (or is spread-spectrum capable; disable that per §5). Size the post-filter accordingly.
- With an isolated secondary, the TPS7A4700 input can be set to ~5 V, giving 1.7 V headroom.
- **Trade-off:** isolator propagation delay (~10–20 ns) reduces the maximum SPI clock. Account for it in IMU burst-read timing.
- **Use when** the IMU shares a chassis with BLDC/ESC drives, or when ground bounce exceeds ~10s of mV.

---

## 7. PCB Layout Rules

1. **Placement:** put the LDO 10–25 mm from the IMU. That is close enough for short traces and far enough to avoid thermal coupling. Never place the LDO directly under or beside the IMU footprint.
2. **Thermal pad:** solder it to a GND pour with a via array. Route the heat **away** from the IMU, using a thermal relief slot if needed.
3. **NR/SS and BYP caps:** place them tight to the pins. These are the highest-impedance, most noise-sensitive nodes, so use a Kelvin connection to GND at the LDO.
4. **Switcher loop:** keep the hot loop minimal and on the opposite board region from the IMU. Do not route switch-node copper under the IMU island on any layer.
5. **Layer stack:** use a solid GND plane under the entire IMU power chain. No split under the LDO-to-IMU trace.
6. **IMU mechanical:** if the IMU is board-mounted, consider isolation slots for vibration and thermal reasons. Route the power trace across a bridge, not a large pour.

---

## 8. Verification Plan (Cycle 7 Exit Criteria)

| Test | Method | Pass Criterion |
|---|---|---|
| Output noise | LNA (e.g., 40–60 dB) + spectrum analyzer, 10 Hz–100 kHz | ≤ 10 µV<sub>RMS</sub> integrated |
| PSRR | Bode analyzer + injection transformer at LDO input | ≥ 60 dB at 1 kHz–100 kHz at actual load |
| Ripple at IMU pins | Scope, 1:1 probe, spring ground, 20 MHz BW, then full BW | < 50 µV<sub>pp</sub> at f<sub>sw</sub> |
| Load transient | IMU burst-read current step | ΔV < 5 mV, no ringing |
| Thermal | IR camera, 1 h soak | IMU-case ΔT from LDO < 1 °C |
| **System-level** | Allan variance with the IMU static, comparing the LDO rail against a battery/linear lab supply | No measurable degradation in ARW/VRW or bias instability |
| Light-load ripple | Verify the buck stays in FPWM at minimum load | No LF ripple components |

The **Allan variance comparison** is the decisive test. Rail-noise numbers are proxies. The IMU's own output is the ground truth.

---

## 9. Risk Register (Cycle 7)

| Risk | Likelihood | Mitigation |
|---|---|---|
| Ceramic C<sub>OUT</sub> DC-bias derating causes instability | Medium | Use