# Low-Noise IMU Power Supply Architecture: TPS7A4700 Post-Regulation (Cycle 30)

## 1. Design Objectives

| Parameter | Target | Rationale |
|---|---|---|
| IMU rail noise (10 Hz to 100 kHz) | < 10 µVrms | Keeps supply-induced bias instability below the gyro/accel noise floor |
| Ripple at the IMU pins | < 50 µVpp | Prevents spurious tones from aliasing into the sensor bandwidth |
| Rail cross-coupling | > 60 dB isolation from digital and motor rails | Avoids load-dependent bias shifts |
| Load | 3.3 V at ≤ 150 mA | Covers MEMS IMU (ADIS16470/16495-class) plus local reference/ADC |
| Efficiency | Pre-regulated, LDO headroom ≤ 1 V | Thermal stability of the IMU's temperature compensation |

## 2. Top-Level Architecture

```
 VIN (9–36 V bus)
   │
   ├─[TVS + reverse-polarity FET + bulk 47 µF]
   │
 ┌─▼──────────────────────────┐
 │ Stage 1: Sync Buck          │  LMR33630 (or equivalent)
 │ VOUT = 4.3 V, fSW = 2.1 MHz │  fixed frequency, spread spectrum OFF
 │ (synced to system clock)    │  (keeps spurs at a known location)
 └─┬──────────────────────────┘
   │
 ┌─▼──────────────────────────┐
 │ Stage 2: Passive π-filter   │  22 µF ─ FB (BLM31KN, 1 kΩ@100MHz, low DCR) ─ 22 µF
 │ + 1 Ω/10 µF damping branch  │  fc ≈ 50 kHz, damped Q < 1
 └─┬──────────────────────────┘
   │
 ┌─▼──────────────────────────┐
 │ Stage 3: TPS7A4700 (U1)     │  VOUT = 3.3 V via ANY-OUT pins
 │ "IMU_AVDD" — analog/sensor  │  4.17 µVrms, PSRR 82 dB @100 Hz
 └─┬──────────────────────────┘
   │
 ┌─▼──────────────────────────┐
 │ Stage 4: Point-of-load      │  Ferrite + 10 µF + 100 nF at IMU VDD pins
 │ decoupling, IMU star ground │
 └────────────────────────────┘

 Optional: a parallel TPS7A4700 (U2) from the same π-filter output produces
 "IMU_DVDD/VDDIO" (1.8 V or 3.3 V), so digital I/O current pulses never
 modulate the sensor rail.
```

**Optional galvanic isolation (Cycle 30 option B):** if the IMU must float relative to the vehicle or motor ground, replace Stage 1 with an SN6505B push-pull driver, a 1:1.5 transformer and a Schottky rectifier, regulating to about 4.5 V. The rest of the chain is unchanged. Bridge data through an ISO7741 (SPI) or ISO7721 (DR/SYNC lines).

## 3. TPS7A4700 Configuration

### 3.1 Output Voltage (ANY-OUT, no external resistors)

VOUT = 1.4 V (reference) + the sum of the ANY-OUT pins that are grounded.

| Target | Pins to GND | Calculation |
|---|---|---|
| **3.3 V (IMU_AVDD)** | 1P6V, 200mV, 100mV | 1.4 + 1.6 + 0.2 + 0.1 = 3.3 V |
| 1.8 V (VDDIO option) | 400mV | 1.4 + 0.4 = 1.8 V |
| 5.0 V (if IMU needs it) | 3P2V, 400mV | 1.4 + 3.2 + 0.4 = 5.0 V |

Leave unused ANY-OUT pins floating. Do not route them near switching nodes. ANY-OUT preserves the low noise figure, whereas an external divider would add resistor noise and gain.

### 3.2 Passive Components

| Pin | Component | Notes |
|---|---|---|
| IN | 10 µF X7R 25 V + 100 nF | Placed at the pin |
| OUT | 10 µF X7R (≥ 4.7 µF effective after DC bias) + 100 nF | The device is stable with ceramics |
| NR/SS | 10 µF X7R | Sets the noise-filter corner low; soft-start ≈ tens of ms. Smaller values give faster start-up but more 10 Hz–1 kHz noise |
| EN | RC from sequencer, or tied to IN via 100 kΩ | Sequence AVDD on before DVDD |
| PG | 10 kΩ pull-up to DVDD | Gates the IMU reset/MCU init |
| Thermal pad | Solid via array to the analog ground plane | Also acts as a heat sink |

### 3.3 Headroom and Dropout

- Dropout is about 307 mV at 1 A. At 150 mA it is about 60–80 mV.
- Chosen headroom is VIN_LDO − VOUT = 4.3 − 3.3 = **1.0 V**. This keeps the LDO well out of its dropout region, where PSRR collapses, while still dissipating little power.

## 4. Noise and Ripple Budget

**Switching ripple path (2.1 MHz):**

| Stage | Attenuation | Residual |
|---|---|---|
| Buck output ripple | — | ~10 mVpp |
| π-filter (2nd order, fc ≈ 50 kHz → ~2 decades) | ~ −60 dB (real-world −45 dB with parasitics) | ~56 µVpp |
| TPS7A4700 PSRR @ 2 MHz | ~ −40 dB | ~0.6 µVpp |
| POL ferrite + MLCC | ~ −20 dB | **< 0.1 µVpp** |

**Broadband noise (10 Hz–100 kHz):**

| Source | Contribution |
|---|---|
| TPS7A4700 intrinsic | 4.17 µVrms |
| Feed-through of buck noise | < 0.5 µVrms |
| POL filter resistive noise | Negligible |
| **Total (RSS)** | **≈ 4.2 µVrms**, meeting the < 10 µVrms target with about 2.4× margin |

**Low-frequency (< 100 Hz) supply sensitivity:** with 82 dB PSRR, a 100 mV bus disturbance at 100 Hz appears as about 8 µV at the IMU. This is below a typical IMU supply-rejection-induced bias error of about 0.1°/hr/V.

## 5. Thermal Check

- P_LDO = (4.3 − 3.3) V × 0.15 A = **0.15 W**.
- With θJA ≈ 35 °C/W (VQFN-20 on a 4-layer board), the temperature rise is about 5 °C.
- Without pre-regulation (12 V → 3.3 V directly), P_LDO would be 1.3 W and the rise about 46 °C. That heat would gradient-load the IMU's temperature compensation, which is why Stage 1 exists.
- Place U1 at least 15 mm from the IMU, or behind a thermal slot, to avoid a local thermal gradient across the sensor.

## 6. Grounding and Layout Rules

1. **Ground strategy:** use one continuous ground plane, partitioned by placement rather than by cuts. Keep the buck's hot loop (VIN cap, switch node, GND) compact and on the far side of the board from the IMU.
2. **Star return:** the LDO GND, NR cap and OUT cap return to a single via cluster tied to the IMU ground pad.
3. **Kelvin sensing:** TPS7A47 senses internally. Keep the OUT trace from U1 to the IMU short and wide (< 20 mΩ), so load steps do not create IR drops.
4. **Keep-outs:**
   - No digital or SPI traces under the NR/SS node or the ANY-OUT pins.
   - Keep the switch node at least 10 mm from U1 and the IMU.
5. **Shielding:** use a stitched via fence around the IMU and LDO island. A can is optional if radiated noise from the 2.1 MHz loop is measurable.
6. **Mechanical noise:** use soft-termination MLCCs, or X7R in a 0805 or larger case, at the IMU. This reduces piezoelectric (singing-capacitor) coupling into the MEMS structure.

## 7. Sequencing and Protection

```
VIN good → Buck EN → (PGOOD_buck) → U1 EN (AVDD) → PG_U1 → U2 EN (DVDD) → PG_U2 → MCU releases IMU RST
```

- Add a back-to-back Schottky (BAT54S) from DVDD to AVDD. This prevents I/O back-powering during power-down.
- The TPS7A4700 provides current limit and thermal shutdown. An external eFuse is needed only on the VIN bus.

## 8. Cycle 30 Changes from Prior Iteration

| Item | Previous | Cycle 30 | Reason |
|---|---|---|---|
| Buck spread spectrum | ON | **OFF + SYNC** | Spread spectrum smeared spurs into the IMU's 1–2 kHz internal sampling band; a fixed, synchronized tone is filterable |
| π-filter damping | None | **1 Ω + 10 µF RC** | Undamped peaking at ~50 kHz exceeded the LDO's weaker PSRR region |
| NR/SS cap | 1 µF | **10 µF** | Reduced 1/f noise below 100 Hz, which matters for bias instability |
| AVDD/DVDD | Shared LDO | **Split U1/U2** | SPI burst current produced ~30 µV steps on the sensor rail |
| Headroom | 2.0 V | **1.0 V** | Lower dissipation and thermal gradient while staying out of dropout |

## 9. Verification Plan

1. **Output noise:** measure with a 40 dB LNA and FFT analyzer (10 Hz–100 kHz), shorted input as baseline. Pass criterion: < 10 µVrms.
2. **PSRR:** inject a sweep on the LDO input via an injection transformer (e.g., Picotest J2120A) and use a VNA from 10 Hz to 10 MHz. Confirm more than 60 dB end-to-end at fSW.
3. **Load transient:** apply 0 → 150 mA, 1 µs edges. Pass criterion: < 5 mV deviation and settling within 50 µs.
4. **System-level:** collect IMU Allan variance with the bench linear supply versus the board supply. Bias instability and ARW should differ by < 5%.
5. **Thermal:** IR-image at maximum ambient. Pass criterion: < 2 °C gradient across the IMU footprint.
6. **EMI coupling:** toggle motor/PWM loads while logging the IMU. There should be no spectral lines above the sensor noise floor.

## 10. Bill of Materials (Power Section)

| Ref | Part | Qty |
|---|---|---|
| U_BUCK | LMR33630ADDA (or TPS62913, which has an integrated ferrite-filter option) | 1 |
| U1, U2 | TPS7A4700RGWR | 2 |
| FB1–FB3 | BLM31KN601SN1 | 3 |
| C (NR) | 10 µF X7R 0805 | 2 |
| C (IN/OUT) | 10 µF X7R 1206 25 V | 6 |
| C (HF) | 100 nF X7R 0402 | 8 |
| R_damp | 1 Ω 0603 | 1 |
| Option B (isolation) | SN6505B, 750315240 transformer, ISO7741 | 1 set |

**Summary:** a synchronized buck pre-regulator, a damped π-filter and dual TPS7A4700 LDOs (split analog/digital, ANY-OUT set to 3.3 V) give about 4.2 µVrms on the IMU rail. Switching residue is below 1 µV, dissipation is minimal and thermal gradients are controlled. Option B adds galvanic isolation without changing the low-noise post-regulation stages.