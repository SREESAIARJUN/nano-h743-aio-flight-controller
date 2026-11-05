# Low-Noise IMU Power Supply Architecture: TPS7A4700 (Design Cycle 5)

I don't have records of Cycles 1–4, so this design states its assumptions explicitly. Adjust the input bus, IMU part, and isolation requirement to match your baseline.

---

## 1. Design Assumptions & Requirements

| Parameter | Assumption / Target |
|---|---|
| Input bus | 12 V nominal (9–18 V), vehicle/robot platform with motor and switching noise |
| IMU class | Tactical/industrial MEMS (e.g., ADIS165xx, BMI088, ICM-42688 class) |
| IMU analog rail | 3.3 V, < 100 mA |
| IMU digital I/O rail | 3.3 V (or 1.8 V), < 20 mA |
| Rail noise target (10 Hz–100 kHz) | < 10 µVrms at IMU pins |
| Ripple at IMU pin | < 100 µVpp at the buck switching frequency |
| Isolation | Galvanic isolation optional (Option B), with functional noise isolation as baseline |
| Gyro bias sensitivity concern | Rail noise in the 0.1 Hz–1 kHz band (affects bias instability and ARW) |

---

## 2. Architecture Overview

```
                         STAGE 1                STAGE 2               STAGE 3              STAGE 4
 12 V BUS ──► [Input   ]──► [Sync Buck Pre-reg ]──► [π-Filter  ]──► [TPS7A4700   ]──► [Local    ]──► IMU AVDD (3.3 V)
 (noisy)     [Protect.]    [→ 4.3 V, 2.2 MHz   ]    [FB+LC     ]    [Ultra-low-  ]    [Decoupl.]
             [TVS, RPP]    [Spread-spectrum    ]    [~-40 dB @ ]    [noise LDO   ]
                           [off for fixed freq]    [ fsw       ]    [3.3 V out   ]
                                                         │
                                                         └──► [Low-noise LDO #2, e.g. TPS7A20] ──► IMU VDDIO (3.3/1.8 V)
```

**Design rationale:** the buck gives efficiency and a fixed headroom point. The passive filter attacks high-frequency ripple, where LDO PSRR rolls off. The TPS7A4700 then supplies the low-frequency and broadband noise rejection, plus its very low intrinsic noise. Analog and digital IMU rails are split so that SPI/I/O edge currents never modulate the sensing core's supply.

---

## 3. Stage-by-Stage Design

### Stage 1: Input Protection
- **TVS:** SMBJ24A (or matched to bus transient spec, e.g., ISO 7637 / ISO 16750 if automotive).
- **Reverse polarity:** ideal-diode controller (LM74700-Q1 + N-FET), which avoids a Schottky drop.
- **Bulk:** 47 µF electrolytic/polymer plus a 1 µF X7R input cap.
- **Common-mode choke** at the connector if long harnesses run near motors.

### Stage 2: Switching Pre-Regulator
- **Recommended:** TPS62913 (TI low-noise buck with integrated ferrite-bead-compensated filter support) or LMR33630.
- **Output set point:** **4.3 V**.
  - The TPS7A4700 dropout is ~307 mV at 1 A, but only tens of mV at < 100 mA.
  - Running ~1.0 V of headroom keeps the pass device out of the dropout region, where PSRR collapses.
  - LDO dissipation: (4.3 – 3.3) V × 0.1 A = **0.1 W**, which is negligible thermally.
- **Switching frequency:** 2.2 MHz, synchronized and fixed.
  - This places the ripple above the IMU's internal ADC/filter bandwidth and far from MEMS resonance (typically 20–30 kHz).
  - It also keeps the ripple well away from the 0.1–1 kHz gyro band.
  - **Disable spread-spectrum** unless EMI compliance requires it. Spreading smears energy into lower bands that are harder to filter.

### Stage 3: Inter-Stage π-Filter
- **Topology:** C1 (10 µF X7R) → ferrite bead (e.g., BLM31KN601, ~600 Ω @ 100 MHz, low DCR) or 2.2 µH shielded inductor → C2 (10 µF X7R + 22 µF polymer for damping).
- **Damping:** add a 1 Ω series R with the polymer cap, or rely on its ESR. An undamped LC peak can amplify noise at resonance.
- **Corner frequency:** with L = 2.2 µH and C = 20 µF, fc ≈ 24 kHz. This gives ~-80 dB (2nd order) attenuation at 2.2 MHz before the LDO.

### Stage 4: TPS7A4700 Configuration

**Key device characteristics:**
- Vin 3–36 V
- Output 1.4–20.5 V via ANY-OUT pins
- 1 A rated
- **~4.17 µVrms noise (10 Hz–100 kHz)**
- PSRR ~82 dB @ 100 Hz, ~55 dB @ 1 MHz

**Output voltage via ANY-OUT (no external resistors, so no resistor noise or tolerance stack-up):**

V_OUT = 1.4 V + Σ(grounded ANY-OUT pin weights)

For **3.3 V** = 1.4 + 1.6 + 0.2 + 0.1, connect these pins to GND:
- **1P6V**
- **0P2V**
- **0P1V**

Leave all others (0P4V, 0P8V, 3P2V, 6P4V) floating.

*(For a 5.0 V IMU variant: 1.4 + 3.2 + 0.4 = 5.0 V, so ground 3P2V and 0P4V.)*

**Component selection:**

| Pin / Function | Component | Notes |
|---|---|---|
| IN | 10 µF X7R 0805/1206, ≥ 25 V | Place within 2 mm of the pin |
| OUT | 10 µF X7R + 1 µF X7R | Meets stability requirement; use low-ESL placement |
| NR/SS | 1 µF – 10 µF X7R/C0G-class film | Larger values lower the noise floor and increase soft-start time. **Use 4.7 µF** in Cycle 5 to balance noise against start-up. |
| SENSE | Kelvin-route to the IMU AVDD pin | Compensates for trace IR drop |
| EN | RC from Vin, or MCU GPIO | Tie to the PG of the buck for sequencing |
| Thermal pad | GND, with a via array to the internal plane | Thermal and noise reference |

**Capacitor caution:** derate X7R for DC bias. A 10 µF 0805 at 3.3 V may give only ~6 µF effective. Use 1206 or 16 V+ rated parts to retain capacitance. Avoid Y5V/Z5U entirely.

### Stage 5: IMU Local Decoupling & Digital Rail
- **At IMU AVDD:** 100 nF C0G/X7R + 1 µF X7R, plus the IMU datasheet-specified values (these take precedence).
- **Optional:** a series ferrite bead (e.g., BLM15AX) between the LDO output and the IMU pin. This is only acceptable if SENSE is taken *before* the bead, or if the bead's DCR × current is negligible.
- **VDDIO rail:**
  - Option 1: a separate TPS7A20 (low-noise, small footprint) from the same 4.3 V node.
  - Option 2: a second TPS7A4700 if the IMU has no internal digital/analog separation.

---

## 4. Isolation Strategy

### Option A: Functional Noise Isolation (Baseline)
- A dedicated **analog ground island** under the IMU and LDO, joined to system ground at **one point** near the LDO GND pin.
- No split-plane gaps under SPI traces. Route SPI over continuous ground; if it must cross, stitch with a 100 nF cap at the crossing.
- 22–33 Ω series resistors on SCLK/MOSI/MISO to slow edges and reduce di/dt coupling into the rails.

### Option B: Galvanic Isolation
Use this when ground loops, high-current motor returns, or safety requirements demand it.

```
12 V ──► SN6505B push-pull driver ──► Isolation transformer (e.g., Würth 750315371)
         ──► Rectifier + LC filter ──► ~5 V isolated ──► π-filter ──► TPS7A4700 ──► 3.3 V IMU (isolated domain)

SPI / DRDY / RESET ◄──► ISO7741 (or ISO6741) digital isolator ◄──► Host MCU
```

- The SN6505B (~420 kHz) is lower in frequency than the buck. The π-filter corner must drop to **≤ 5 kHz**: use a larger L, or a second-order RC (e.g., 2.2 Ω + 100 µF) given the low IMU load.
- Size the isolated rail for ≥ 1.2 V of LDO headroom to absorb transformer regulation variation.
- Add an isolation-barrier stitching capacitor (~1–2 nF, Y-rated if needed) to manage common-mode EMI.

---

## 5. Noise Budget (Option A)

| Contributor | Estimated Level at IMU AVDD | Notes |
|---|---|---|
| TPS7A4700 intrinsic | ~4.2 µVrms | 10 Hz–100 kHz, with CNR ≥ 1 µF |
| Buck ripple residual | < 1 µVrms | Ripple ~10 mVpp × π-filter (-80 dB) × LDO PSRR (~55 dB @ 2.2 MHz rolled toward lower) |
| Low-frequency bus disturbance (100 Hz–1 kHz) | < 1 µVrms | 82 dB PSRR on a ~100 mV disturbance |
| Ground/IR coupling | Layout-dependent | Controlled via Kelvin SENSE and the single-point ground |
| **Total (RSS)** | **≈ 4.5–5 µVrms** | **Meets the < 10 µVrms target with ~2× margin** |

---

## 6. Power Sequencing & Protection

1. 12 V applied → buck soft-starts → buck PG asserts.
2. PG drives the TPS7A4700 EN. The NR/SS cap (4.7 µF) sets a controlled ramp, avoiding IMU POR glitches.
3. The VDDIO LDO is enabled **simultaneously or after** AVDD, per the IMU datasheet sequencing (usually VDD ≥ VDDIO or simultaneous).
4. The MCU holds IMU RESET low until both rails are valid plus ~10 ms.
5. TPS7A4700 internal current limit and thermal shutdown cover fault cases. Add a PTC fuse on the 12 V input.

---

## 7. PCB Layout Rules

- **Component order:** buck → π-filter → LDO → IMU in a linear flow, with no backtracking of noisy nodes past quiet ones.
- **Buck keep-out:** keep the buck's switch node and inductor ≥ 15 mm from the IMU. Orient the inductor field away from it, and use a shielded inductor.
- **Layer stack:** 4+ layers with a solid GND on L2 directly beneath the LDO and IMU.
- **NR/SS cap:** place it closest of all to its pin. This is a high-impedance node, so guard it with GND.
- **Thermal and mechanical:** place the IMU away from the LDO and any heat sources. Even 0.1 W of local gradient can shift gyro bias. Avoid board-flex zones and connectors that transmit vibration.
- **No routing under the IMU** except its own ground, per the vendor recommendation.

---

## 8. Cycle 5 Verification Plan

| Test | Method | Pass Criteria |
|---|---|---|
| Output noise | LNA (e.g., 40–60 dB gain, battery-powered) + spectrum analyzer, 10 Hz–100 kHz | < 10 µVrms integrated |
| Switching ripple | 50 Ω coax pigtail probe at the IMU pin, 20 MHz BW-limit off | < 100 µVpp at fsw |
| PSRR | Inject on 12 V via injection transformer (e.g., Picotest J2120A) + VNA, 10 Hz–10 MHz | ≥ 60 dB to 100 kHz (system level) |
| Load transient | IMU active/sleep mode toggle | < 2 mV deviation |
| IMU performance | Allan variance, ≥ 2 h static capture: board-supplied vs. lab battery reference | Bias instability and ARW within 5% of the battery reference |
| Motor-noise immunity | Run actuators while logging the IMU | No spectral spurs above the IMU noise floor |
| Thermal | IR camera at max ambient | LDO ΔT < 10 °C; no gradient across the IMU |

The **battery-reference Allan variance comparison** is the decisive test. It isolates the power-supply contribution to sensor performance directly, rather than inferring it from rail measurements.

---

## 9. Bill of Materials (Key Items)

| Function | Part | Qty |
|---|---|---|
| Ultra-low-noise LDO (AVDD) | TI TPS7A4700RGWR | 1 (2 if VDDIO also uses it) |
| VDDIO LDO | TI TPS7A2033 | 1 |
| Pre-regulator | TI TPS62913 / LMR33630 | 1 |
| Ideal diode | TI LM74700-Q1 | 1 |
| Ferrite bead | Murata BLM31KN601SN1 | 1–2 |
| Isolated supply (Option B) | TI SN