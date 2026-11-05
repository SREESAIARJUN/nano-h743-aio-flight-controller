# 2 oz Motor Phase Tracks at 48 kHz: Impedance and Skin-Effect AC Resistance (Cycle 1)

I don't have your Cycle 1 board geometry, so this sets up the general formulation and works one example with placeholder dimensions. Replace the inputs in the table below with your stack-up and layout values.

---

## 1. Input Parameters

| Symbol | Meaning | Example value |
|---|---|---|
| t | Copper thickness, 2 oz | 70 µm (≈2.7 mil, finished) |
| w | Trace width | 8 mm |
| l | Trace length | 40 mm |
| h | Dielectric height to return plane | 0.2 mm |
| ρ₂₀ | Copper resistivity at 20 °C | 1.72 × 10⁻⁸ Ω·m |
| α | Temperature coefficient | 0.00393 /°C |
| T | Operating copper temperature | 100 °C |
| f_sw | PWM frequency | 48 kHz |
| µ₀ | Permeability of free space | 4π × 10⁻⁷ H/m |

**Temperature-corrected resistivity:**

ρ(T) = ρ₂₀ · [1 + α(T − 20)] = 1.72e-8 × 1.314 = **2.26 × 10⁻⁸ Ω·m** at 100 °C

---

## 2. DC Resistance

R_dc = ρ·l / (w·t)

R_dc = 2.26e-8 × 0.04 / (0.008 × 70e-6) = **1.62 mΩ** at 100 °C (1.23 mΩ at 20 °C)

---

## 3. Skin Depth at 48 kHz

δ = √( ρ / (π·f·µ₀) )

| Temperature | δ at 48 kHz |
|---|---|
| 20 °C | **301 µm** |
| 100 °C | **345 µm** |

Harmonics scale as δ ∝ 1/√f:
- 96 kHz: 213 µm
- 144 kHz: 174 µm
- 240 kHz: 135 µm (all at 20 °C)

**Key result:** δ ≫ t/2 (35 µm). Even at the 5th harmonic, the copper is thinner than one skin depth. Classical through-thickness skin effect is therefore negligible.

---

## 4. AC Resistance Factor (Through-Thickness)

Use the Dowell 1-D conductor model, with x = t/δ and m = 1 (single layer):

F_R = R_ac / R_dc = x · [sinh(2x) + sin(2x)] / [cosh(2x) − cos(2x)]

For small x:

F_R ≈ 1 + (4/45)·x⁴

| Frequency | x (20 °C) | F_R |
|---|---|---|
| 48 kHz | 0.233 | **1.0003** |
| 144 kHz | 0.402 | 1.002 |
| 240 kHz | 0.519 | 1.006 |

---

## 5. Lateral Current Crowding (Width-Direction)

Lateral crowding is the effect that actually matters for wide power traces. It is governed by trace width, not thickness.

**Onset criterion:**

k = ω·µ₀·σ·w·t

- If k ≪ 1, current is uniform across the width.
- If k ≳ 1, current crowds toward the edges, or toward the side nearest the return path.

**Example:** k = 2.20 × 10⁷ m⁻² × (8 mm × 70 µm) ≈ **12**

So expect a moderate rise in R_ac at 48 kHz:
- **Planning estimate:** F_R ≈ 1.1–1.3. This is a rough range, not a derived value.
- **Verification:** Confirm with a 2-D field solver (FEMM, Ansys Q2D, Sonnet), or with an AC 4-wire measurement on the Cycle 1 board.

**Layout notes:**
- A solid return plane directly beneath the trace pulls current to the facing surface and reduces edge crowding.
- Splitting very wide pours into paralleled traces on several layers helps more than widening a single trace.

---

## 6. Trace Impedance Z = R_ac + jωL

**(a) Over a solid return plane** (wide-strip, parallel-plate approximation, valid for w ≫ h):

L ≈ µ₀·h·l / w = 4π×10⁻⁷ × 0.2 mm × 40 mm / 8 mm ≈ **1.26 nH**

X_L = 2π·48 kHz·1.26 nH ≈ **0.38 mΩ**

**(b) No adjacent return plane** (partial self-inductance; l, w, t in cm, result in nH):

L = 2l · [ ln(2l/(w+t)) + 0.5 + 0.2235·(w+t)/l ] ≈ **22.7 nH**

X_L ≈ **6.8 mΩ**

**Impedance summary at 48 kHz, 100 °C:**

| Configuration | R_ac | X_L | \|Z\| |
|---|---|---|---|
| Over plane | ~1.8–2.1 mΩ | 0.38 mΩ | ~2.0 mΩ |
| No plane | ~1.8–2.1 mΩ | 6.8 mΩ | ~7.1 mΩ |

**Context:**
- The motor winding inductance (typically µH–mH) dwarfs either trace value, so trace inductance does not affect current control.
- Trace inductance does matter for dv/dt ringing and EMI near the switch node.
- If you need a microstrip Z₀ (for EMI or transmission-line checks rather than loss), use Hammerstad–Jensen. The IPC-2141 formula is invalid for w/h ≫ 1.

---

## 7. Loss Formulation

The phase current is mostly the low-frequency fundamental (f_e, motor electrical frequency) plus PWM ripple at f_sw and its harmonics:

P = I_fund,rms² · R_dc(T) + Σₙ I_n,rms² · R_dc(T) · F_R(n·f_sw)

**Example inputs:** 30 A rms fundamental; 3 A pk-pk triangular ripple, so I_ripple,rms = I_pp/(2√3) = 0.87 A.

| Component | Loss |
|---|---|
| Fundamental | 30² × 1.62 mΩ = **1.46 W** |
| 48 kHz ripple | 0.87² × 1.62 mΩ × 1.3 ≈ **1.6 mW** |

**Conclusion:** At 48 kHz, AC/skin-effect losses are below 0.2 % of the total. The design is governed by DC resistance and thermal rise, which you should check against IPC-2152. Note that 1.46 W in a 40 mm trace needs a ΔT check, plus attention to via current sharing and plane spreading.

---

## 8. Cycle 1 Checklist

1. Insert actual w, l, h, and layer count per phase. For paralleled layers, sum their conductances.
2. Use the **finished** copper thickness from the fab (plating can change it ±10 %).
3. Confirm which ripple frequency appears in the phase current. With center-aligned PWM, check for components at 2·f_sw.
4. Run a 2-D solver for the lateral F_R and replace the 1.1–1.3 placeholder.
5. Run IPC-2152 thermal checks at worst-case ambient, then iterate with ρ(T).
6. Plan a Cycle 1 measurement: 4-wire DC resistance plus an LCR or impedance-analyzer sweep at 48–250 kHz to validate the model.

If you share your stack-up and the phase-track dimensions, I can rerun these numbers for your actual board.