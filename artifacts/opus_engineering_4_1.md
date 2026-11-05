# PCB Trace Impedance and AC Resistance at 48 kHz: 2 oz Motor Phase Tracks

I don't have your Cycle 4 geometry, so I've built the formulation around a representative trace. Substitute your own values for width, length, dielectric height and temperature.

## 1. Assumed Parameters (replace with your values)

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | – | 2 oz/ft² |
| Copper thickness | t | 70 µm (0.070 mm) |
| Trace width | w | 10 mm |
| Trace length | l | 50 mm |
| Dielectric to return plane | h | 1.5 mm (2-layer FR-4) |
| Relative permittivity | εr | 4.5 |
| Resistivity, Cu at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Operating temperature | T | 100 °C (worst case) |
| Frequency | f | 48 kHz (PWM fundamental) |

## 2. Skin Depth

**Formula:**
δ = √( ρ / (π · f · μ₀) ), where μ₀ = 4π × 10⁻⁷ H/m

**At 20 °C:**
δ = √( 1.72×10⁻⁸ / (π · 48 000 · 4π×10⁻⁷) ) = √(9.08×10⁻⁸) ≈ **0.301 mm (301 µm)**

**At 100 °C:**
ρ₁₀₀ = ρ₂₀ · [1 + α(T − 20)] = 1.72×10⁻⁸ × 1.314 = 2.26×10⁻⁸ Ω·m
δ₁₀₀ ≈ **0.345 mm (345 µm)**

**Key ratio:**
Δ = t / δ = 0.070 / 0.301 ≈ **0.23**

Because t ≪ δ, the full copper cross-section conducts at 48 kHz.

## 3. DC Resistance

**Formula:**
R_DC = ρ · l / (w · t)

**At 20 °C:**
R_DC = (1.72×10⁻⁸ × 0.05) / (0.010 × 70×10⁻⁶) = **1.23 mΩ**

**At 100 °C:**
R_DC = 1.23 × 1.314 = **1.62 mΩ**

## 4. AC Resistance (Dowell Model, Single Layer)

**Formula:**
F_R = R_AC / R_DC = Δ · (sinh 2Δ + sin 2Δ) / (cosh 2Δ − cos 2Δ)

**Small-Δ approximation (Δ < 1):**
F_R ≈ 1 + (4/45) · Δ⁴

**Calculation:**
F_R ≈ 1 + 0.0889 × (0.23)⁴ = 1 + 0.0889 × 0.0029 ≈ **1.0003**

**Result:**
- R_AC(48 kHz, 20 °C) ≈ 1.23 mΩ
- R_AC(48 kHz, 100 °C) ≈ 1.62 mΩ

**Conclusion:** skin effect is negligible at the 48 kHz fundamental for 2 oz copper.

### Harmonic check

| Frequency | δ (20 °C) | Δ = t/δ | F_R |
|---|---|---|---|
| 48 kHz (fundamental) | 301 µm | 0.23 | ≈1.000 |
| 240 kHz (5th) | 135 µm | 0.52 | ≈1.006 |
| 1 MHz | 66 µm | 1.06 | ≈1.10 |
| 7 MHz (edge knee, tr ≈ 50 ns) | 25 µm | 2.8 | ≈2.8 |

Skin effect only becomes significant in the switching-edge spectrum. That matters for EMI and ringing, not for conduction loss.

## 5. Trace Impedance at 48 kHz

The wavelength at 48 kHz is about 3 km in FR-4 (λ = c / (f√εr_eff)). The trace is therefore electrically tiny, and characteristic impedance Z₀ is not meaningful here. Treat the trace as a lumped R–L element.

**(a) Isolated flat-strip partial inductance (Ruehli/Rosa), with l, w, t in cm:**
L (nH) = 2l · [ ln(2l/(w+t)) + 0.5 + 0.2235(w+t)/l ]
= 2(5) · [ ln(10/1.007) + 0.5 + 0.045 ] = 10 × 2.841 ≈ **28 nH**

**(b) Over a tight return plane (wide-strip approximation):**
L ≈ μ₀ · h · l / w = 4π×10⁻⁷ × 1.5×10⁻³ × 0.05 / 0.01 ≈ **9.4 nH**

**Reactance:**
X_L = 2πfL

| Case | L | X_L at 48 kHz | Z = √(R² + X²) at 100 °C |
|---|---|---|---|
| Isolated (no close return) | 28 nH | 8.5 mΩ | ≈ 8.7 mΩ |
| Over return plane | 9.4 nH | 2.8 mΩ | ≈ 3.3 mΩ |

**Observation:** at 48 kHz the inductive reactance exceeds the resistance. Phase-to-phase and loop geometry dominate the trace impedance more than copper loss does.

**For completeness (edge-rate / EMI analysis only):** microstrip Z₀ via Hammerstad for w/h > 1:

Z₀ = 120π / [ √εr_eff · (w/h + 1.393 + 0.667 ln(w/h + 1.444)) ]

With w/h = 6.67 and εr_eff ≈ 3.9, Z₀ ≈ **21 Ω**.

## 6. Loss Calculation

**Formula:**
P = I_RMS,fund² · R_DC(T) + Σ I_h² · R_AC(f_h)

**Example:** 30 A_RMS phase current, 100 °C:
P ≈ 30² × 1.62 mΩ ≈ **1.46 W per trace**

The ripple-current contribution at 48 kHz is typically below 1 % of this.

## 7. Design Takeaways

1. **No skin derating is needed** for 2 oz copper at 48 kHz. Moving to 3–4 oz still gives full utilisation, since δ ≫ t/2.
2. **Size traces for DC thermal rise** per IPC-2152, using the 100 °C resistance.
3. **Minimise loop inductance.** Route phase pairs and the DC-link return tightly coupled. This matters more than the AC resistance does.
4. **Account for proximity effect and current crowding** near vias, pads and bends at the edge harmonics. Stitch parallel layers with via arrays.
5. **Use your actual Cycle 4 values.** Substitute your w, l, h, current and T into Sections 3–6. Every relation above scales linearly or logarithmically, so updating the numbers is straightforward.