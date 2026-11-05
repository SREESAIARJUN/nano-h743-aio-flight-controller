# PCB Phase-Track Impedance and AC Resistance at 48 kHz (2 oz Cu)

I don't have your Cycle 25 design data, so this uses a **representative geometry**. Every formula is written out so you can substitute your actual trace width, length, stack-up and currents.

## 1. Assumptions (replace with your values)

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | — | 2 oz → **t = 70 µm** |
| Trace width | w | 10 mm |
| Trace length | ℓ | 50 mm |
| Dielectric height to return plane | h | 1.6 mm (FR-4) |
| Relative permittivity | εr | 4.5 |
| PWM / ripple frequency | f | 48 kHz |
| Resistivity at 20 °C | ρ₂₀ | 1.724×10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Operating temperature | T | 100 °C (design case) |

The temperature-corrected resistivity is:

ρ(T) = ρ₂₀·[1 + α(T − 20)] → **ρ₁₀₀ = 2.266×10⁻⁸ Ω·m**

## 2. DC Resistance

R_DC = ρ·ℓ / (w·t)

- At 20 °C: R_DC = (1.724×10⁻⁸ × 0.05) / (0.01 × 70×10⁻⁶) = **1.23 mΩ**
- At 100 °C: **1.62 mΩ**

## 3. Skin Depth at 48 kHz

δ = √( ρ / (π·f·μ₀) ), where μ₀ = 4π×10⁻⁷ H/m

- At 20 °C: δ = √(1.724×10⁻⁸ / 0.1895) = **0.302 mm**
- At 100 °C: δ = **0.346 mm**

**Key ratio:** Δ = t/δ = 0.070 / 0.302 ≈ **0.23**

Because Δ ≪ 1 (the foil is about 4× thinner than one skin depth), current fills the full copper thickness at 48 kHz.

## 4. AC Resistance Factor (Dowell, single foil layer)

F_R = R_AC / R_DC = Δ · [sinh 2Δ + sin 2Δ] / [cosh 2Δ − cos 2Δ]

For small Δ this reduces to:

F_R ≈ 1 + Δ⁴/45 = 1 + (0.23)⁴/45 ≈ **1.00006**

**Result:** R_AC(48 kHz) ≈ R_DC. Thickness-direction skin effect is negligible.

### Harmonic check

δ scales as 1/√f. PWM edges carry energy at harmonics, so check where skin effect starts to matter:

| Frequency | δ (20 °C) | t/δ | F_R (approx.) |
|---|---|---|---|
| 48 kHz | 302 µm | 0.23 | 1.00 |
| 240 kHz (5th) | 135 µm | 0.52 | 1.002 |
| 1 MHz | 66 µm | 1.06 | ~1.03 |
| 10 MHz (ringing) | 21 µm | 3.3 | ~3.3 (≈ t/δ) |

Skin effect only becomes significant for switching-edge ringing (MHz range). That is an EMI and snubber concern, not a conduction-loss concern.

### Lateral current crowding (the real AC effect)

Over a solid return plane, AC current concentrates on the underside of the trace and toward its edges, because the current follows the path of minimum inductance. For wide traces (w/h > 3), a conservative estimate is:

- F_R,lateral ≈ 1.05–1.15 at 48 kHz

To confirm this for your layout, use a 2D field solver (e.g., FEMM, or Ansys Q2D/Saturn PCB).

## 5. Characteristic Impedance (Microstrip, Hammerstad)

For w/h = 6.25 (> 1):

**Effective permittivity:**

ε_eff = (εr+1)/2 + (εr−1)/2 · (1 + 12h/w)^(−½)
= 2.75 + 1.75 × 0.585 = **3.77**

**Characteristic impedance:**

Z₀ = 120π / ( √ε_eff · [ w/h + 1.393 + 0.667·ln(w/h + 1.444) ] )
= 376.99 / (1.943 × 9.00) ≈ **21.6 Ω**

### Is Z₀ relevant at 48 kHz?

- λ in board = c / (f·√ε_eff) ≈ **3.2 km**
- Critical length for edges (t_r ≈ 20 ns): ℓ_crit ≈ t_r·v/6 ≈ **0.5 m**

The trace is electrically short. You should therefore model it as a **lumped R + L**, not as a transmission line.

## 6. Lumped Inductance and Total Impedance

L' = Z₀·√ε_eff / c = 21.6 × 1.943 / 3×10⁸ ≈ **140 nH/m**, so L ≈ **7.0 nH** for 50 mm.

X_L(48 kHz) = 2π·f·L = **2.11 mΩ**

Z(48 kHz) = R_AC + jX_L = 1.62 + j2.11 mΩ → **|Z| ≈ 2.66 mΩ** (at 100 °C)

**Design insight:** the reactance is comparable to the resistance. Trace inductance governs ripple current sharing between parallel paths and the voltage overshoot at switching (V = L·di/dt). Skin effect does not.

## 7. Loss Calculation Template

P = I²_fund,rms · R_DC(T) + I²_ripple,rms · R_AC(48k) + Σₙ I²ₙ · R_AC(n·48k)

**Example:** 20 A_rms fundamental and 4 A_rms ripple, at 100 °C:

P ≈ 20² × 1.62 mΩ + 4² × 1.62 mΩ × 1.1 = 0.648 + 0.029 ≈ **0.68 W per phase track**

Verify the resulting temperature rise against **IPC-2152** for your actual board, plane, and airflow conditions.

## 8. Summary for Cycle 25

| Quantity | Value |
|---|---|
| Skin depth @ 48 kHz | 0.30–0.35 mm (≫ 70 µm foil) |
| R_AC / R_DC (thickness) | ≈ 1.00 |
| R_AC / R_DC (lateral crowding) | ~1.05–1.15 (verify with field solver) |
| Z₀ (microstrip) | ~21.6 Ω (not design-relevant at 48 kHz) |
| L per mm | ~0.14 nH/mm |
| Dominant concerns | DC loss and temperature; loop inductance for overshoot |

## 9. Recommendations

- **Lower resistance:** go wider, or stack layers with via stitching. Thicker copper gives little benefit for skin effect at this frequency.
- **Lower inductance:** route each phase over a solid return plane and keep the hot loop tight.
- **Better accuracy:** send me your actual w, ℓ, h, εr and current profile, and I'll recompute with them.