# PCB Phase-Track Calculations: 2 oz Copper at 48 kHz

I don't have your "Cycle 24" design data, so this uses representative values. Replace the inputs in Section 0 with your actual geometry and stack-up.

## 0. Input Parameters (assumed)

| Symbol | Meaning | Value |
|---|---|---|
| t | Cu thickness (2 oz, finished) | 70 µm (0.070 mm) |
| w | Trace width | 5.0 mm |
| ℓ | Trace length | 50 mm |
| h | Dielectric height to reference plane | 0.20 mm |
| εr | FR-4 relative permittivity | 4.3 |
| ρ₂₀ | Cu resistivity at 20 °C | 1.72×10⁻⁸ Ω·m |
| α | Cu temperature coefficient | 0.00393 /°C |
| T_op | Operating copper temperature | 100 °C |
| f_sw | PWM frequency | 48 kHz |
| t_r | Switch-node edge rise time | 20 ns |
| I_rms | Phase current | 30 A |

## 1. DC Resistance

**Resistivity at operating temperature:**

ρ(T) = ρ₂₀ · [1 + α(T − 20)]

ρ(100 °C) = 1.72×10⁻⁸ × (1 + 0.00393 × 80) = **2.26×10⁻⁸ Ω·m**

**Trace resistance:**

R_DC = ρ·ℓ / (w·t)

- At 20 °C: R_DC = (1.72×10⁻⁸ × 0.05) / (0.005 × 70×10⁻⁶) = **2.46 mΩ**
- At 100 °C: **3.23 mΩ**

## 2. Skin Depth

δ = √( ρ / (π · f · µ₀) ), where µ₀ = 4π×10⁻⁷ H/m.

| Frequency | δ at 20 °C | δ at 100 °C | t/δ (100 °C) |
|---|---|---|---|
| 48 kHz (fundamental) | 301 µm | 345 µm | 0.20 |
| 144 kHz (3rd harmonic) | 174 µm | 199 µm | 0.35 |
| 240 kHz (5th harmonic) | 135 µm | 154 µm | 0.45 |
| 480 kHz (10th harmonic) | 95 µm | 109 µm | 0.64 |

**Key result:** δ is much larger than t (70 µm) through at least the 10th harmonic. Current is essentially uniform through the copper thickness.

## 3. AC Resistance Factor

**Through-thickness skin effect (Dowell form, single layer):**

F_R = R_AC / R_DC = Δ · [sinh 2Δ + sin 2Δ] / [cosh 2Δ − cos 2Δ], with Δ = t/δ

For Δ much less than 1, this simplifies to F_R ≈ 1 + Δ⁴/45.

- At 48 kHz: Δ = 0.20, so F_R ≈ 1 + 3.6×10⁻⁵, which is negligible.
- At 240 kHz: Δ = 0.45, so F_R ≈ 1.001.

**Lateral (width-wise) current crowding** is the effect that actually matters for wide traces. It is governed by the ratio of the trace's inductive to resistive impedance, not by skin depth. A practical estimate:

- If a return plane lies directly beneath: current concentrates under the trace toward the return path. Effective R_AC rises about 5–20% for w/h ≫ 1.
- If no plane is present: current crowds to the edges. Use field simulation, or apply a margin of about 1.1–1.3× for w much greater than δ.

**Design value:** R_AC(48 kHz, 100 °C) ≈ 1.1 × 3.23 ≈ **3.6 mΩ** (conservative).

## 4. Power Dissipation

Separate the low-frequency phase current from the PWM ripple:

P = I_fund,rms² · R_DC + Σₙ I_ripple,n² · R_AC,n

- Main term: 30² × 3.23 mΩ ≈ **2.9 W**
- Ripple term: typically 5–20% of the main term ripple amplitude, so its I² contribution is under 1–4% extra.

Cross-check the temperature rise against IPC-2152. About 2.9 W in 50 mm of 5 mm trace is high, so consider widening the trace, using both outer layers, or adding solder or busbar reinforcement.

## 5. Characteristic Impedance (Microstrip over Plane)

Because w/h = 25, the IPC-2141 formula is invalid here. Use Hammerstad–Wheeler.

**Effective permittivity:**

ε_eff = (εr + 1)/2 + (εr − 1)/2 · (1 + 12h/w)^(−½)
= 2.65 + 1.65 × (1.48)^(−½) = **4.01**

**Characteristic impedance (valid for w/h ≥ 1):**

Z₀ = 120π / { √ε_eff · [ w/h + 1.393 + 0.667 · ln(w/h + 1.444) ] }
= 376.99 / (2.0015 × 28.58) ≈ **6.6 Ω**

**Per-unit-length parameters:**

- L′ = Z₀√ε_eff / c ≈ 44 nH/m, giving **2.2 nH** over 50 mm
- C′ = √ε_eff / (Z₀ c) ≈ 1.0 nF/m, giving **50 pF** over 50 mm

## 6. Is Transmission-Line Behaviour Relevant?

- Wavelength at 48 kHz: λ = c / (f√ε_eff) ≈ 3,100 m.
- Edge knee frequency: f_knee = 0.35/t_r = 17.5 MHz, giving λ ≈ 8.6 m. The λ/10 threshold is 0.86 m, far longer than the 50 mm trace.

**Conclusion:** Treat the track as a **lumped RLC element**. Z₀ matters only for deriving L′ and C′:

- **Reactance at 48 kHz:** X_L = 2π f L ≈ 0.66 mΩ. This is about 20% of R, so it is not negligible for current-sense accuracy.
- **Switching ringing:** 2.2 nH multiplied by di/dt contributes to overshoot. For example, at 1 A/ns, V = 2.2 V.
- **Switch-node capacitance:** About 50 pF of trace-to-plane capacitance. At 50 V/ns, this injects about 2.5 A of common-mode current spike, which matters for EMI and gate-drive noise.

## 7. Summary

| Parameter | Value |
|---|---|
| R_DC (100 °C) | 3.23 mΩ |
| Skin depth at 48 kHz | 345 µm (much greater than 70 µm Cu) |
| R_AC/R_DC (skin effect) | ≈ 1.00 |
| R_AC design value (with crowding margin) | ≈ 3.6 mΩ |
| Z₀ | ≈ 6.6 Ω (lumped regime) |
| L / C (50 mm) | 2.2 nH / 50 pF |
| Conduction loss at 30 A | ≈ 2.9 W per phase track |

**Takeaway:** At 48 kHz, skin effect is not a design driver for 2 oz copper. Focus on DC resistance at operating temperature, thermal rise (IPC-2152), loop inductance, and switch-node capacitance.

If you share the actual Cycle 24 stack-up, current, and track dimensions, I can rerun the numbers.