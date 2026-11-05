# PCB Motor-Phase Trace Analysis at 48 kHz (2 oz Cu)

I don't have your Cycle 19 parameters, such as trace width, length, stack-up, or current. The formulas below are general, and the worked example uses placeholder values that you can substitute.

**Placeholder geometry:** w = 10 mm, L = 50 mm, h = 1.6 mm (FR-4, εr = 4.5), t = 70 µm (2 oz).

---

## 1. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

| Condition | ρ (Ω·m) | δ at 48 kHz |
|---|---|---|
| 20 °C | 1.72×10⁻⁸ | **301 µm** |
| 100 °C | 2.25×10⁻⁸ | **344 µm** |

A handy shortcut for copper at 20 °C is δ ≈ 66 mm / √f(Hz).

**Key result:** t/δ ≈ 0.23. The 70 µm copper is much thinner than the skin depth, so through-thickness skin effect is negligible at 48 kHz.

---

## 2. DC Resistance

$$R_{DC} = \frac{\rho L}{w\,t}\,[1+\alpha(T-20)],\quad \alpha = 0.00393/°C$$

$$R_{DC,20°C} = \frac{1.72\times10^{-8}\cdot 0.05}{0.01\cdot 70\times10^{-6}} = 1.23\ \text{m}\Omega$$

At 100 °C this rises to about **1.61 mΩ**.

---

## 3. AC Resistance (Thickness Skin Effect)

Use the Dowell form with Δ = t/δ:

$$F_R = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta} \;\approx\; 1 + \frac{\Delta^4}{45}\quad(\Delta<1)$$

| Harmonic | f | δ | Δ | F_R |
|---|---|---|---|---|
| Fundamental (PWM) | 48 kHz | 301 µm | 0.23 | 1.0001 |
| 3rd | 144 kHz | 174 µm | 0.40 | 1.0006 |
| 5th | 240 kHz | 135 µm | 0.52 | 1.002 |
| 10th | 480 kHz | 95 µm | 0.74 | 1.007 |

The ripple-current harmonics add effectively nothing to loss through thickness skin effect.

---

## 4. Width-Direction Current Crowding (Dominant AC Effect)

Width-direction crowding is the effect that actually matters here, because w ≫ δ. Compare the per-length resistance with the inductive reactance:

- r = R_DC / L = 0.0246 Ω/m
- ωL' ≈ 2π · 48k · (0.14–0.7 µH/m) ≈ 0.04–0.21 Ω/m

Since ωL' > r, the current distribution is inductance-controlled rather than resistance-controlled. How this plays out depends on the layout:

- **Isolated strip (return far away):** current crowds to the edges, following J(x) ∝ 1/√(1−(2x/w)²). Expect R_AC/R_DC ≈ 1.2–1.5.
- **Return on an adjacent layer, with w ≫ h:** current spreads nearly uniformly across the face facing the return (proximity effect). Expect R_AC/R_DC ≈ 1.0–1.1. This is the recommended layout.
- **Parallel phase traces close together:** the proximity effect pushes current toward or away from neighbours. Simulate this case (FEMM, Q3D, or Sonnet).

**Total loss formula:**
$$P = I_{RMS,fund}^2 R_{DC}(T) + \sum_n I_{n,ripple}^2\, R_{DC}(T)\, F_{R,n}\, k_{crowd,n}$$

At typical motor fundamentals (below 1 kHz), the first term dominates. The 48 kHz ripple term usually contributes less than 5% of total loss.

---

## 5. Characteristic Impedance (Microstrip, Hammerstad)

For w/h ≥ 1:

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 3.77$$

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\,\left[\frac{w}{h}+1.393+0.667\ln\left(\frac{w}{h}+1.444\right)\right]} = \frac{377}{1.943\times9.00} \approx \mathbf{21.6\ \Omega}$$

**Is Z₀ relevant?** The wavelength at 48 kHz is

$$\lambda = \frac{c}{f\sqrt{\varepsilon_{eff}}} \approx 3.2\ \text{km}$$

The trace is about 10⁻⁵ λ long, so it is electrically short. Treat it as a lumped R–L element; transmission-line effects do not apply at 48 kHz.

**Lumped inductance:**
$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = 0.14\ \mu\text{H/m}\;\Rightarrow\; L_{50mm} \approx 7\ \text{nH}$$
$$X_L(48\text{ kHz}) = 2.1\ \text{m}\Omega$$

This is comparable to R_DC, which confirms the inductance-controlled distribution described in Section 4.

---

## 6. Switching-Edge Check (Where Skin Effect Does Matter)

The PWM frequency is 48 kHz, but the switching edges contain much higher frequencies:

$$f_{knee} \approx \frac{0.35}{t_r}\quad (t_r = 20\text{ ns} \Rightarrow 17.5\text{ MHz})$$

- δ(17.5 MHz) ≈ 16 µm, giving t/δ ≈ 4.4 and F_R ≈ 2.5–3.
- These edge harmonics carry little energy, so the extra loss is minor. They do matter for EMI and voltage overshoot: V = L·di/dt ≈ 7 nH × (e.g. 50 A / 20 ns) ≈ 17.5 V.

---

## 7. Summary

| Parameter | Value (placeholder geometry) |
|---|---|
| Skin depth at 48 kHz | 301 µm (20 °C) / 344 µm (100 °C) |
| R_DC | 1.23 mΩ (20 °C) / 1.61 mΩ (100 °C) |
| F_R (thickness) | ≈ 1.000 |
| Width-crowding factor | 1.0–1.5 (layout-dependent) |
| Z₀ | ≈ 21.6 Ω (not relevant at 48 kHz) |
| Loop L | ≈ 7 nH (X_L ≈ 2.1 mΩ) |

**Design recommendations:**
1. Size the traces by DC resistance and temperature rise (IPC-2152), since 2 oz copper is not skin-limited at 48 kHz.
2. Route the return or DC-link plane directly beneath the phase traces. This minimises loop L and edge crowding.
3. Paralleling layers works well because each 70 µm layer is much thinner than δ. Moving to heavier copper (3–4 oz) also remains effective.

If you share your actual Cycle 19 parameters (width, length, stack-up, RMS and ripple current, ambient temperature), I can rerun the numbers with those values.