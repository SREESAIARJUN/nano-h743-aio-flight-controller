# PCB Motor Phase Tracks: Impedance & Skin-Effect AC Resistance at 48 kHz

**Note:** I don't have your project's "Cycle 15" parameters, such as trace width, length, stack-up or current. The worked numbers below use **assumed values**. Substitute your own; all formulas are given in general form.

## 0. Assumed Parameters

| Parameter | Symbol | Assumed value |
|---|---|---|
| Copper weight | 2 oz | t = 70 µm (0.070 mm) |
| Trace width | w | 5.0 mm |
| Trace length | l | 50 mm |
| Dielectric height to reference plane | h | 1.6 mm (FR-4) |
| Relative permittivity | εr | 4.5 |
| Copper resistivity (20 °C) | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| PWM frequency | f | 48 kHz |

## 1. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

At 20 °C:

$$\delta = \sqrt{\frac{1.72\times10^{-8}}{\pi \cdot 48\,000 \cdot 4\pi\times10^{-7}}} = \sqrt{9.08\times10^{-8}} \approx 0.301\ \text{mm}$$

At 100 °C, ρ ≈ 2.26 × 10⁻⁸ Ω·m, so **δ ≈ 0.345 mm**.

**Key ratio:** Δ = t/δ = 0.070 / 0.301 ≈ **0.23**

The copper is about 4× thinner than one skin depth. Current therefore penetrates the full thickness almost uniformly.

## 2. DC Resistance

$$R_{DC} = \frac{\rho\, l}{w\, t}\,,\qquad R_{DC}(T) = R_{DC,20}\,[1+\alpha(T-20)]$$

- Sheet resistance of 2 oz copper: ρ/t = **0.246 mΩ/□**.
- At 20 °C: R = 0.246 mΩ × (50/5 squares) = **2.46 mΩ**.
- At 100 °C: 2.46 × 1.314 = **3.23 mΩ**.

## 3. AC Resistance (Skin + Proximity)

Use Dowell's factor for a flat conductor:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta} + \frac{2(m^2-1)}{3}\,\Delta\,\frac{\sinh\Delta - \sin\Delta}{\cosh\Delta + \cos\Delta}$$

Here m is the number of stacked current-carrying layers, which accounts for proximity effect. For Δ < 1 this simplifies to:

$$F_R \approx 1 + \frac{5m^2 - 1}{45}\,\Delta^4$$

Results:

- **Single layer (m = 1):** Δ⁴ = 0.0028, so F_R ≈ 1 + 6.2 × 10⁻⁵. The effect is negligible.
- **Same trace paralleled on 4 layers (m = 4):** F_R ≈ 1 + (79/45)(0.0028) ≈ **1.005**.

### Harmonics of the PWM Ripple

The phase current's fundamental is the motor electrical frequency (low). The 48 kHz content is the ripple, so check its harmonics:

| Harmonic | f | δ (20 °C) | Δ | F_R (m=1) |
|---|---|---|---|---|
| 1st | 48 kHz | 301 µm | 0.23 | ≈1.000 |
| 5th | 240 kHz | 135 µm | 0.52 | ≈1.002 |
| 10th | 480 kHz | 95 µm | 0.74 | ≈1.007 |
| 20th | 960 kHz | 67 µm | 1.04 | ≈1.07 |

**Conclusion:** Skin effect across the thickness is not a design driver for 2 oz copper at 48 kHz.

Edge and plane-proximity crowding across the width (w ≫ δ) can add a few percent. The current concentrates under the trace, toward the return plane. If you need to quantify this, use a 2D field solver.

**Design R:** R_AC ≈ 1.01–1.05 × R_DC ≈ **3.3–3.4 mΩ at 100 °C**.

### Loss Calculation

$$P = I_{RMS,fund}^2 R_{DC}(T) + \sum_n I_{ripple,n}^2\, R_{DC}(T)\,F_R(n)$$

The DC/fundamental term dominates.

## 4. Characteristic Impedance

Use Hammerstad–Wheeler for a microstrip with w/h > 1. Here w/h = 3.125.

Effective permittivity:

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 2.75 + 1.75(0.4545) = 3.545$$

Characteristic impedance:

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\!\left(\frac{w}{h}+1.444\right)\right]} = \frac{377}{1.883 \times 5.531} \approx \mathbf{36\ \Omega}$$

The IPC-2141 formula is not valid at w/h > 2, so don't use it here.

### Relevance Check

The wavelength at 48 kHz in this medium is:

$$\lambda = \frac{c}{f\sqrt{\varepsilon_{eff}}} \approx 3.3\ \text{km}$$

Even the switching-edge knee frequency is short of transmission-line territory. With t_r = 20 ns, f_knee ≈ 0.35/t_r = 17.5 MHz, giving λ/10 ≈ 0.9 m. A 50 mm trace is therefore **electrically short**, and Z₀ doesn't matter. Model the trace as **lumped R + L**.

## 5. Inductance (the Parameter That Actually Matters)

**Isolated flat strip (partial self-inductance):**

$$L \approx 0.2\,l\left[\ln\frac{2l}{w+t} + 0.5 + 0.2235\frac{w+t}{l}\right]\ \text{nH (l, w, t in mm)}$$

$$L \approx 0.2(50)[2.982 + 0.5 + 0.023] \approx \mathbf{35\ nH}$$

**Over a solid return plane (loop inductance, w ≫ h approximation):**

$$L \approx \mu_0\frac{h\,l}{w} \approx \mathbf{20\ nH}$$

This drops to about 1.6 nH with a 0.127 mm prepreg to the plane.

**Reactance at 48 kHz:** X_L = 2π(48 k)(35 nH) ≈ **10.6 mΩ**. This is about 3× R_AC. Inductance, not skin effect, dominates the trace's AC impedance.

**Switching transients:** V = L·di/dt. For example, 35 nH × (50 A / 50 ns) ≈ **35 V** of ringing/overshoot potential. This is why the commutation loop (bulk cap → high-side FET → low-side FET) must be minimized. Phase-output trace inductance itself is mostly in series with the motor's own inductance and is benign.

## 6. Summary Table

| Quantity | Value (assumed geometry) |
|---|---|
| Skin depth @ 48 kHz | 0.30 mm (20 °C) / 0.35 mm (100 °C) |
| t/δ | 0.23 |
| R_DC | 2.46 mΩ (20 °C) / 3.23 mΩ (100 °C) |
| R_AC/R_DC | ≈1.00 (single layer), ≤1.05 with proximity |
| Z₀ (microstrip) | ≈36 Ω (not relevant: electrically short) |
| L (isolated / over plane) | ≈35 nH / ≈20 nH |
| X_L @ 48 kHz | ≈10.6 mΩ |

## 7. Design Recommendations

1. **Size width for I²R and temperature rise**, not skin effect. Use IPC-2152 charts for allowable ΔT.
2. **Increase effective cross-section** rather than going thicker. Parallel the trace on multiple layers with via stitching, or use 3 oz copper; at 48 kHz this still has no skin-effect penalty.
3. **Minimize loop inductance** with a tight return plane and short commutation loops.
4. **Validate** with a 2D solver (e.g., Saturn PCB Toolkit, Ansys Q3D, or FEMM) if edge crowding matters at your current levels.

If you share the actual Cycle 15 width, length, stack-up, phase current and ripple, I can redo the numbers precisely.