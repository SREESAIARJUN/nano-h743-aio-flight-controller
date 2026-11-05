# PCB Phase-Track Calculations at 48 kHz (2 oz Copper)

I don't have the specific Cycle 21 design data, such as track geometry, stack-up, or current levels. So below is the full formulation with a worked example using **assumed values**. Replace them with your actual numbers.

**Assumed example geometry**

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | 2 oz | t = 70 µm (0.070 mm) |
| Track width | w | 10 mm |
| Track length | l | 50 mm |
| Dielectric height to reference plane | h | 0.2 mm (FR-4, εr ≈ 4.4) |
| Copper resistivity at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Operating temperature | T | 100 °C |
| RMS phase current | I | 30 A |

---

## 1. Skin Depth at 48 kHz

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

At 20 °C:

$$\delta = \sqrt{\frac{1.72\times10^{-8}}{\pi \cdot 48\,000 \cdot 4\pi\times10^{-7}}} = \sqrt{9.08\times10^{-8}} \approx 0.301\ \text{mm}$$

At 100 °C, use ρ₁₀₀ = ρ₂₀[1 + α(80)] = 2.26 × 10⁻⁸ Ω·m, which gives **δ ≈ 0.345 mm**.

**Key ratio:**

$$\xi = \frac{t}{\delta} = \frac{0.070}{0.301} \approx 0.23$$

Because ξ ≪ 1, the 2 oz foil is much thinner than one skin depth at 48 kHz.

---

## 2. DC Resistance

$$R_{DC} = \frac{\rho\, l}{w\, t}$$

At 20 °C:

$$R_{DC,20} = \frac{1.72\times10^{-8} \times 0.05}{0.010 \times 70\times10^{-6}} = 1.23\ \text{m}\Omega$$

At 100 °C:

$$R_{DC,100} = R_{DC,20}\,[1+\alpha(T-20)] = 1.23 \times 1.314 = 1.62\ \text{m}\Omega$$

---

## 3. AC Resistance (Skin Effect)

Use the Dowell foil-conductor factor for a single layer:

$$F_R = \frac{R_{AC}}{R_{DC}} = \xi\,\frac{\sinh 2\xi + \sin 2\xi}{\cosh 2\xi - \cos 2\xi}$$

For ξ ≪ 1, this reduces to:

$$F_R \approx 1 + \frac{4}{45}\xi^4$$

At 48 kHz:

$$F_R \approx 1 + 0.0889 \times (0.23)^4 = 1 + 0.00025 \approx \mathbf{1.0003}$$

$$R_{AC,48k} \approx 1.62\ \text{m}\Omega \quad (\text{at } 100\ °\text{C})$$

**Conclusion:** Through-thickness skin effect is negligible at the 48 kHz fundamental for 2 oz copper.

### Caveat: Edge Crowding and PWM Harmonics

1. **Width-direction crowding.** When a track sits over a return plane, current concentrates under the edges and on the plane-facing side. Practical correction:

$$R_{AC} \approx \frac{\rho\, l}{w \cdot \min(t,\delta)} \cdot k_{edge}, \quad k_{edge}\approx 1.0\text{–}1.2$$

2. **Switching harmonics.** If 48 kHz is the PWM frequency, the phase-track current contains ripple harmonics. Evaluate each harmonic separately:

$$P_{loss} = \sum_n I_n^2\, R_{DC}\, F_R(\xi_n), \qquad \xi_n = \frac{t}{\delta}\sqrt{n}$$

   - At around 1 MHz, δ ≈ 65 µm ≈ t, so F_R rises to roughly 1.1–1.3.
   - Harmonic ripple current amplitudes are usually small relative to the fundamental, so their total contribution is typically under 5% of the loss.

---

## 4. Track Inductance and Impedance

### Electrical Length Check

$$\lambda = \frac{c}{f\sqrt{\varepsilon_{eff}}} \approx \frac{3\times10^8}{48\,000 \times 1.9} \approx 3.3\ \text{km} \gg l$$

The track is electrically short, so treat it as a **lumped R–L element**. Characteristic impedance Z₀ does not matter at 48 kHz.

### Case A: Isolated Track (No Adjacent Return Plane)

Use the Grover/Ruehli partial self-inductance, with l, w, t in mm:

$$L = 0.2\,l\left[\ln\frac{2l}{w+t} + 0.5 + 0.2235\,\frac{w+t}{l}\right]\ \text{nH}$$

$$L = 0.2(50)\left[\ln(9.93) + 0.5 + 0.045\right] \approx \mathbf{28\ nH}$$

### Case B: Track Over a Solid Return Plane (Wide Microstrip, w ≫ h)

$$L \approx \mu_0\,\frac{h}{w}\,l = 4\pi\times10^{-7}\cdot\frac{0.2}{10}\cdot 0.05 \approx \mathbf{1.3\ nH}$$

Add roughly 20–30% for fringing.

For reference only, the Z₀ formulation (Hammerstad, w/h > 1) is:

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h}+1.393+0.667\ln\left(\frac{w}{h}+1.444\right)\right]}$$

With w/h = 50, this gives Z₀ ≈ 3–4 Ω.

### Total Impedance at 48 kHz

$$Z = R_{AC} + j\omega L$$

| Configuration | R_AC (100 °C) | X_L = 2πfL | \|Z\| |
|---|---|---|---|
| Isolated (28 nH) | 1.62 mΩ | 8.4 mΩ | **8.6 mΩ** |
| Over plane (~1.6 nH) | 1.62 mΩ | 0.48 mΩ | **1.7 mΩ** |

**Key insight:** At 48 kHz, inductance dominates the impedance unless the phase tracks have a tightly coupled return path. This matters for:
- di/dt overshoot at the switch node: V = L·di/dt. At 28 nH and 1 A/ns, the overshoot is 28 V.
- Matching impedance between phases, which affects current-sense balance.

---

## 5. Power Dissipation and Thermal Check

$$P = I_{RMS}^2 \, R_{AC} = 30^2 \times 1.62\ \text{m}\Omega \approx \mathbf{1.46\ W}\ \text{per track}$$

Verify the temperature rise with **IPC-2152**. Use the chart for 2 oz copper at 10 mm width, then apply corrections for board thickness, copper planes, and airflow. Iterate, because R depends on T.

---

## 6. Summary Formula Set

| Quantity | Formula |
|---|---|
| Skin depth | δ = √(ρ / πfμ₀) |
| DC resistance | R_DC = ρ(T)·l / (w·t) |
| Temperature correction | ρ(T) = ρ₂₀[1 + α(T−20)] |
| AC factor | F_R = ξ(sinh2ξ + sin2ξ)/(cosh2ξ − cos2ξ), ξ = t/δ |
| Self-inductance (isolated) | L = 0.2l[ln(2l/(w+t)) + 0.5 + 0.2235(w+t)/l] nH |
| Inductance over plane | L ≈ μ₀·h·l / w |
| Impedance | Z = R_DC·F_R + j2πfL |
| Loss | P = Σ Iₙ² R_DC F_R(ξₙ) |

---

## Design Recommendations

1. **Thicker copper gives no skin-effect penalty at 48 kHz.** 3–4 oz copper (δ ≈ 0.3 mm) remains effective, so heavier copper reduces R directly.
2. **Route the phase outputs over a solid plane** to cut inductance by more than 10×.
3. **Use parallel layers with via stitching** instead of a single wide track. This lowers both R and L.
4. **Keep the three phase tracks geometrically matched** so their impedance stays balanced.

If you share the actual Cycle 21 parameters (track dimensions, stack-up, phase current, ripple spectrum, ambient temperature), I can recompute these with your values.