# PCB Motor Phase Trace Analysis: 2 oz Cu at 48 kHz

I don't have your "Cycle 31" design data (trace geometry, stackup, phase current), so this gives the general formulation and a worked example with stated assumptions. Substitute your values where marked.

---

## 1. Baseline Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Copper thickness (2 oz) | t | 70 µm (≈68.6 µm nominal; ~60 µm after etch is common) |
| Resistivity @ 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Permeability | µ₀ | 4π × 10⁻⁷ H/m |
| Frequency | f | 48 kHz |
| **Example trace** | w × l | 10 mm × 50 mm |
| Dielectric (FR-4) | εᵣ, h | 4.4, 1.6 mm to return plane |
| **Example current** | I_rms | 30 A |

Temperature correction for resistivity:

$$\rho(T) = \rho_{20}\,[1 + \alpha(T - 20)] \quad\Rightarrow\quad \rho_{100} \approx 2.26\times10^{-8}\ \Omega\cdot m$$

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

| Condition | δ |
|---|---|
| 20 °C, 48 kHz | **0.301 mm** |
| 100 °C, 48 kHz | 0.345 mm |
| 20 °C, 480 kHz (10th harmonic) | 0.095 mm |

**Key comparison:** t = 70 µm, while 2δ ≈ 600 µm. The current can penetrate from both faces, so the copper is about 8.6× thinner than the full skin-effect penetration.

---

## 3. AC Resistance

### DC resistance

$$R_{DC} = \frac{\rho\, l}{w\, t}$$

- @ 20 °C: 1.72e-8 × 0.05 / (0.01 × 70e-6) = **1.23 mΩ**
- @ 100 °C: **1.61 mΩ**

### Skin-effect factor (foil/Dowell form, single isolated layer)

Let Δ = t/δ. Then

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}\;\approx\;1 + \frac{4}{45}\Delta^4 \quad (\Delta \ll 1)$$

At Δ = 0.070/0.301 = 0.233:

$$F_R \approx 1 + 0.089 \times 0.00295 \approx \mathbf{1.0003}$$

**Result:** Through-thickness skin effect is negligible at 48 kHz:

$$R_{AC} \approx 1.61\ \text{m}\Omega \ (100\,°\text{C})$$

### What actually raises R_AC in wide power traces

1. **Edge current crowding (width direction).** For w ≫ δ over a plane, current concentrates at the trace edges and on the side facing the return. A practical engineering estimate for wide microstrip at this frequency is F_R ≈ 1.1–1.5. Field-solve this (e.g., FEMM, Q3D, or Saturn PCB as a first pass).
2. **Proximity effect.** Adjacent phase traces carrying opposing currents push current toward their facing edges. Keep the spacing at least 2–3× t, or interleave with the return.
3. **PWM ripple harmonics.** If 48 kHz is the switching frequency, the ripple at 2f, 3f, … sees higher F_R. Sum the losses per harmonic:

$$P = I_{DC/fund}^2 R_{DC} + \sum_n I_n^2\, R_{DC}\, F_R(n f)$$

### Conduction loss (example)

$$P = I_{rms}^2 R_{AC} = 30^2 \times 1.61\text{ m}\Omega \approx \mathbf{1.45\ W}$$

Cross-check the temperature rise against IPC-2152 for your copper area.

---

## 4. Inductance and Reactance (dominant at 48 kHz)

**Isolated strip (no nearby return plane)**, with l, w, t in cm and L in nH:

$$L = 2l\left[\ln\frac{2l}{w+t} + 0.5 + 0.2235\frac{w+t}{l}\right]$$

$$L = 10\,[2.296 + 0.5 + 0.045] \approx \mathbf{28\ nH}$$

**Over a return plane (parallel-plate approximation):**

$$L \approx \frac{\mu_0 h l}{w} = \frac{4\pi\times10^{-7}\times1.6\times10^{-3}\times0.05}{0.01} \approx \mathbf{10\ nH}$$

**Reactance:**

$$X_L = 2\pi f L$$

| Configuration | X_L |
|---|---|
| Isolated strip | 8.6 mΩ |
| Over plane | 3.0 mΩ |

**Insight:** At 48 kHz the trace impedance is inductance-dominated. X_L is 2–5× R_AC, so |Z| ≈ 3.4–8.7 mΩ. Layout loop area matters more than copper weight for di/dt overshoot and EMI.

---

## 5. Characteristic Impedance (for completeness)

Use the Hammerstad microstrip formula for w/h ≥ 1:

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 3.70$$

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]} \approx \mathbf{21.8\ \Omega}$$

The wavelength at 48 kHz is about 3 km in FR-4, so the trace is electrically tiny. Treat it as a **lumped R–L element**. Z₀ only matters for switching-edge harmonics in the tens of MHz and above (ringing and EMI).

---

## 6. Summary

| Quantity | Value (example) |
|---|---|
| Skin depth @ 48 kHz | 0.30 mm |
| t/δ | 0.23 |
| Skin-effect F_R (thickness) | ≈1.000 |
| R_DC / R_AC @ 100 °C | 1.61 mΩ |
| Practical F_R (edge/proximity) | 1.1–1.5 (field-solve) |
| L (over plane / isolated) | 10 / 28 nH |
| X_L @ 48 kHz | 3.0 / 8.6 mΩ |
| Z₀ | ~22 Ω |
| Loss @ 30 A_rms | ~1.5–2.2 W |

**Design takeaways:**
- 2 oz copper is fully utilized at 48 kHz. Going to 3–4 oz still gives a nearly linear R reduction, since t remains less than δ up to ~300 µm.
- Prioritize minimizing loop inductance: run the return close underneath and parallel phase layers with stitching vias.
- Verify the edge-crowding factor with a 2D field solver for your actual geometry.

If you share the Cycle 31 trace widths, lengths, stackup, and current profile, I can recompute with your exact numbers.