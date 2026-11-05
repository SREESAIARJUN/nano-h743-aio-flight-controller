# PCB Motor Phase Trace Analysis: 2 oz Copper at 48 kHz

I don't have the details of your "Cycle 17" design, so this uses stated assumptions. Swap in your actual geometry, stack-up, and current values. All formulas are general.

---

## 1. Assumptions (replace with your values)

| Parameter | Symbol | Assumed value |
|---|---|---|
| Copper weight | — | 2 oz/ft² |
| Copper thickness | t | 70 µm (≈68.6 µm nominal) |
| Trace width | w | 10 mm |
| Trace length | ℓ | 50 mm |
| Dielectric height to return plane | h | 1.6 mm |
| Dielectric constant (FR-4) | εr | 4.5 |
| Copper resistivity at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Operating temperature | T | 100 °C |
| Frequency of interest (PWM) | f | 48 kHz |

**Key point:** The motor phase current consists of the electrical fundamental (typically tens to hundreds of Hz) plus PWM ripple at 48 kHz and its harmonics. The DC/fundamental component usually dominates the loss. The 48 kHz analysis applies to the ripple component.

---

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

At 48 kHz:

$$\delta = \sqrt{\frac{1.72\times10^{-8}}{\pi \cdot 48\,000 \cdot 4\pi\times10^{-7}}} = \sqrt{9.08\times10^{-8}} \approx \mathbf{301\ \mu m}$$

At 100 °C, ρ rises by about 31%, so δ ≈ 345 µm.

**Ratio:** Δ = t/δ = 70/301 ≈ **0.23**

The copper is far thinner than one skin depth, so **skin effect is negligible at 48 kHz.**

---

## 3. DC Resistance

$$R_{DC} = \frac{\rho \,\ell}{w\,t}$$

$$R_{DC,20^\circ C} = \frac{1.72\times10^{-8}\cdot 0.05}{0.010 \cdot 70\times10^{-6}} = \mathbf{1.23\ m\Omega}$$

Temperature correction:

$$R_{DC}(T) = R_{DC,20}\,[1 + \alpha (T - 20)] = 1.23 \times 1.314 = \mathbf{1.62\ m\Omega\ @\ 100^\circ C}$$

---

## 4. AC Resistance at 48 kHz

### 4a. Through-thickness skin effect (Dowell, single layer, small Δ)

$$F_R = \frac{R_{AC}}{R_{DC}} \approx 1 + \frac{\Delta^4}{45}$$

$$F_R \approx 1 + \frac{0.23^4}{45} = 1 + 6\times10^{-5} \approx \mathbf{1.000}$$

### 4b. Exact single-foil form (for higher harmonics)

$$F_R = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}$$

### 4c. Effects that do matter

- **Lateral (edge) crowding:** Since w ≫ δ, current concentrates on the side of the trace facing the return path. For a microstrip over a solid plane, this is mild at 48 kHz, typically adding a few percent. It is worse if the return trace runs side-by-side.
- **Proximity effect:** This applies to adjacent phase traces or parallel layers carrying opposing current. Estimate it with the Dowell m-layer term:

$$F_R = \Delta\left[\frac{\sinh2\Delta+\sin2\Delta}{\cosh2\Delta-\cos2\Delta} + \frac{2(m^2-1)}{3}\cdot\frac{\sinh\Delta-\sin\Delta}{\cosh\Delta+\cos\Delta}\right]$$

- **PWM harmonics:**

| Frequency | δ | t/δ | Effect |
|---|---|---|---|
| ~1 MHz | ≈66 µm | ≈ 1 | Skin effect becomes significant |
| ~10 MHz | ≈21 µm | ≈ 3.3 | F_R ≈ 3.3 or more |

These harmonics matter mainly for EMI, not conduction loss.

**Engineering result:** R_AC(48 kHz) ≈ R_DC ≈ **1.6 mΩ @ 100 °C**. Allow a 1.05–1.1× margin for edge crowding and proximity.

---

## 5. Trace Impedance

### 5a. Characteristic impedance (microstrip, Hammerstad–Wheeler, w/h > 1)

Effective permittivity:

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 2.75 + 1.75(2.92)^{-1/2} \approx 3.77$$

Characteristic impedance:

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\!\left(\frac{w}{h}+1.444\right)\right]}$$

With w/h = 6.25:

$$Z_0 = \frac{377}{1.943 \times 9.00} \approx \mathbf{21.6\ \Omega}$$

**Is Z₀ relevant?** No, not at 48 kHz. The wavelength is:

$$\lambda = \frac{c}{f\sqrt{\varepsilon_{eff}}} = \frac{3\times10^8}{48\,000 \times 1.943} \approx 3.2\ \text{km}$$

A 50 mm trace is electrically tiny, so treat it as a **lumped R + jωL**. Z₀ only matters for switching-edge ringing at tens of MHz.

### 5b. Lumped inductance

**Case A: over a solid return plane.** Inductance per unit length:

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = \frac{21.6 \times 1.943}{3\times10^8} \approx 140\ \text{nH/m}$$

$$L \approx 140 \times 0.05 = \mathbf{7\ nH}$$

**Case B: isolated flat strip, no plane** (partial self-inductance, ℓ and w in cm, result in nH):

$$L = 2\ell\left[\ln\!\frac{2\ell}{w+t} + 0.5 + 0.2235\frac{w+t}{\ell}\right] \approx \mathbf{28\ nH}$$

### 5c. Total impedance magnitude at 48 kHz

$$|Z| = \sqrt{R_{AC}^2 + (2\pi f L)^2}$$

| Configuration | R_AC | ωL | \|Z\| |
|---|---|---|---|
| Over solid plane (7 nH) | 1.62 mΩ | 2.1 mΩ | **≈2.7 mΩ** |
| No plane (28 nH) | 1.62 mΩ | 8.6 mΩ | **≈8.7 mΩ** |

At 48 kHz, inductive reactance is comparable to or larger than resistance. **Layout of the return path dominates the impedance, not copper skin effect.**

---

## 6. Power Dissipation

$$P = I_{RMS,fund}^2\, R_{DC}(T) + \sum_n I_{ripple,n}^2\, R_{AC}(nf)$$

**Example:** 30 A RMS phase current with 3 A RMS ripple at 48 kHz:

$$P \approx 30^2(1.62\,\text{m}\Omega) + 3^2(1.7\,\text{m}\Omega) \approx 1.46 + 0.015 \approx \mathbf{1.47\ W}$$

The ripple contribution is about 1%. Validate the resulting temperature rise using **IPC-2152** charts or thermal simulation, since the 100 °C assumption must be self-consistent with the calculated losses.

---

## 7. Summary

| Quantity | Value |
|---|---|
| Skin depth @ 48 kHz | ~300 µm (≫ 70 µm copper) |
| R_DC (50 mm × 10 mm, 2 oz, 100 °C) | 1.62 mΩ |
| R_AC/R_DC @ 48 kHz | ≈1.00 (use 1.05–1.1 with proximity/edge margin) |
| Z₀ (microstrip) | ~22 Ω (not relevant at 48 kHz) |
| Loop L (plane / no plane) | 7 nH / 28 nH |
| \|Z\| @ 48 kHz | 2.7–8.7 mΩ |

## 8. Design Recommendations

1. Use a solid return plane or a tightly coupled return to minimize loop L.
2. Parallel the trace on multiple layers to lower R_DC. This is more effective than heavier copper only if layers aren't proximity-coupled with opposing currents.
3. Model vias as current bottlenecks, roughly 0.5–1 mΩ each for small vias. Use via arrays.
4. Verify with field-solver extraction (e.g., Ansys Q3D, Saturn PCB, or FEMM) if the geometry is non-uniform.

If you share your actual trace dimensions, stack-up, current profile, and what "Cycle 17" refers to, I can rerun the numbers specifically.