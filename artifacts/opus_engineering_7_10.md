# PCB Motor Phase Trace Analysis at 48 kHz: Impedance and Skin-Effect AC Resistance

I don't have the specific parameters from your "Cycle 7" design (trace length, width, stackup, phase current). The derivation below uses a stated **reference geometry**. Substitute your values into the same equations.

---

## 1. Reference Geometry and Constants

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | — | 2 oz/ft² |
| Copper thickness | t | 70 µm (0.070 mm) |
| Trace width | w | 5.0 mm |
| Trace length (one phase run) | ℓ | 50 mm |
| Dielectric height to return plane | h | 1.5 mm (FR-4) |
| Relative permittivity | εᵣ | 4.5 |
| Copper resistivity at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Frequency (PWM fundamental) | f | 48 kHz |
| Permeability | µ₀ | 4π × 10⁻⁷ H/m |

---

## 2. DC Resistance

**Resistance:**

$$R_{DC} = \frac{\rho\,\ell}{w\,t}$$

$$R_{DC,20^\circ C} = \frac{1.72\times10^{-8} \times 0.05}{0.005 \times 70\times10^{-6}} = \mathbf{2.46\ m\Omega}$$

**Sheet resistance of 2 oz copper:**

$$R_\square = \rho/t = 0.246\ \text{m}\Omega/\square$$

This means R ≈ 0.246 mΩ × (ℓ/w) = 0.246 × 10 squares.

**Temperature correction (e.g., 100 °C operating copper):**

$$R_{DC,T} = R_{DC,20}\,[1+\alpha(T-20)] = 2.46 \times 1.314 = \mathbf{3.23\ m\Omega}$$

---

## 3. Skin Depth at 48 kHz

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

$$\delta_{20^\circ C} = \sqrt{\frac{1.72\times10^{-8}}{\pi \cdot 48\,000 \cdot 4\pi\times10^{-7}}} = \sqrt{9.08\times10^{-8}} \approx \mathbf{301\ \mu m}$$

$$\delta_{100^\circ C} \approx 301\sqrt{1.314} \approx \mathbf{345\ \mu m}$$

The normalized thickness ratio is:

$$\Delta = t/\delta = 70/301 = 0.233$$

**Key conclusion:** the skin depth is about 4.3× the copper thickness. Through the thickness, the 2 oz copper is effectively fully utilized at 48 kHz.

---

## 4. AC Resistance: Thickness Direction (Skin Effect)

Using the Dowell foil-conductor model:

$$F_R = \frac{R_{AC}}{R_{DC}} = \Delta\,\frac{\sinh 2\Delta + \sin 2\Delta}{\cosh 2\Delta - \cos 2\Delta}$$

For Δ ≪ 1, the series expansion is:

$$F_R \approx 1 + \frac{4}{45}\Delta^4$$

| Case | F_R | R_AC (20 °C) |
|---|---|---|
| Field on one face (worst case, adjacent return) | 1 + 0.089 × 0.00293 = **1.00026** | 2.46 mΩ |
| Symmetric field (isolated strip) ≈ 1 + Δ⁴/180 | **1.000016** | 2.46 mΩ |

**The vertical skin effect is negligible (<0.03%) at 48 kHz.**

---

## 5. AC Resistance: Width Direction (Edge Crowding / Proximity)

This is the effect that actually matters for wide, thin power traces. Lateral current redistribution becomes significant when the width exceeds a characteristic length:

$$\ell_c \approx \frac{\delta^2}{t} = \frac{(301\ \mu m)^2}{70\ \mu m} \approx 1.3\ \text{mm}$$

Since w = 5 mm > ℓ_c, current crowds toward the trace edges (isolated strip) or toward the side nearest the return conductor (adjacent phase or plane).

- **Over a solid return plane:** current spreads fairly uniformly under the trace. Expect F_R ≈ 1.05–1.15.
- **Coplanar return (adjacent phase track, no plane):** current crowds at facing edges. F_R ≈ 1.2–1.5 is possible.

For the design value, use:

$$R_{AC} = F_{R,lateral} \times R_{DC,T}$$

For example, 1.15 × 3.23 = **3.7 mΩ at 100 °C**. Confirm with a 2D field solver (FEMM, Ansys Q3D, or Sonnet) if the margin is tight.

---

## 6. Harmonic Content (PWM Edges)

The skin depth scales as δ ∝ 1/√f:

| Frequency | δ | Δ = t/δ | F_R (one-sided Dowell) |
|---|---|---|---|
| 48 kHz | 301 µm | 0.23 | ≈1.00 |
| 240 kHz (5th) | 135 µm | 0.52 | ≈1.006 |
| 1 MHz | 66 µm | 1.06 | ≈1.2 |
| 10 MHz | 21 µm | 3.3 | ≈3.3 |

Total loss is the sum over the spectrum:

$$P_{loss} = I_{DC/fund,rms}^2\,R_{AC}(f_1) + \sum_n I_n^2\,R_{AC}(f_n)$$

In a motor drive, the phase current is mostly the fundamental (electrical motor frequency, nearly DC relative to 48 kHz) plus a small 48 kHz ripple. The MHz-range harmonic currents are tiny, so their loss contribution is usually <1%.

---

## 7. Trace "Impedance" at 48 kHz

### 7a. Lumped vs. transmission-line check

$$\lambda_{FR4} = \frac{c}{f\sqrt{\varepsilon_{eff}}} \approx \frac{3\times10^8}{48\,000 \times 1.89} \approx 3.3\ \text{km}$$

Since ℓ = 50 mm ≪ λ/10, the trace is **lumped**. The relevant impedance is therefore Z = R_AC + jωL, not Z₀.

### 7b. Characteristic impedance (for edge-rate and EMI reference only)

Using the Hammerstad microstrip formulas with w/h = 3.33:

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 2.75 + 1.75(0.466) = 3.57$$

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h}+1.393+0.667\ln\left(\frac{w}{h}+1.444\right)\right]} = \frac{377}{1.889 \times 5.77} \approx \mathbf{34.6\ \Omega}$$

### 7c. Inductance

**Over a return plane (loop inductance per unit length from Z₀):**

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = \frac{34.6\times1.889}{3\times10^8} = 218\ \text{nH/m}$$

$$L = 218 \times 0.05 = \mathbf{10.9\ nH}$$

**No plane (partial self-inductance, Grover; ℓ in cm, result in nH):**

$$L = 2\ell\left[\ln\frac{2\ell}{w+t} + 0.5 + 0.2235\frac{w+t}{\ell}\right] = 10[2.982+0.5+0.023] \approx \mathbf{35\ nH}$$

### 7d. Total impedance at 48 kHz

$$X_L = 2\pi f L$$

| Configuration | R_AC (100 °C) | X_L | \|Z\| |
|---|---|---|---|
| Over solid plane | 3.7 mΩ | 3.3 mΩ | **≈5.0 mΩ** |
| No return plane | ~4.2 mΩ | 10.6 mΩ | **≈11.4 mΩ** |

At 48 kHz, inductive reactance is **comparable to or larger than** the resistance. Return-path layout matters more than skin effect.

---

## 8. Loss and Design Summary

Power loss per trace:

$$P = I_{rms}^2 \cdot R_{AC}$$

For example, at 30 A_rms: P = 900 × 3.7 mΩ ≈ **3.3 W per phase run**. Check the resulting temperature rise against IPC-2152.

**Design tak