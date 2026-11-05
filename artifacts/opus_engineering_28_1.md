# PCB Trace Impedance and Skin-Effect AC Resistance at 48 kHz: 2 oz Motor Phase Tracks

I don't have the documentation for "Cycle 28", so this uses representative values. Replace the parameters marked **[Assumed]** with your stack-up data and the formulas will carry through.

---

## 1. Input Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | — | 2 oz/ft² |
| Copper thickness | t | 70 µm (68.6 µm nominal) |
| Trace width | w | 10 mm **[Assumed]** |
| Trace length | ℓ | 50 mm **[Assumed]** |
| Dielectric height to reference plane | h | 1.6 mm **[Assumed]** |
| Dielectric constant (FR-4) | εᵣ | 4.4 |
| Copper resistivity at 20 °C | ρ₂₀ | 1.72 × 10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /°C |
| Operating temperature | T | 100 °C **[Assumed]** |
| PWM / ripple frequency | f | 48 kHz |

---

## 2. DC Resistance

**Formula**

$$R_{DC} = \frac{\rho \, \ell}{w \, t}, \qquad \rho(T) = \rho_{20}\left[1 + \alpha (T - 20)\right]$$

**At 20 °C**

$$R_{DC} = \frac{1.72\times10^{-8} \times 0.05}{0.01 \times 70\times10^{-6}} = 1.23\ \text{m}\Omega$$

**At 100 °C**

The correction factor is 1 + 0.00393 × 80 = 1.314, so:

$$R_{DC,100} = 1.61\ \text{m}\Omega$$

---

## 3. Skin Depth

**Formula**

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

**At 48 kHz, 20 °C**

$$\delta = \sqrt{\frac{1.72\times10^{-8}}{\pi \times 48\,000 \times 4\pi\times10^{-7}}} = 301\ \mu\text{m}$$

At 100 °C, δ scales by √1.314, giving δ ≈ 345 µm.

**Key ratio**

$$\frac{t}{\delta} = \frac{70}{301} \approx 0.23 \ll 1$$

The copper is about four times thinner than the skin depth. Skin effect through the thickness is therefore negligible at the fundamental.

---

## 4. AC Resistance

### 4a. Thickness (skin) effect

Use the 1-D slab (Dowell-type) solution for a conductor of thickness t, with x = t/δ:

$$\frac{R_{AC}}{R_{DC}} = \frac{x}{2}\cdot\frac{\sinh x + \sin x}{\cosh x - \cos x} \;\approx\; 1 + \frac{x^4}{45} \quad (x \ll 1)$$

$$\frac{R_{AC}}{R_{DC}} \approx 1 + \frac{0.23^4}{45} \approx 1.0001$$

This term is negligible.

### 4b. Edge crowding and proximity effect (the real contributor)

In a wide, thin strip over a return plane, current concentrates at the trace edges and on the face nearest the plane. This is a magnetic-field effect, not a thickness effect. Typical empirical values for w/h ≈ 6 at tens to hundreds of kHz:

$$k_{edge} \approx 1.05 \text{ to } 1.15$$

$$R_{AC,48k} \approx 1.10 \times 1.61\ \text{m}\Omega \approx 1.77\ \text{m}\Omega \quad (100\ ^\circ\text{C})$$

Use a 2-D field solver for exact values (e.g., Ansys Q3D, FEMM, or Saturn PCB as a quick check).

### 4c. PWM harmonics

The ripple current is roughly triangular, so it has odd harmonics at 48k × n. Skin depth falls with frequency:

| Harmonic | f | δ | t/δ |
|---|---|---|---|
| 1 | 48 kHz | 301 µm | 0.23 |
| 3 | 144 kHz | 174 µm | 0.40 |
| 5 | 240 kHz | 135 µm | 0.52 |
| ~19 | ~900 kHz | 70 µm | 1.0 |

Harmonic amplitudes fall as 1/n², so the weighted AC penalty stays below about 2–3% above the edge factor.

### 4d. Total phase-track loss

$$P = I_{RMS,DC/fund}^2 \, R_{DC}(T) + \sum_n I_{n,RMS}^2 \, R_{AC}(f_n)$$

For a sinusoidal motor current, the fundamental is the motor electrical frequency (Hz to a few hundred Hz), so it is effectively DC. Only the 48 kHz ripple component sees R_AC.

---

## 5. Trace Impedance (Microstrip over a Plane)

Here w/h = 6.25, which is outside the range of the IPC-2141 approximation. Use the Hammerstad wide-strip equations instead.

**Effective permittivity**

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\cdot\frac{1}{\sqrt{1+12h/w}} = 2.70 + \frac{1.70}{1.709} = 3.70$$

**Characteristic impedance**

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\left[\frac{w}{h} + 1.393 + 0.667\ln\left(\frac{w}{h}+1.444\right)\right]} = \frac{377}{1.922 \times 9.00} \approx 21.8\ \Omega$$

**Relevance at 48 kHz**

The wavelength in the dielectric is about λ ≈ c/(f√ε_eff) ≈ 3.2 km. A 50 mm trace is therefore electrically tiny, so Z₀ itself is not a design constraint. The useful quantity is the loop inductance.

---

## 6. Loop Inductance and Reactance

**Inductance per unit length**

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = \frac{21.8 \times 1.922}{3\times10^8} \approx 0.14\ \text{nH/mm}$$

For the 50 mm trace, L ≈ 7.0 nH.

**Reactance comparison**

| Frequency | X_L = 2πfL | Comparison |
|---|---|---|
| 48 kHz | 2.1 mΩ | Comparable to R_AC (1.8 mΩ) |
| 17.5 MHz (0.35/t_r, for t_r = 20 ns) | 0.77 Ω | Dominant; sets switching overshoot |

**Switching overshoot:** V = L·di/dt. For example, 50 A in 20 ns on 7 nH gives about **17.5 V** of spike. This, not R_AC, is usually the binding constraint for motor phase layout. Minimize it by routing the phase and its return as overlapping planes on adjacent layers, which cuts L′ by 3–10×.

---

## 7. Summary

| Quantity | Value (assumed geometry) |
|---|---|
| R_DC at 20 °C / 100 °C | 1.23 / 1.61 mΩ |
| Skin depth at 48 kHz | 301 µm (t/δ = 0.23) |
| Skin-effect factor | ≈ 1.000 (negligible) |
| Edge/proximity factor | ≈ 1.05–1.15 |
| R_AC at 48 kHz, 100 °C | ≈ 1.77 mΩ |
| Z₀ (microstrip) | ≈ 22 Ω (not critical, lumped regime) |
| Loop inductance | ≈ 0.14 nH/mm, 7 nH total |
| X_L at 48 kHz | ≈ 2.1 mΩ |

**Conclusion:** At 48 kHz, 2 oz copper is fully penetrated by current. Size the tracks on DC resistance and thermal rise (per IPC-2152), add about 10% for edge effects, and put the layout effort into minimizing loop inductance.

If you share the actual Cycle 28 parameters (trace width, length, stack-up height, phase RMS current, ripple amplitude, and switching edge rate), I'll recompute the numbers and add the IPC-2152 temperature-rise check.