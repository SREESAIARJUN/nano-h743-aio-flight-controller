# PCB Trace Impedance and Skin-Effect AC Resistance at 48 kHz: 2 oz Motor Phase Tracks

**Note on "Cycle 22":** I don't have your board files or earlier design cycles. Below, the method is complete, and the worked numbers use a **placeholder geometry**. Substitute your actual width, length, dielectric height and temperature.

**Placeholder geometry**

| Parameter | Value |
|---|---|
| Copper | 2 oz, t = 70 µm (nominal; finished is often 60–68 µm, so check with your fab) |
| Width | w = 10 mm |
| Length | ℓ = 50 mm |
| Height above return plane | h = 0.2 mm (FR-4, εr ≈ 4.4) |
| Resistivity | ρ = 1.72×10⁻⁸ Ω·m at 20 °C; α = 0.00393 /K |

---

## 1. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

| Condition | δ |
|---|---|
| 48 kHz, 20 °C | **0.301 mm** |
| 48 kHz, 100 °C (ρ = 2.25×10⁻⁸) | **0.345 mm** |

A quick general form is δ ≈ 66 µm / √(f in MHz) at 20 °C.

**Key result:** t = 70 µm is much smaller than 2δ ≈ 600 µm. **Skin effect through the copper thickness is negligible at 48 kHz.**

---

## 2. DC Resistance

$$R_{DC} = \frac{\rho\,\ell}{w\,t}\,[1+\alpha(T-20)]$$

- At 20 °C: R_DC = 1.72e-8 × 0.05 / (0.01 × 70e-6) = **1.23 mΩ**
- At 100 °C: multiply by 1.314, giving **1.61 mΩ**

---

## 3. AC Resistance Factor (Slab Conductor, Current on Both Faces)

Define ξ = t / (2δ). Then:

$$F_R = \frac{R_{AC}}{R_{DC}} = \xi\,\frac{\sinh 2\xi + \sin 2\xi}{\cosh 2\xi - \cos 2\xi}$$

For small ξ (below about 0.5), this simplifies to F_R ≈ 1 + (4/45)ξ⁴.

| Frequency | δ (µm) | ξ | F_R | Notes |
|---|---|---|---|---|
| 48 kHz (f_sw) | 301 | 0.116 | 1.0000 | Negligible |
| 96 kHz (2f_sw, center-aligned sidebands) | 213 | 0.164 | 1.0001 | Negligible |
| 480 kHz (10th harmonic) | 95 | 0.37 | 1.002 | Negligible |
| 1 MHz | 66 | 0.53 | 1.007 | Negligible |
| 10 MHz (switching edges) | 21 | 1.67 | ≈1.5 | Ringing energy only |

**Correction for wide traces:** on a wide trace over a plane, the proximity effect pushes current toward the plane-facing surface and the trace edges. Use the one-sided form instead (ξ = t/δ with the same expression), or apply an empirical edge-crowding factor of 1.2–2× above about 1 MHz. At 48 kHz this is still negligible.

---

## 4. Loss Formulation

Separate the current into the fundamental and the PWM ripple harmonics:

$$P = I_{rms,fund}^2\,R_{DC}(T) + \sum_h I_{h}^2\,R_{DC}(T)\,F_R(f_h)$$

- **Fundamental:** the motor electrical frequency is typically below 2 kHz, so it sees pure R_DC.
- **Ripple:** the 48 kHz ripple is usually 10–30 % of I_pk and contributes almost nothing extra.

**Example:** 30 A RMS through the trace at 100 °C:
P = 30² × 1.61 mΩ ≈ **1.45 W**

That is a thermal (IPC-2152) problem, not a skin-effect problem.

---

## 5. Trace Impedance (What Actually Matters at 48 kHz)

For a phase track at 48 kHz, the relevant parameter is **loop inductance**, not characteristic impedance Z₀. The electrical length (λ ≈ 3 km in FR-4) is about 10⁵ times longer than the trace, so it is lumped.

**a) Isolated flat bar (no adjacent return plane), Rosa/Grover formula**

$$L \approx 2\ell\left[\ln\frac{2\ell}{w+t} + 0.5 + 0.2235\frac{w+t}{\ell}\right]\ \text{nH}\quad(\ell\ \text{in cm})$$

- L ≈ **28 nH**
- X_L = 2π(48 kHz)L ≈ **8.6 mΩ**, which is about 5× R_DC

**b) Over a solid return plane, parallel-plate approximation (w ≫ h)**

$$L \approx \frac{\mu_0 h \ell}{w}$$

- L ≈ **1.26 nH**
- X_L ≈ **0.38 mΩ**

**Total impedance**

$$Z = \sqrt{(R_{DC}F_R)^2 + (2\pi f L)^2}$$

| Case | \|Z\| at 48 kHz, 100 °C |
|---|---|
| Isolated trace | ≈ 8.7 mΩ |
| Over a plane | ≈ 1.65 mΩ |

**Characteristic impedance (for completeness):** Z₀ is only relevant for edge ringing in the 10–100 MHz range. For a wide trace with w/h > 1, use Wheeler/Hammerstad:

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\,[w/h + 1.393 + 0.667\ln(w/h+1.444)]}$$

With w/h = 50 and ε_eff ≈ 4.2, Z₀ ≈ **3.4 Ω**. This is very low, which is good: it minimizes the dv/dt ringing amplitude.

---

## 6. Design Conclusions

1. **Skin effect is not a design driver at 48 kHz for 2 oz copper.** Even 4 oz (140 µm) gives F_R ≈ 1.002. Size the track on R_DC and temperature rise.
2. **Inductance is the dominant AC term.** Route phase tracks over or adjacent to their return path, or on paired layers. This reduces L by about 20×, cutting the voltage overshoot (L·di/dt) at the switches.
3. **Paralleling layers works.** Because δ is much greater than t, stacking 2 oz layers with stitching vias halves R almost ideally, with no AC penalty.
4. **Verify the inputs:** use the measured finished copper thickness, the actual ripple spectrum (FFT of the phase current), and the hotspot temperature for ρ(T).

If you share your actual Cycle 22 dimensions, current profile and stack-up, I can recompute these numbers for your board.