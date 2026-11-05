# PCB Phase-Track Calculations: Impedance and Skin-Effect AC Resistance at 48 kHz (2 oz Cu)

I don't have the Cycle 8 design data (track geometry, stack-up, currents), so the numbers below use the stated assumptions. Each formula is parametric, so you can substitute your actual values.

## 0. Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Copper weight | – | 2 oz/ft² → **t = 70 µm** (68.6 µm nominal) |
| Track width | W | 5.0 mm |
| Track length (bridge to connector) | l | 50 mm |
| Resistivity at 20 °C | ρ₂₀ | 1.72×10⁻⁸ Ω·m |
| Temperature coefficient | α | 0.00393 /K |
| Operating temperature | T | 100 °C |
| PWM frequency | f | 48 kHz |
| Dielectric (FR-4) | εr | 4.4 |
| Height to reference plane (4-layer) | h | 0.36 mm |
| Phase current | I_rms | 30 A fundamental, 3 A rms ripple at 48 kHz |

## 1. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

- **At 20 °C:** δ = √(1.72×10⁻⁸ / (π · 48 000 · 4π×10⁻⁷)) = **0.301 mm (301 µm)**
- **At 100 °C:** ρ₁₀₀ = 1.72×10⁻⁸·(1 + 0.00393·80) = 2.26×10⁻⁸ Ω·m, so **δ = 345 µm**

**Penetration ratio:** Δ = t/δ = 70/301 = **0.23**

The copper is about 4× thinner than one skin depth. Classical through-thickness skin effect is therefore negligible at 48 kHz.

## 2. DC Resistance

$$R_{DC} = \frac{\rho\, l}{W\, t}$$

The cross-section is A = 5 mm × 0.070 mm = 0.35 mm².

- **At 20 °C:** R_DC = 1.72×10⁻⁸ · 0.05 / 3.5×10⁻⁷ = **2.46 mΩ**
- **At 100 °C:** R_DC = **3.23 mΩ**

## 3. AC Resistance at 48 kHz

### (a) Through-thickness skin effect (slab model)

$$\frac{R_{AC}}{R_{DC}} = \frac{\Delta}{2}\cdot\frac{\sinh\Delta+\sin\Delta}{\cosh\Delta-\cos\Delta} \;\approx\; 1+\frac{\Delta^4}{45}\quad(\Delta<1)$$

With Δ = 0.23: Rac/Rdc ≈ 1 + 6.5×10⁻⁵, which is **effectively 1.000**.

### (b) Lateral edge-crowding (the real concern for wide, thin strips)

Current crowds toward the strip edges above a characteristic frequency:

$$f_c \approx \frac{\rho}{\mu_0\, t\, W} = \frac{1.72\times10^{-8}}{1.257\times10^{-6}\cdot 70\times10^{-6}\cdot 5\times10^{-3}} \approx 39\ \text{kHz}$$

At 48 kHz (f/f_c ≈ 1.2) the track is in the onset region. Expect **Rac/Rdc ≈ 1.05–1.15**. This range is an estimate; confirm it with a 2-D field solver (e.g. FEMM, Q2D), since a ground plane underneath and proximity to the adjacent phase both shift the result.

**Design value:** R_AC(48 kHz, 100 °C) ≈ 1.1 × 3.23 ≈ **3.6 mΩ**

### (c) Switching-edge harmonics

For a rise time t_r = 50 ns, the knee frequency is f_knee ≈ 0.35/t_r = 7 MHz. At that frequency δ = 66.1/√f mm ≈ 25 µm, which is less than t/2 = 35 µm. Edge harmonics therefore *do* see skin effect (Rac/Rdc ≈ 1.5–2), but they carry little rms energy.

## 4. Loss Budget (separate the spectral components)

$$P = I_{fund}^2 R_{DC}(T) + I_{ripple,48k}^2 R_{AC}(48k) + \sum I_n^2 R_{AC}(f_n)$$

- **Fundamental:** 30² × 3.23 mΩ = **2.91 W**
- **48 kHz ripple:** 3² × 3.6 mΩ = **0.032 W**
- **Harmonics:** typically less than 0.01 W

**Conclusion:** Losses are dominated by DC/fundamental I²R, not skin effect. Almost 3 W in a 5 mm × 50 mm track is excessive. Check the temperature rise against IPC-2152 and widen to about 10 mm, add parallel layers, or solder-tin/busbar the track.

## 5. Characteristic Impedance (microstrip, Hammerstad, W/h > 1)

W/h = 5/0.36 = 13.9

**Effective permittivity:**

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2}+\frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{W}\right)^{-1/2} = 2.70 + 1.70/1.365 = 3.95$$

**Characteristic impedance:**

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\,\left[W/h + 1.393 + 0.667\ln(W/h+1.444)\right]} = \frac{377}{1.986\cdot17.11} \approx \mathbf{11\ \Omega}$$

Note that the IPC-2141 formula ln(5.98h/(0.8W+t)) is invalid for traces this wide (its log argument falls below 1), so it can't be used here.

**Electrical length check:** λ = c/(√ε_eff · f) = 3×10⁸/(1.986 · 48 000) ≈ **3.1 km**. A 50 mm track is electrically short (< λ/10 holds even up to about 75 MHz). **Z₀ is irrelevant at 48 kHz**, and the track should be modelled as lumped R + L.

## 6. Lumped Inductance (what actually matters)

**With a solid ground/return plane directly underneath:**

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = \frac{11.1\cdot1.986}{3\times10^8} \approx 73\ \text{nH/m}$$

This gives **L ≈ 3.7 nH** over 50 mm.

**Without a plane (isolated flat strip, Rosa formula, l and W in cm, L in nH):**

$$L = 2l\left[\ln\frac{2l}{W+t}+0.5+0.2235\frac{W+t}{l}\right] = 10\,[2.98+0.5+0.02] \approx \mathbf{35\ nH}$$

**Reactance at 48 kHz:**

| Configuration | X_L = 2πfL | Compared with R_AC (3.6 mΩ) |
|---|---|---|
| Over plane (3.7 nH) | 1.1 mΩ | Smaller |
| No plane (35 nH) | 10.6 mΩ | About 3× larger |

At 7 MHz edge content, L × di/dt drives the ringing. For 35 nH at 30 A in 50 ns, V = L·di/dt ≈ **21 V of overshoot**. Keep the phase/return loop tight and over a plane.

## 7. Summary Table (per 50 mm track)

| Quantity | 20 °C | 100 °C |
|---|---|---|
| Skin depth at 48 kHz | 301 µm | 345 µm |
| t/δ | 0.23 | 0.20 |
| R_DC | 2.46 mΩ | 3.23 mΩ |
| R_AC at 48 kHz (incl. edge crowding) | ≈2.7 mΩ | ≈3.6 mΩ |
| Z₀ (microstrip) | 11 Ω | – |
| L (over plane / no plane) | 3.7 nH / 35 nH | – |
| Loss at 30 A rms + 3 A ripple | – | ≈2.95 W |

## Recommendations for Cycle 8

1. Size the track for DC/fundamental I²R and IPC-2152 temperature rise. Skin effect at 48 kHz is a minor (<15 %) correction.
2. Mirror the phase copper on two or more layers with via stitching. This halves resistance and reduces inductance, and it is more effective than going to 3 oz copper.
3. Route over an unbroken plane to minimise loop inductance and switching overshoot.

If you share the actual Cycle 8 widths, lengths, stack-up and current profile, I'll rerun the numbers.