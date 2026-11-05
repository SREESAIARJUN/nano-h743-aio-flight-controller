# PCB Phase-Track Impedance & AC Resistance at 48 kHz (2 oz Cu)

I don't have the parameters from earlier cycles, so the worked example uses representative values (listed below). Replace them with your Cycle 10 values; the formulas stay the same.

## 0. Assumptions

| Parameter | Symbol | Value used |
|---|---|---|
| Copper weight | – | 2 oz/ft² → **t = 70 µm** (finished ≈ 66–70 µm) |
| Trace width | w | 5.0 mm |
| Trace length (bridge to connector) | l | 50 mm |
| Dielectric height to reference plane | h | 0.20 mm (4-layer stack) |
| Laminate permittivity | εr | 4.3 (FR-4 @ ~1 MHz) |
| PWM frequency | f_sw | 48 kHz |
| Phase current (fundamental) | I_ph | 30 A_rms |
| PWM ripple current | I_rip | 3 A_rms at 48 kHz |
| Switch-node rise time | t_r | 20 ns |
| ρ_Cu (20 °C), α | ρ₀, α | 1.68×10⁻⁸ Ω·m, 0.00393 /K |

## 1. DC Resistance with Temperature

$$R_{DC}(T) = \frac{\rho_0\,[1+\alpha(T-20)]\;l}{w\,t}$$

- 20 °C: R = (1.68×10⁻⁸ × 0.05)/(5×10⁻³ × 70×10⁻⁶) = **2.40 mΩ**
- 100 °C (ρ = 2.21×10⁻⁸): **3.15 mΩ**

## 2. Skin Depth

$$\delta = \sqrt{\frac{\rho}{\pi f \mu_0}}$$

| Frequency | δ @ 20 °C | δ @ 100 °C | t/δ (20 °C) |
|---|---|---|---|
| 48 kHz (f_sw) | **298 µm** | 341 µm | 0.235 |
| 96 kHz (2f_sw, center-aligned ripple) | 211 µm | 241 µm | 0.33 |
| 1 MHz | 65 µm | 75 µm | 1.07 |
| 17.5 MHz (edge knee, 0.35/t_r) | 15.6 µm | 17.8 µm | 4.5 |

**Key result:** at 48 kHz, t = 70 µm is much smaller than 2δ ≈ 600 µm. Current flows through the full copper thickness, so through-thickness skin effect is negligible.

## 3. AC Resistance Factor (Dowell Formulation)

With Δ = t/δ and m = the number of current-carrying layers stacked in the field (proximity effect):

$$F_R=\frac{R_{AC}}{R_{DC}} = \Delta\left[\frac{\sinh 2\Delta+\sin 2\Delta}{\cosh 2\Delta-\cos 2\Delta} + \frac{2(m^2-1)}{3}\cdot\frac{\sinh\Delta-\sin\Delta}{\cosh\Delta+\cos\Delta}\right]$$

For small Δ, this reduces to:

$$F_R \approx 1 + \frac{4}{45}\Delta^4 + \frac{(m^2-1)}{9}\cdot\frac{\Delta^4}{1}\;(\text{approx.})$$

At 48 kHz (Δ = 0.235, Δ⁴ = 3.05×10⁻³):
- Single layer (m = 1): F_R ≈ **1.0003**
- Same phase paralleled on 2 layers (m = 2): F_R ≈ **1.001**

So R_AC(48 kHz, 100 °C) ≈ **3.15 mΩ**, effectively equal to DC.

**Lateral edge crowding.** Across the 5 mm width, current crowds toward the trace edges and toward the return path. Over a solid plane at h = 0.2 mm, this adds roughly 2–10 % at 48 kHz. Budget **F_R ≈ 1.05** as a conservative design factor.

## 4. Series Impedance (the quantity that matters at 48 kHz)

The trace is electrically short. Wavelength at 48 kHz in FR-4 is about 3 km; even at the 17.5 MHz edge knee it is about 8.6 m, compared with l/λ ≈ 0.006. Treat the trace as lumped: **Z = R_AC + jωL**.

### 4a. Inductance with a solid reference plane (microstrip)

Effective permittivity (Hammerstad, w/h = 25):

$$\varepsilon_{eff} = \frac{\varepsilon_r+1}{2} + \frac{\varepsilon_r-1}{2}\left(1+\frac{12h}{w}\right)^{-1/2} = 4.01$$

Characteristic impedance (w/h ≥ 1):

$$Z_0 = \frac{120\pi}{\sqrt{\varepsilon_{eff}}\,\left[\frac{w}{h}+1.393+0.667\ln\left(\frac{w}{h}+1.444\right)\right]} = \mathbf{6.6\ \Omega}$$

Inductance:

$$L' = \frac{Z_0\sqrt{\varepsilon_{eff}}}{c} = 44\ \text{nH/m} \;\Rightarrow\; L = \mathbf{2.2\ nH}$$

The resulting reactance is X_L(48 kHz) = 2π·48k·2.2n = **0.66 mΩ**.

### 4b. Inductance with no adjacent return plane (isolated strip, partial self-inductance)

$$L = 2\times10^{-7}\,l\left[\ln\frac{2l}{w+t} + 0.5 + 0.2235\frac{w+t}{l}\right] = \mathbf{35\ nH}$$

The resulting reactance is X_L(48 kHz) = **10.6 mΩ**, which is more than three times R_AC.

### Summary of impedance

| Configuration | R_AC (100 °C) | X_L @ 48 kHz | \|Z\| |
|---|---|---|---|
| Over solid plane | 3.3 mΩ | 0.66 mΩ | **3.4 mΩ** |
| No return plane | 3.3 mΩ | 10.6 mΩ | **11.1 mΩ** |

**Design takeaway:** at 48 kHz, layout-driven inductance dominates over skin effect. Route the phase over a continuous plane or with a closely coupled return path.

## 5. Loss and Thermal Check

Losses are summed per harmonic component:

$$P = I_{ph}^2 R_{DC} + \sum_n I_{rip,n}^2\,R_{DC}\,F_R(f_n)$$

- Fundamental: 30² × 3.15 mΩ = 2.84 W
- 48 kHz ripple: 3² × 3.15 mΩ × 1.05 = 0.03 W
- **Total ≈ 2.87 W per phase track** (about 57 W/m)

**Thermal iteration.** Using IPC-2152 for an external 2 oz trace, 5 mm wide, with 30 A, the expected rise is roughly 30–45 °C (strongly dependent on board and plane spreading). Iterate T → ρ → R → P until the result converges, usually within 2–3 passes. If ΔT exceeds the budget:
- widen the trace,
- parallel the phase on an inner layer with stitching vias (keep m small and the layers closely coupled), or
- add solder or busbar reinforcement.

## 6. High-Frequency Edge Content (EMI, not loss)

At 1–17.5 MHz, Δ ranges from about 1 to 4.5, so F_R rises to about 1.5–4.5. The current is small, so the loss impact is minor, but the result matters for:
- dv/dt ringing with L_trace (2.2–35 nH) and the MOSFET C_oss,
- placement of snubbers and Y-capacitors.

## Results Summary

| Quantity | Value |
|---|---|
| δ @ 48 kHz | 298 µm (20 °C) / 341 µm (100 °C) |
| t/δ | 0.235 → skin effect negligible |
| R_DC / R_AC (100 °C, 50 mm) | 3.15 mΩ / ≈3.3 mΩ (F_R ≈ 1.05 incl. edge crowding) |
| Z₀ (over plane) | 6.6 Ω (informational only; line is electrically short) |
| L | 2.2 nH (with plane) – 35 nH (no plane) |
| Loss per phase | ≈ 2.9 W at 30 A_rms |

If you share the actual Cycle 10 width, length, stack-up, and current values, I can rerun the numbers.