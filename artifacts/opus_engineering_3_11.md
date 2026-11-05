# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current (Cycle 3)

I don't have your Cycle 3 design files, so this uses a representative build: a 6S 4-in-1 board with 2 oz copper and a prop-wash-cooled stack. Every parameter is listed in **§0** so you can substitute your own values. The method is the same regardless of the numbers.

---

## 0. Assumed Design Parameters

| Symbol | Meaning | Value |
|---|---|---|
| V_bus | Pack voltage, 6S under load | 25 V |
| I_tot | Total battery current (burst) | 80 A, so I_dc = 20 A per channel |
| D | Effective duty at burst throttle | 0.90 |
| f_sw | PWM frequency | 48 kHz |
| R_ds,on(25 °C) | MOSFET on-resistance, 40 V class | 1.8 mΩ |
| α_R | R_on temperature coefficient | 0.006 K⁻¹ |
| t_r + t_f | Switching transition time | 20 ns |
| t_dt, V_f | Dead time, body-diode drop | 200 ns, 0.8 V |
| Board | 37 × 37 × 1.6 mm, 6 layers, 2 oz (70 µm) copper | |
| R_bus | Battery pads → DC bus copper | 0.30 mΩ |
| R_sh | Current shunt per channel | 0.5 mΩ |
| h_conv | Prop-wash forced convection (~6 m/s) | 80 W/m²K |
| h_rad | Linearized radiation (ε ≈ 0.9, ~90 °C) | ~8 W/m²K |
| T_∞ | Ambient temperature | 25 °C |

---

## 1. Heat-Generation Physics (Source Terms)

**Phase current.** In 6-step/trapezoidal drive, the phase current during conduction is

$$I_{ph} = \frac{I_{dc}}{D} = \frac{20}{0.9} = 22.2\ \text{A}$$

Two MOSFETs conduct at any instant: the high side of one phase and the low side of another.

### 1.1 Conduction loss (temperature-dependent)

$$P_{cond}(T_j) = 2\, I_{ph}^2\, R_{25}\,[1+\alpha_R (T_j-25)]$$

$$P_{cond}(25^\circ\text{C}) = 2(22.2)^2(1.8\times10^{-3}) = 1.77\ \text{W per channel}$$

### 1.2 Switching loss

$$P_{sw} = \tfrac12 V_{bus} I_{ph}(t_r+t_f) f_{sw} = \tfrac12(25)(22.2)(20\text{ ns})(48\text{ kHz}) = 0.27\ \text{W}$$

### 1.3 Dead-time (body-diode) loss

$$P_{dt} = V_f\, I_{ph}\,(2t_{dt})\,f_{sw} = 0.8(22.2)(400\text{ ns})(48\text{ kHz}) = 0.34\ \text{W}$$

### 1.4 Gate drive and C_oss loss

$$P_g + P_{oss} \approx Q_g V_{GS} f_{sw}\cdot 2 + \sum \tfrac12 C_{oss}V^2 f_{sw} \approx 0.07\ \text{W}$$

### 1.5 Board-level sources

| Source | Expression | Loss |
|---|---|---|
| DC bus copper | I_tot² R_bus = 80² × 0.3 mΩ | 1.92 W (copper α = 0.0039 K⁻¹) |
| Shunts | 4 × 20² × 0.5 mΩ | 0.80 W |
| Bus-capacitor ESR (ripple) | | ~0.20 W |
| MCUs and regulators | | ~0.40 W |

### 1.6 Total loss as a linear function of temperature

Linearize about 25 °C:

$$P(T) = P_0 + \beta\,(T-25)$$

- **P₀** = 4(1.77 + 0.27 + 0.34 + 0.07) + 1.92 + 0.8 + 0.2 + 0.4 ≈ **13.1 W**
- **β** = 4(1.77)(0.006) + 1.92(0.0039) ≈ **0.050 W/K**

---

## 2. Governing Conduction Equation

### 2.1 Field equation

Transient anisotropic heat equation in the board:

$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{k}\,\nabla T) + \dot q'''(\mathbf{x},T), \qquad \mathbf{k}=\text{diag}(k_{xy},k_{xy},k_z)$$

### 2.2 Effective laminate conductivity

Copper thickness: 6 layers × 70 µm = 0.42 mm. The FR-4 (k = 0.3 W/mK) makes up the remaining 1.18 mm.

**In-plane (parallel rule):**

$$k_{xy} = \frac{\sum k_i t_i}{\sum t_i} = \frac{385(0.42) + 0.3(1.18)}{1.6} \approx 101\ \text{W/mK}$$

**Through-plane (series rule):**

$$k_z = \frac{\sum t_i}{\sum t_i/k_i} = \frac{1.6}{0.42/385 + 1.18/0.3} \approx 0.41\ \text{W/mK}$$

**Under the FET pads (thermal-via region).** Local through-plane conductivity is raised by the via fill fraction φ_v:

$$k_z^{via} \approx \phi_v k_{Cu} + (1-\phi_v)k_z$$

For example, φ_v = 0.08 gives k_z^via ≈ 31 W/mK.

### 2.3 Boundary conditions

| Boundary | Condition |
|---|---|
| Top and bottom faces (Robin) | −k ∂T/∂n = h_eff (T − T_∞), with h_eff = h_conv + h_rad ≈ 88 W/m²K |
| Radiation (exact form, if not linearized) | h_rad = εσ(T² + T_∞²)(T + T_∞) |
| Battery leads and motor wires | Fin sink: Q_w = √(h P k A_c) · tanh(mL) · (T_pad − T_∞) |
| Stack standoffs | Adiabatic, or a contact conductance G_c |

### 2.4 Reduced 2.5-D "shell/fin" form

Because the board is thin (Biot number in z is small away from the vias), integrate through the thickness t:

$$k_{xy}\,t\,\nabla^2_{xy}T - 2h_{eff}(T-T_\infty) + \dot q''(x,y,T) = \rho c_p t\,\frac{\partial T}{\partial t}$$

This defines a **thermal spreading length**:

$$L_s = \sqrt{\frac{k_{xy}t}{2h_{eff}}} = \sqrt{\frac{101(0.0016)}{176}} \approx 30\ \text{mm}$$

**Interpretation:** L_s is comparable to the board size (37 mm). The copper therefore spreads heat across most of the board, but measurable hot spots will still sit over each FET cluster.

---

## 3. Finite-Element Formulation

### 3.1 Weak form (Galerkin)

Find T ∈ H¹ such that, for all test functions w:

$$\int_\Omega \rho c_p t\, w\,\dot T\,dA + \int_\Omega k_{xy}t\,\nabla w\cdot\nabla T\,dA + \int_\Omega 2h_{eff}\,wT\,dA = \int_\Omega w\,(\dot q'' + 2h_{eff}T_\infty)\,dA + \oint_\Gamma w\,q_n\,ds$$

### 3.2 Discretization

Substitute T ≈ Σ N_j T_j. Each element then contributes three matrices and a load vector:

**Conduction matrix:**

$$\mathbf{K}^e_{cond} = \int_{\Omega_e} k_{xy}t\,\mathbf{B}^T\mathbf{B}\,dA$$

**Convection matrix:**

$$\mathbf{K}^e_{conv} = \int_{\Omega_e} 2h_{eff}\,\mathbf{N}^T\mathbf{N}\,dA$$

**Capacitance matrix:**

$$\mathbf{C}^e = \int_{\Omega_e}