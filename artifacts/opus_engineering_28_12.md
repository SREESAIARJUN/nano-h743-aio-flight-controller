# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

**Scope.** No prior cycles were available, so this formulation states its assumptions explicitly. Replace the example values with your BOM and stack-up data.

## 0. Baseline Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Supply (4S, full charge) | V_bus | 16.8 V |
| Total burst current | I_tot | 80 A, so I_ch = 20 A per channel |
| Commutation | — | 6-step trapezoidal, synchronous rectification |
| PWM frequency | f_sw | 48 kHz |
| MOSFETs per channel | — | 6 (3 half-bridges) |
| R_ds(on) at 25 °C | R_0 | 1.5 mΩ |
| R_ds(on) temperature coefficient | α | ≈ 0.005 K⁻¹ |
| Rise + fall time | t_r + t_f | 40 ns |
| Dead time; body-diode forward voltage | t_dt; V_f | 100 ns; 0.8 V |
| Gate charge; gate drive voltage | Q_g; V_gs | 50 nC; 10 V |
| Board | — | 36 × 36 mm, 1.6 mm thick, 6 layers of 2 oz Cu |
| Ambient air | T_∞ | 30 °C |
| Effective convection (prop wash, per face) | h_c | ≈ 60 W/m²K |

---

## 1. Heat-Generation Physics (Source Terms)

### 1.1 MOSFET conduction loss
In 6-step drive, two phase legs conduct at any instant: one through a high-side path and one through a low-side path. With synchronous rectification, the current always flows through exactly two channels. The on-resistance rises with temperature:

$$
R_{ds}(T_j) = R_0\,[1+\alpha (T_j - 25)]
$$

So the conduction loss per channel is:

$$
P_{cond} = 2\,I_{ch}^2\,R_{ds}(T_j)
$$

- At 25 °C: P_cond = 2·400·1.5 mΩ = **1.20 W**
- At 100 °C: R_ds = 2.06 mΩ, so P_cond = **1.65 W**

This temperature dependence couples the electrical and thermal problems.

### 1.2 Switching loss
Only the PWM-switched leg contributes:

$$
P_{sw} = \tfrac12 V_{bus} I_{ch}(t_r+t_f) f_{sw} = 0.5\cdot16.8\cdot20\cdot40\text{ ns}\cdot48\text{ kHz} = \mathbf{0.32\ W}
$$

### 1.3 Dead-time (body-diode) loss
There are two dead-time intervals per PWM period:

$$
P_{dt} = V_f I_{ch}\,(2 t_{dt}) f_{sw} = \mathbf{0.15\ W}
$$

Reverse-recovery loss adds roughly Q_rr·V_bus·f_sw, which is about 0.05 W here.

### 1.4 Gate-drive loss
Two devices switch per PWM cycle:

$$
P_g = 2\,Q_g V_{gs} f_{sw} \approx \mathbf{0.05\ W}
$$

This heat is dissipated mostly in the gate driver and gate resistors, not in the MOSFET die.

### 1.5 Copper (Joule) losses
- **Phase traces:** P_tr = I_ch²·R_trace ≈ 400 · 0.5 mΩ = **0.20 W per channel**
- **Shared battery bus at 80 A:** P_bus = 80² · 0.2 mΩ = **1.28 W**

The bus term is concentrated at the battery pads and the bulk capacitor region.

### 1.6 Logic and regulators
The MCUs, LDO/BEC and current-sense circuitry dissipate about **0.3 W** in total.

### 1.7 Power budget at T_j ≈ 100 °C

| Source | Per channel | ×4 / shared |
|---|---|---|
| Conduction | 1.65 W | 6.60 W |
| Switching | 0.32 W | 1.28 W |
| Dead time + Q_rr | 0.20 W | 0.80 W |
| Gate | 0.05 W | 0.20 W |
| Phase copper | 0.20 W | 0.80 W |
| Bus copper | — | 1.28 W |
| Logic | — | 0.30 W |
| **Total** | **≈ 2.4 W** | **Q_tot ≈ 11.3 W** |

### 1.8 Volumetric source field
For FEM, each source k becomes a volumetric heat density over its footprint Ω_k:

$$
\dot q'''(\mathbf x) = \sum_k \frac{P_k(T)}{V_k}\,\chi_{\Omega_k}(\mathbf x)
$$

Here χ_Ωk is the indicator function: 1 inside Ω_k and 0 outside.

- MOSFET dies are modeled as 2D surface fluxes q'' applied at the die-attach pads.
- Trace losses use the local current density: q''' = |J|²/σ_Cu(T), with σ_Cu(T) = σ_0 / [1 + 0.0039(T − 20)].

---

## 2. Governing Conduction Equation

### 2.1 Strong form
The transient heat equation, with an anisotropic conductivity tensor:

$$
\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot\big(\mathbf k\,\nabla T\big) + \dot q'''(\mathbf x, T),
\qquad
\mathbf k = \mathrm{diag}(k_\parallel, k_\parallel, k_\perp)
$$

### 2.2 Effective PCB conductivities
Treat the layer stack as conductors in parallel (in-plane) and in series (through-plane). Here t_i and k_i are the thickness and conductivity of each layer; the stack has 0.42 mm of copper and 1.18 mm of FR-4.

**In-plane (parallel layers):**

$$
k_\parallel = \frac{\sum k_i t_i}{\sum t_i} = \frac{385(0.42) + 0.3(1.18)}{1.6} \approx 101\ \text{W/m·K}
$$

**Through-plane (series layers):**

$$
k_\perp = \frac{\sum t_i}{\sum t_i/k_i} \approx 0.41\ \text{W/m·K}
$$

**Thermal vias under the FET pads** (via fill fraction φ_v):

$$
k_\perp^{via} \approx \phi_v k_{Cu} + (1-\phi_v) k_\perp
$$

With φ_v ≈ 0.05, k_⊥^via ≈ **19 W/m·K** locally.

### 2.3 Boundary conditions

**Top and bottom faces (Robin condition)**, combining convection and linearized radiation:

$$
-\mathbf k\nabla T\cdot\mathbf n = h_{eff}\,(T - T_\infty),
\qquad h_{eff} = h_c + h_r
$$

The radiation coefficient is:

$$
h_r = 4\varepsilon\sigma T_m^3 \approx 4(0.9)(5.67\times10^{-8})(340)^3 \approx 8\ \text{W/m}^2\text{K}
$$

So h_eff ≈ **68 W/m²K**.

**Other boundaries:**
- **Mounting standoffs:** Robin condition with h_s = 1/(R_standoff·A_s), giving a small conduction path to the frame.
- **Motor and battery wires:** fin boundaries, where the wire acts as a heat sink.

$$
q_{wire} = \sqrt{h P_w k_{Cu} A_w}\,(T_{pad} - T_\infty)\tanh(mL),
\qquad m = \sqrt{\frac{hP_w}{k_{Cu}A_w}}
$$

Here P_w is the wire perimeter and A_w its cross-sectional area.

---

## 3. Finite-Element Formulation

### 3.1 Weak form
Multiply by a test function w and integrate by parts over the domain Ω with convective boundary Γ_h:

$$
\int_\Omega \rho c_p\, w\,\dot T\, d\Omega + \int_\Omega \nabla w^{\!T}\mathbf k\nabla T\, d\Omega + \int_{\Gamma_h} h_{eff}\, wT\, d\Gamma
= \int_\Omega w\,\dot q'''\, d\Omega + \int_{\Gamma_h} h_{eff}\, w T_\infty\, d\Gamma
$$

### 3.2 Galerkin discretization
Approximate T ≈ N(x)·T (shape functions N, nodal temperatures T). This gives:

$$
\mathbf C\dot{\mathbf T} + \big[\mathbf K_k + \mathbf K_h\big]\mathbf T = \mathbf F_Q(\mathbf T) + \mathbf F_h
$$

### 3.3 Element matrices
**2.5D shell formulation** (thickness t, both faces cooled; this is efficient for a PCB):

$$
\mathbf K_k^e = \int_{A_e} t\,k_\parallel\,\nabla\mathbf N^T\nabla\mathbf N\, dA,
\qquad
\mathbf K_h^e = \int_{A_e} 2h_{eff}\,\mathbf N^T\mathbf N\, dA
$$

$$
\mathbf F_Q^e = \int_{A_e} \mathbf N^T q''\, dA,
\qquad
\mathbf F_h^e = \int_{A_e} 2h_{eff}T_\infty\mathbf N^T dA,
\qquad
\mathbf C^e = \int_{A_e} \rho c_p t\,\mathbf N^T\mathbf N\, dA
$$

**Linear triangle (T3) closed forms**, for element area A and gradient matrix B = ∇N (constant over the element):

$$
\mathbf K_k^e = t\,k_\parallel A\,\mathbf B^T\mathbf B,
\qquad
\mathbf K_h^e = \frac{2h_{eff}A}{12}\begin{bmatrix}2&1&1\\1&2&1\\1&1&2\end{bmatrix},
\qquad
\mathbf F^e = \frac{q''A}{3}\begin{bmatrix}1\\1\\1\end{bmatrix}
$$

**MOSFET junctions** are coupled to the board node as extra degrees of freedom. Each junction node j connects to its pad node p through a thermal resistance:

$$
R_{jp} = R_{\theta jc} + R_{solder} + R_{via\text{-}array},
\qquad \text{e.g. } 0.8 + 0.3 + 2.0 \approx 3\ \text{K/W}
$$

This adds a conductance stamp to the global stiffness matrix:

$$
\frac{1}{R_{jp}}\begin{bmatrix}1&-1\\-1&1\end{bmatrix}
$$

### 3.4 Steady-state nonlinear solve
At steady state, dT/dt = 0, leaving:

$$
\mathbf K\,\mathbf T = \mathbf F(\mathbf T)
$$

The load vector depends on temperature through R_ds(T) and σ_Cu(T). Solve by Picard iteration:

1. Start with