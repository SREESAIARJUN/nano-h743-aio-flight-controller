# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the parameters from your earlier design cycles, so this formulation uses a stated baseline. Every number feeds the equations symbolically, so you can substitute your Cycle-8 values directly.

**Baseline assumptions**

| Parameter | Symbol | Value |
|---|---|---|
| Board size | L × W | 30.5 × 30.5 mm (≈ 36 × 36 mm outline) |
| Stack-up | | 1.6 mm FR-4, 6 layers × 2 oz Cu (70 µm) |
| Bus voltage | V_bus | 6S, 25.2 V (worst case, fully charged) |
| Current | I_m | 80 A total, so 20 A phase current per channel |
| MOSFETs | | 24 × PQFN 3.3×3.3 (6 per channel), R_ds,on(25 °C) = 1.5 mΩ |
| R_ds temperature coefficient | α | ≈ 0.005 K⁻¹ |
| PWM frequency | f_sw | 48 kHz |
| Switching times | t_r, t_f | 20 ns each |
| Dead time | t_dt | 300 ns |
| Body-diode forward voltage | V_f | 0.8 V |
| Reverse-recovery charge | Q_rr | 40 nC |
| Gate charge / drive voltage | Q_g, V_gs | 30 nC, 10 V |
| Ambient | T_∞ | 40 °C |
| Prop-wash convection | h_c | 60–150 W/m²K (still air: 10–15) |
| Surface emissivity | ε | 0.9 (solder mask) |

---

## 1. Heat-Source Physics (Loss Model)

Each channel uses six-step commutation with synchronous rectification. Each phase leg conducts two-thirds of the time, and one FET in the active leg is always on.

**Conduction loss** (temperature dependent):

$$P_{cond} = 3\cdot\tfrac{2}{3}\, I_m^2\, R_{ds}(T_j) = 2 I_m^2 R_{ds,25}\,[1+\alpha(T_j-25)]$$

**Switching loss** (one leg PWM-switching at a time):

$$P_{sw} = \tfrac12 V_{bus} I_m (t_r+t_f) f_{sw}$$

**Dead-time body-diode loss and reverse recovery:**

$$P_{dt} = 2 V_f I_m t_{dt} f_{sw}, \qquad P_{rr} = V_{bus} Q_{rr} f_{sw}$$

**Gate drive loss** (6 FETs), **copper trace loss**, and **capacitor ESR loss:**

$$P_g = 6\, Q_g V_{gs} f_{sw}, \quad P_{Cu} = I_m^2 R_{trace}, \quad P_{ESR} = I_{C,rms}^2 \text{ESR}$$

**Per-channel totals** at T_j ≈ 105 °C, where R_ds ≈ 2.2 mΩ:

| Term | Value |
|---|---|
| P_cond = 2·400·0.0022 | 1.76 W |
| P_sw = ½·25.2·20·40 ns·48 kHz | 0.48 W |
| P_dt = 2·0.8·20·300 ns·48 kHz | 0.46 W |
| P_rr | 0.05 W |
| P_g | 0.09 W |
| P_Cu (R_trace ≈ 0.5 mΩ) | 0.20 W |
| P_ESR | 0.05 W |
| **Per channel** | **≈ 3.1 W** |

**Board total:** 4 × 3.1 + BEC/MCU (≈ 0.5 W) gives **Q_tot ≈ 13 W**.

The volumetric source term used in the FEM model is each component's loss divided by its die or pad volume:

$$\dot q'''(\mathbf{x},T) = \sum_k \frac{P_k(T)}{V_k}\,\chi_k(\mathbf{x})$$

Here χ_k is the indicator function of component k's region.

---

## 2. Governing Conduction Equation

The PCB is an orthotropic laminate. Its conductivity tensor is:

$$\mathbf{k} = \mathrm{diag}(k_\parallel, k_\parallel, k_\perp)$$

**In-plane conductivity** (parallel rule):

$$k_\parallel = \frac{\sum_i k_i t_i}{t} = \frac{0.42\text{ mm}\cdot 385 + 1.18\text{ mm}\cdot 0.3}{1.6\text{ mm}} \approx 101\ \text{W/m·K}$$

**Through-plane conductivity** (series rule):

$$k_\perp = \frac{t}{\sum_i t_i/k_i} \approx 0.41\ \text{W/m·K} \quad (\text{no vias})$$

Under a thermal-via field with copper area fraction φ_v, this rises to:

$$k_\perp^{via} \approx \varphi_v\, k_{Cu} + (1-\varphi_v)\, k_\perp$$

For example, φ_v = 5 % gives k_⊥ ≈ 19.6 W/m·K.

**Transient heat equation:**

$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{k}\nabla T) + \dot q'''(\mathbf{x},T) \quad \text{in } \Omega$$

**Steady state** (∂T/∂t = 0):

$$\nabla\cdot(\mathbf{k}\nabla T) + \dot q'''(T) = 0$$

**Boundary conditions** (Robin type, on both faces Γ_top and Γ_bot):

$$-\mathbf{n}\cdot\mathbf{k}\nabla T = h_{eff}(T - T_\infty)$$

The effective coefficient combines convection with linearized radiation:

$$h_{eff} = h_c + h_r, \qquad h_r = 4\varepsilon\sigma \bar T^3 \approx 4(0.9)(5.67\times10^{-8})(360)^3 \approx 9.5\ \text{W/m}^2\text{K}$$

Two further boundary conditions apply:
- **Mounting standoffs:** use a contact conductance h_mount on the screw-hole annuli.
- **Board edges:** treat as adiabatic, since the edge area is negligible.

---

## 3. Finite-Element Formulation

### Weak form

Find T ∈ H¹(Ω) such that, for all test functions w:

$$\int_\Omega \nabla w\cdot\mathbf{k}\nabla T\, d\Omega + \int_{\Gamma} h_{eff}\, w\, T\, d\Gamma = \int_\Omega w\,\dot q'''\, d\Omega + \int_{\Gamma} h_{eff}\, w\, T_\infty\, d\Gamma$$

### Galerkin discretization

Substitute T ≈ Σ N_j T_j with shape functions N_j, which yields the global system:

$$\big[\mathbf{K}_c + \mathbf{K}_h\big]\mathbf{T} = \mathbf{F}_q(\mathbf{T}) + \mathbf{F}_h$$

| Matrix / vector | Definition |
|---|---|
| Conduction matrix | $K_{c,ij} = \int_\Omega \nabla N_i\cdot\mathbf{k}\nabla N_j\, d\Omega$ |
| Convection matrix | $K_{h,ij} = \int_\Gamma h_{eff} N_i N_j\, d\Gamma$ |
| Heat-generation load | $F_{q,i} = \int_\Omega N_i \dot q'''\, d\Omega$ |
| Convective load | $F_{h,i} = \int_\Gamma h_{eff} T_\infty N_i\, d\Gamma$ |

### 2.5-D shell element (efficient for a thin PCB)

Model the board as a 3-node linear triangle with thickness t, cooled on both faces. The element contributions are:

$$\mathbf{K}^e = t\,A_e\,\mathbf{B}^T k_\parallel \mathbf{B} \;+\; \frac{2h_{eff}A_e}{12}\begin{bmatrix}2&1&1\\1&2&1\\1&1&2\end{bmatrix}, \qquad \mathbf{F}^e = \frac{(q'' + 2h_{eff}T_\infty)A_e}{3}\begin{Bmatrix}1\\1\\1\end{Bmatrix}$$

The gradient matrix for the triangle is:

$$\mathbf{B} = \frac{1}{2A_e}\begin{bmatrix} y_2-y_3 & y_3-y_1 & y_1-y_2\\ x_3-x_2 & x_1-x_3 & x_2-x_1\end{bmatrix}$$

Component junctions are attached as lumped nodes linked to the board node by conductances G_jb = 1/(R_θjc + R_solder + R_via).

### Nonlinearity

Conduction loss depends on R_ds(T), so the system is solved with Picard iteration:

$$\mathbf{K}\,\mathbf{T}^{(n+1)} = \mathbf{F}\big(\mathbf{T}^{(n)}\big), \qquad \text{until } \|\mathbf{T}^{(n+1)}-\mathbf{T}^{(n)}\|_\infty < 0.01\ \text{K}$$

This converges in about 4–6 iterations, because the loop gain 2I²R_ds,25·α·R_θ is much less than 1.

### Transient form (for burst analysis)

$$\mathbf{C}\dot{\mathbf{T}} + \mathbf{K}\mathbf{T} = \mathbf{F}, \qquad C_{ij} = \int_\Omega \rho c_p N_i N_j\, d\Omega$$

Integrate with backward Euler or Crank–Nicolson.

---

## 4. Steady-State Solution