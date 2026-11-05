# Electro-Thermal Model of a 4-in-1 ESC at 80 A Total Burst (Cycle 14)

I don't have the board-specific data from earlier cycles, so this formulation uses the parameters stated in §0. Replace them with your stackup, MOSFET datasheet and airflow values. The equations stay valid; only the numbers change.

---

## 0. Assumed Configuration

| Parameter | Value |
|---|---|
| Board | 37 × 37 mm (30.5 mm mount), 6-layer, 2 oz Cu all layers, ~70 % copper fill |
| Bus | 6S, V_bus ≈ 25 V (fully charged, worst case for switching loss) |
| Power stage | 4 channels × 3 half-bridges = 24 MOSFETs (5×6 / 3.3×3.3 class) |
| R_ds(on) | 1.5 mΩ at 25 °C, α_R ≈ 0.007 K⁻¹ |
| Q_g / Q_rr / C_oss | 50 nC / 50 nC / 1.5 nF |
| Switching | f_sw = 48 kHz, t_r + t_f ≈ 40 ns, dead time 100 ns |
| Modulation | Six-step with complementary (synchronous) PWM, duty D ≈ 0.9 at burst |
| Current | I_DC = 80 A total → 20 A per channel |
| Cooling | Prop-wash forced convection h_c ≈ 60 W/m²K, emissivity ε = 0.9 |
| Ambient | T_∞ = 30 °C (in-frame air) |

---

## 1. Heat-Generation Physics (Source Terms)

### 1.1 Phase vs. DC current
With synchronous six-step drive, the DC link sees the duty-weighted motor current:

$$I_{DC,ch} = D\, I_m \;\Rightarrow\; I_m = \frac{20}{0.9} \approx 22.2\ \text{A}$$

### 1.2 Conduction loss (temperature-dependent)
Two FETs are always in the current path (one per active leg, high or low side):

$$P_{cond}(T) = 2\, I_m^2\, R_{ds,25}\,[1+\alpha_R (T_j - 25)]$$

At 25 °C this is 2 × 22.2² × 1.5 mΩ = **1.48 W per channel**.

### 1.3 Switching loss (one PWM leg per channel)
$$P_{sw} = \tfrac12 V_{bus} I_m (t_r+t_f) f_{sw} = 0.5(25)(22.2)(40\text{ns})(48\text{k}) \approx 0.53\ \text{W}$$

### 1.4 Parasitic and secondary terms

| Term | Expression | Value per channel |
|---|---|---|
| C_oss | ½ C_oss V² f_sw | 0.02 W |
| Reverse recovery | Q_rr V f_sw | 0.06 W |
| Dead-time body-diode conduction | V_f I_m (2 t_dt) f_sw | 0.17 W |
| Gate drive (dissipated mostly in the driver) | 2 Q_g V_gs f_sw | 0.05 W |

### 1.5 Board-level totals
| Source | Expression | Total |
|---|---|---|
| Fixed FET losses (4 channels) | 4 × 0.83 W | 3.3 W |
| Current shunts | ≈ 4 × I_m² R_sh, R_sh ≈ 0.5 mΩ | ≈ 1.0 W |
| Bus copper (I²R of planes and pads) | 80² × ~0.15 mΩ | ≈ 1.0 W |
| MCUs and LDO/BEC | | ≈ 0.5 W |
| Conduction loss at 25 °C (4 channels) | 4 × 1.48 W | 5.9 W |

Collecting these:

$$P_{tot}(T) = \underbrace{5.8\,\text{W}}_{P_{fixed}} + \underbrace{5.9\,\text{W}}_{P_{c0}}\,[1+\alpha_R(T-25)]$$

### 1.6 Volumetric source for FEM
Each component footprint Ω_k becomes a heat-flux patch:

$$q''_k(\mathbf{x}) = \frac{P_k(T)}{A_k}\,\chi_{\Omega_k}(\mathbf{x})$$

Here χ is the indicator function of the footprint. Apply the patch to the top-layer nodes, or to the die volume in a full 3D model.

---

## 2. Governing Conduction Equations

### 2.1 Full 3D anisotropic form
$$\nabla\cdot(\mathbf{k}\,\nabla T) + \dot q = \rho c_p \frac{\partial T}{\partial t}, \qquad \mathbf{k} = \text{diag}(k_\parallel, k_\parallel, k_z)$$

### 2.2 Effective PCB properties (layered stackup)
In-plane conductivity uses a parallel rule; through-plane uses a series rule plus a via term:

$$K_\parallel = \sum_i k_i t_i \approx \phi_{Cu}\,(390)(6\times70\,\mu\text{m}) + 0.3(1.2\,\text{mm}) \approx 0.115\ \text{W/K}$$

$$k_z = \frac{t_{tot}}{\sum_i t_i/k_i} + \phi_{via}\, k_{Cu} \;\;(\approx 0.4\text{–}1.5\ \text{W/mK, depending on thermal-via density})$$

### 2.3 Reduced 2.5D plate equation
The board is thin (Biot number through the thickness ≪ 1), so integrate through z:

$$\nabla_\parallel\cdot\big(K_\parallel \nabla_\parallel T\big) - (h_{top}+h_{bot})(T - T_\infty) + q''(\mathbf{x},T) = C''\frac{\partial T}{\partial t}$$

### 2.4 Boundary conditions
- **Faces:** Robin condition, with radiation linearized about the operating temperature:
  $$h_r = 4\varepsilon\sigma \bar T^3 \approx 4(0.9)(5.67\times10^{-8})(350)^3 \approx 8.8\ \text{W/m}^2\text{K}$$
  $$h_{eff} = h_c + h_r \approx 69\ \text{W/m}^2\text{K}$$
- **Board edges:** adiabatic, $\partial T/\partial n = 0$.
- **Mounting standoffs and battery pads:** Robin condition with contact conductance $G_c$. The lead wires act as fins.

---

## 3. Finite-Element Formulation

### 3.1 Weak form (steady state)
Find $T \in H^1(\Omega)$ such that, for all test functions $w$:

$$\int_\Omega K_\parallel \nabla w\cdot\nabla T\, dA + \int_\Omega h_\Sigma\, w\,T\, dA = \int_\Omega w\, q''\, dA + \int_\Omega h_\Sigma\, w\, T_\infty\, dA$$

### 3.2 Galerkin discretization
Use the interpolation $T \approx \mathbf{N}\mathbf{T}_e$, with bilinear quads or linear triangles. Refine the mesh to about 0.25 mm under the FETs and about 1 mm elsewhere.

Element conductance matrix:
$$\mathbf{K}_e = \int_{\Omega_e} \mathbf{B}^T K_\parallel \mathbf{B}\, dA + \int_{\Omega_e} h_\Sigma \mathbf{N}^T\mathbf{N}\, dA, \qquad \mathbf{B}=\nabla\mathbf{N}$$

Element load vector:
$$\mathbf{f}_e = \int_{\Omega_e} \mathbf{N}^T q''(T)\, dA + \int_{\Omega_e} h_\Sigma T_\infty \mathbf{N}^T dA$$

For a linear triangle (area $A$), these evaluate in closed form:
$$\mathbf{K}_e = \frac{K_\parallel}{4A}\begin{bmatrix} b_ib_i + c_ic_i & \cdots \\ \vdots & \ddots\end{bmatr