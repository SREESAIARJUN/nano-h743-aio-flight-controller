# Thermal Model: 4-in-1 ESC at 80 A Total Burst (Cycle 16)

I don't have the parameters from earlier design cycles, so this formulation uses stated baseline assumptions. Replace them with your Cycle-16 values and the equations carry through unchanged.

## 0. Baseline Assumptions

| Parameter | Value |
|---|---|
| Board | 30.5 × 30.5 mm, 1.6 mm, 6-layer, 2 oz (70 µm) Cu |
| Supply | 6S LiPo, V_bus ≈ 25 V (fresh pack) |
| Channels | 4 motors × 3 half-bridges = 24 MOSFETs |
| Current | 80 A total → I = 20 A per motor (burst) |
| MOSFET | R_DS(on),25 = 1.5 mΩ; α = 0.005 K⁻¹; R_θJC = 1.2 K/W |
| Switching | f_sw = 48 kHz; t_r + t_f = 40 ns; dead time t_dt = 200 ns; diode V_f = 0.8 V |
| Environment | T_∞ = 30 °C; prop-wash forced convection h_c ≈ 100 W/m²K; ε = 0.9 |

---

## 1. Dissipation Physics (Heat Sources)

### 1.1 MOSFET conduction loss (temperature-dependent)

$$R_{DS}(T_j) = R_{DS,25}\,[1+\alpha(T_j-25)]$$

With 6-step commutation and synchronous rectification, two phases conduct at any instant. The high-side duty D and low-side duty (1−D) sum to 1 on each leg, so per motor:

$$P_{cond} = 2\,I^2 R_{DS}(T_j)$$

- At 25 °C: 2·20²·1.5 mΩ = **1.20 W**
- At T_j ≈ 100 °C (×1.375): **1.65 W**

### 1.2 Switching loss (hard-switched leg)

$$P_{sw} = \tfrac12 V_{bus} I (t_r+t_f) f_{sw} = 0.5·25·20·40\text{ns}·48\text{kHz} = \mathbf{0.48\ W}$$

### 1.3 Output capacitance loss

$$P_{oss} = \tfrac12 C_{oss}V_{bus}^2 f_{sw} \approx 0.5·1\text{nF}·625·48\text{k} = 0.015\text{ W}$$

### 1.4 Dead-time body-diode conduction

$$P_{dt} = V_f\, I\, (2t_{dt})\, f_{sw} = 0.8·20·400\text{ns}·48\text{k} = \mathbf{0.31\ W}$$

### 1.5 Gate drive (mostly dissipated in the driver)

$$P_g = Q_g V_{GS} f_{sw}\cdot n_{sw} \approx 30\text{nC}·10\text{V}·48\text{k}·2 \approx 0.03\text{ W}$$

### 1.6 Board-level sources

- **Copper bus Joule heating:** P_Cu = I_tot² · R_bus. With R_bus ≈ 0.3 mΩ for the plane plus pads, 6400 · 0.3 mΩ = **1.9 W**. As a volumetric source:
  $$q_{Cu}''' = \frac{J^2}{\sigma_{Cu}(T)}, \qquad \sigma_{Cu}(T)=\frac{\sigma_{20}}{1+0.00393(T-20)}$$
- **Capacitor ESR:** I_rip,rms ≈ I√(D(1−D)) ≈ 10 A per motor, partially interleaved. This gives ≈ **0.5 W** total.
- **MCU and regulators:** ≈ **0.3 W**

### 1.7 Loss budget

| Source | Per motor | Total |
|---|---|---|
| Conduction (hot) | 1.65 W | 6.6 W |
| Switching + C_oss | 0.50 W | 2.0 W |
| Dead time | 0.31 W | 1.2 W |
| Gate | 0.03 W | 0.1 W |
| Cu bus | — | 1.9 W |
| Caps + logic | — | 0.8 W |
| **Total Q** | | **≈ 12.6 W** (≈ 6.3 % of ~200 W delivered) |

---

## 2. Governing Conduction Equations

### 2.1 Strong form (transient, anisotropic)

$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{k}\nabla T) + q'''(\mathbf{x},T) \quad \text{in } \Omega$$

At steady state (∂T/∂t = 0):

$$\nabla\cdot(\mathbf{k}\nabla T) + q'''(\mathbf{x},T) = 0$$

### 2.2 Effective PCB conductivity tensor

The board is a laminate of copper and FR-4 layers, modeled as a homogenized orthotropic solid:

$$\mathbf{k}=\begin{bmatrix}k_\parallel&0&0\\0&k_\parallel&0\\0&0&k_\perp\end{bmatrix},\quad
k_\parallel = \frac{\sum k_i t_i \phi_i}{\sum t_i},\quad
k_\perp = \frac{\sum t_i}{\sum t_i/k_i}$$

Here φ_i is the copper coverage fraction. With 0.42 mm Cu (k = 385, φ ≈ 0.7) and 1.18 mm FR-4 (k = 0.3):

- **k_∥ ≈ 71 W/m·K**
- **k_⊥ ≈ 0.41 W/m·K** without vias

Thermal via arrays under the MOSFETs are a parallel path:

$$k_{\perp,via} = k_{FR4}(1-f_v) + k_{Cu} f_v$$

Plated vias at ~10 % area fraction raise k_⊥ to about **≈ 39 W/m·K** locally.

### 2.3 Boundary conditions

- **Convection plus linearized radiation** on top and bottom surfaces Γ_c:
  $$-\mathbf{n}\cdot\mathbf{k}\nabla T = h_{eff}(T-T_\infty),\qquad h_{eff}=h_c + \underbrace{4\varepsilon\sigma T_m^3}_{h_r\approx 7\,\text{W/m}^2\text{K}}$$
- **Conduction into the motor wires and battery leads** (fin BC):
  $$q_{lead} = \sqrt{hPkA}\,\tanh(mL)\,(T_b-T_\infty)$$
- **Standoffs:** Robin condition with a contact conductance h_contact.
- **Junction nodes:** Each MOSFET is a lumped node coupled to its pad footprint.
  $$T_j = T_{pad} + P_{FET}(R_{\theta JC} + R_{solder})$$

---

## 3. Finite-Element Formulation

### 3.1 Weak form

Find T ∈ H¹(Ω) such that for all test functions w:

$$\int_\Omega \nabla w\cdot\mathbf{k}\nabla T\,d\Omega + \int_{\Gamma_c} h_{eff}\,wT\,d\Gamma = \int_\Omega w\,q'''\,d\Omega + \int_{\Gamma_c} h_{eff}\,w\,T_\infty\,d\Gamma$$

### 3.2 Galerkin discretization

Set T ≈ Σ N_a T_a, with 8-node hexahedra for the 3D stack or layered shell elements for a 2.5D model.

$$\mathbf{K}_e = \int_{\Omega_e}\mathbf{B}^T\mathbf{k}\mathbf{B}\,d\Omega + \int_{\Gamma_e} h_{eff}\mathbf{N}^T\mathbf{N}\,d\Gamma$$

$$\mathbf{F}_e = \int_{\Omega_e}\mathbf{N}^T q'''\,d\Omega + \int_{\Gamma_e}h_{eff}T_\infty\mathbf{N}^T\,d\Gamma$$

Here B = ∇N. Gauss quadrature uses 2×2×2 points for the volume terms and 2×2 for the faces.

Assembled system:

$$\mathbf{K}(\mathbf{T})\,\mathbf{T} = \mathbf{F}(\mathbf{T})$$

The transient form adds a capacitance matrix:

$$\mathbf{C}\dot{\mathbf{T}} + \mathbf{K}\mathbf{T} = \mathbf{F},\qquad \mathbf{C}_e=\int\rho c_p\mathbf{N}^T\mathbf{N}\,d\Omega$$

Integrate it with backward Euler or Crank–Nicolson.

### 3.3 Nonlinear coupling (electro-thermal)

The system is nonlinear because both P_cond and the copper resistivity depend on T. Solve it by Picard iteration:

1. Assume T⁽⁰⁾ = T_∞.
2. Compute q'''(T⁽ⁿ⁾) from §1, using R_DS(T_j⁽ⁿ⁾) and σ_Cu(T⁽ⁿ⁾).
3. Solve K T⁽ⁿ⁺¹⁾ = F.
4. Repeat until ‖T⁽ⁿ⁺¹⁾