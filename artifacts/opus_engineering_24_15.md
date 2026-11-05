# Thermal Model: 4-in-1 ESC at 80 A Total Burst (Cycle 24)

I don't have the parameters from earlier cycles, so this model uses the assumptions in the table below. Replace them with your board's values and the equations still apply.

| Parameter | Assumed value |
|---|---|
| Board | 30.5 × 30.5 mm (20×20 mount variant scales down), 6-layer, 2 oz Cu, 1.6 mm FR-4 |
| Supply | 6S LiPo, 25.2 V max |
| Current | 80 A total, so **20 A per channel** |
| Power stage | 24 MOSFETs (6 per channel), PQFN 3.3×3.3, R_ds(on) = 1.5 mΩ at 25 °C |
| Switching | f_sw = 48 kHz, complementary PWM, t_r + t_f = 20 ns, dead time 200 ns |
| Ambient | T∞ = 25 °C |

---

## 1. Heat-Source Physics (Volumetric Generation q̇)

### 1.1 MOSFET conduction loss (temperature-coupled)
With trapezoidal commutation and synchronous freewheeling, two FETs per channel carry the phase current at any instant:

$$P_{cond,ch} = 2\,I_{ph}^2\,R_{ds}(T_j),\qquad R_{ds}(T)=R_{25}\,[1+\alpha_R(T-25)],\ \ \alpha_R\approx0.0045\ \text{K}^{-1}$$

- At 25 °C: 2 × 20² × 1.5 mΩ = **1.20 W/channel**
- At 100 °C: R_ds ≈ 2.0 mΩ, giving ≈ **1.61 W/channel**

### 1.2 Switching loss
$$P_{sw}=\tfrac12 V_{bus} I_{ph}(t_r+t_f)f_{sw}=\tfrac12(25.2)(20)(20\,\text{ns})(48\,\text{kHz})=0.24\ \text{W}$$

### 1.3 Body-diode dead-time loss
$$P_{dt}=V_f I_{ph}(2t_{dead})f_{sw}=0.8\cdot20\cdot400\,\text{ns}\cdot48\,\text{kHz}=0.31\ \text{W}$$

### 1.4 Reverse recovery and gate drive
$$P_{rr}=Q_{rr}V_{bus}f_{sw}\approx0.06\ \text{W},\qquad P_g=Q_gV_{gs}f_{sw}\cdot n_{sw}\approx0.05\ \text{W}$$

### 1.5 Board-level sources
| Source | Model | Loss |
|---|---|---|
| Power planes and bus (80 A) | I²R_Cu(T), R ≈ 0.25 mΩ, α_Cu = 0.0039 | 1.6 W at 25 °C |
| Current shunt(s) | I²R_sh, 0.25 mΩ | 1.6 W |
| MCUs, gate drivers, LDO | fixed | 0.4 W |

### 1.6 Loss budget at 25 °C
| | Per channel | ×4 / total |
|---|---|---|
| Conduction | 1.20 W | 4.80 W |
| Switching + dead time + Qrr + gate | 0.66 W | 2.64 W |
| Bus copper | | 1.60 W |
| Shunt | | 1.60 W |
| Logic | | 0.40 W |
| **P₀ (at 25 °C)** | | **≈ 11.0 W** |

Linearized temperature sensitivity:

$$P(T)=P_0+a\,(T-T_\infty),\qquad a = 4.8(0.0045)+1.6(0.0039)\approx\mathbf{0.028\ W/K}$$

---

## 2. Governing Conduction Equation

### 2.1 Transient anisotropic heat equation
$$\rho c_p\frac{\partial T}{\partial t}=\nabla\cdot\big(\mathbf{k}\,\nabla T\big)+\dot q(\mathbf{x},T),\qquad \mathbf{k}=\mathrm{diag}(k_{xy},k_{xy},k_z)$$

### 2.2 Effective PCB conductivity (layered composite)
The board has 6 × 70 µm Cu = 0.42 mm and 1.18 mm FR-4 (k = 0.3 W/m·K). Assume 70 % copper fill.

**In-plane (parallel layers):**
$$k_{xy}=\phi\frac{\sum k_i t_i}{\sum t_i}=0.7\cdot\frac{385(0.42)+0.3(1.18)}{1.6}\approx\mathbf{71\ W/m\cdot K}$$

**Through-plane (series layers):**
$$k_z=\frac{\sum t_i}{\sum t_i/k_i}=\frac{1.6}{0.42/385+1.18/0.3}\approx\mathbf{0.41\ W/m\cdot K}$$

Under the FET pads, thermal via arrays raise the local k_z. For n vias with plating thickness t_p and drill diameter d:

$$k_{z,via}\approx k_{FR4}+\frac{n\,\pi\,d\,t_p\,k_{Cu}}{A_{pad}}\ \ \Rightarrow\ \ \sim 20\text{–}40\ \text{W/m·K}$$

### 2.3 Boundary conditions
**Convective and radiative surfaces (Robin):**
$$-\mathbf{n}\cdot\mathbf{k}\nabla T = h_c(T-T_\infty)+\varepsilon\sigma(T^4-T_\infty^4)$$

Linearize the radiation term with
$$h_r=\varepsilon\sigma(T^2+T_\infty^2)(T+T_\infty)\approx 7\ \text{W/m}^2\text{K}\quad(\varepsilon=0.9)$$

**Prop-wash forced convection (laminar flat plate):** take U = 10 m/s and L = 0.03 m, so Re ≈ 1.9×10⁴.

$$h_c=0.664\frac{k_{air}}{L}Re^{1/2}Pr^{1/3}=0.664\cdot\frac{0.026}{0.03}\cdot137\cdot0.89\approx\mathbf{70\ W/m^2K}$$

For a stack enclosed in the frame with stagnant air, h_c ≈ 10–15 W/m²K.

**Other boundaries:**
- **Mounting standoffs:** Robin condition with contact conductance h_contact (nylon standoffs are nearly adiabatic).
- **Motor and battery wires:** fin boundary, with $q = \sqrt{hPkA}\,\theta_b\tanh(mL)$.

---

## 3. Finite-Element Formulation

### 3.1 Weak form (Galerkin)
Find T ∈ H¹(Ω) such that for all test functions w:

$$\int_\Omega \nabla w\cdot\mathbf{k}\nabla T\,d\Omega+\int_{\Gamma_R}(h_c+h_r)\,wT\,d\Gamma=\int_\Omega w\,\dot q\,d\Omega+\int_{\Gamma_R}(h_c+h_r)\,wT_\infty\,d\Gamma$$

### 3.2 Discretization
Approximate $T\approx\sum N_j T_j$. Element matrices:

$$\mathbf{K}^e=\int_{\Omega^e}\mathbf{B}^T\mathbf{k}\mathbf{B}\,d\Omega+\int_{\Gamma^e_R}h\,\mathbf{N}^T\mathbf{N}\,d\Gamma,\qquad \mathbf{C}^e=\int_{\Omega^e}\rho c_p\mathbf{N}^T\mathbf{N}\,d\Omega$$

$$\mathbf{f}^e=\int_{\Omega^e}\mathbf{N}^T\dot q