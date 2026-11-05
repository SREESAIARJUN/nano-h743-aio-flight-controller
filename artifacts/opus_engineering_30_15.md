# Thermal Model of a 4‑in‑1 ESC at 80 A Total Burst Current (Cycle 30)

## 0. Assumptions

I don't have the earlier cycles, so this uses typical design values. Replace any of them with your actual numbers.

| Parameter | Symbol | Value |
|---|---|---|
| Board | — | 37 × 37 mm, 1.6 mm thick, 6 layers of 2 oz Cu (30.5 mm mount pattern) |
| Bus voltage | V_bus | 22.2 V (6S nominal) |
| Total current | I_tot | 80 A, so I_m = 20 A per motor |
| MOSFETs | — | 24 (3 half‑bridges × 4), DFN3.3 package |
| MOSFET on‑resistance at 25 °C | R_ds25 | 1.5 mΩ |
| R_ds temperature coefficient | α | 0.005 K⁻¹ |
| PWM frequency | f_sw | 48 kHz |
| Rise + fall time | t_r + t_f | 40 ns |
| Dead time | t_dt | 200 ns |
| Body‑diode forward voltage | V_f | 0.8 V |
| Gate charge, gate voltage | Q_g, V_gs | 30 nC, 10 V |
| Current shunts | — | 0.5 mΩ, one per motor |
| Ambient temperature | T_∞ | 25 °C |
| Commutation | — | Trapezoidal (6‑step): one high‑side and one low‑side FET conduct at any instant |

---

## 1. Dissipation Physics (Heat Sources)

### 1.1 MOSFET conduction loss
The on‑resistance rises with temperature:

$$R_{ds}(T)=R_{ds25}\,[1+\alpha(T-25)]$$

Two FETs conduct the motor current at any time, so per motor:

$$P_{cond,m}=2\,I_m^2\,R_{ds}(T)=2(20)^2(1.5\,\text{m}\Omega)[1+\alpha\Delta T]=1.20\,[1+\alpha\Delta T]\ \text{W}$$

### 1.2 Switching loss
$$P_{sw,m}=\tfrac12 V_{bus} I_m (t_r+t_f) f_{sw}=\tfrac12(22.2)(20)(40\text{ ns})(48\text{ kHz})=0.426\ \text{W}$$

### 1.3 Dead‑time (body‑diode) loss
$$P_{dt,m}=V_f\, I_m\,(2t_{dt})\,f_{sw}=0.8\cdot20\cdot400\text{ ns}\cdot48\text{ kHz}=0.307\ \text{W}$$

### 1.4 Gate‑drive loss
$$P_{g,m}=6\,Q_g V_{gs} f_{sw}\cdot\tfrac{1}{2}\approx0.03\ \text{W}$$

This is mostly dissipated in the driver, and is small.

### 1.5 Copper (Joule) loss in the power planes
The sheet resistance of 2 oz copper is

$$R_\square=\rho_{Cu}/t=1.72\times10^{-8}/70\,\mu\text{m}=0.246\ \text{m}\Omega/\square$$

Assume an effective bus path resistance of about 0.15 mΩ, with several layers in parallel. Then

$$P_{bus}=I_{tot}^2R_{bus}=80^2(0.15\,\text{m}\Omega)=0.96\ \text{W}$$

In the FEM this loss is not a lumped number. It is a distributed source, $q_J=\mathbf J\cdot\mathbf J/\sigma$, obtained from the electrical sub‑problem $\nabla\cdot(\sigma\nabla\phi)=0$.

### 1.6 Shunts and logic
- Shunts: $4\cdot I_m^2 R_{sh}=4(400)(0.5\,\text{m}\Omega)=0.80$ W
- MCUs and regulators: ≈ 0.30 W

### 1.7 Loss budget (before temperature correction)

| Source | Per motor | ×4 / board |
|---|---|---|
| Conduction (at 25 °C) | 1.20 W | 4.80 W |
| Switching | 0.426 W | 1.70 W |
| Dead‑time | 0.307 W | 1.23 W |
| Gate | 0.03 W | 0.12 W |
| Bus copper | — | 0.96 W |
| Shunts | — | 0.80 W |
| Logic | — | 0.30 W |
| **Total** | | **9.91 W + 0.024·ΔT W** |

---

## 2. Governing Conduction Equations

### 2.1 Transient heat equation (anisotropic)
$$\rho c_p\frac{\partial T}{\partial t}=\nabla\cdot(\mathbf{k}\,\nabla T)+q(\mathbf x,T)$$

### 2.2 Steady state
$$\nabla\cdot(\mathbf{k}\,\nabla T)+q(\mathbf x,T)=0 \quad\text{in }\Omega$$

### 2.3 Effective PCB conductivity tensor
The board is a copper/FR4 laminate:
- 6 × 70 µm copper = 0.42 mm, with k = 398 W/m·K
- 1.18 mm FR4, with k = 0.3 W/m·K

In‑plane, the layers act as parallel paths:

$$k_{xy}=\frac{\sum k_i t_i}{\sum t_i}=\frac{398(0.42)+0.3(1.18)}{1.6}=104.7\ \text{W/m·K}$$

Through‑plane, they act in series:

$$k_z=\frac{\sum t_i}{\sum t_i/k_i}=\frac{1.6}{0.42/398+1.18/0.3}=0.41\ \text{W/m·K}$$

$$\mathbf{k}=\mathrm{diag}(104.7,\,104.7,\,0.41)$$

Under the FET pads, thermal vias raise $k_z$ locally. Model this as a via‑array region with

$$k_z^{via}\approx k_{Cu}\,\phi_{via}+k_{FR4}(1-\phi_{via})$$

For example, a copper fill fraction $\phi_{via}\approx0.08$ gives $k_z^{via}\approx32$ W/m·K.

### 2.4 Boundary conditions
**Robin (convection plus linearized radiation)** on the top and bottom faces:

$$-\mathbf n\cdot\mathbf k\nabla T=h_{eff}(T-T_\infty),\qquad h_{eff}=h_c+h_r$$

The radiation term is linearized as

$$h_r=4\varepsilon\sigma\bar T^3=4(0.9)(5.67\times10^{-8})(330)^3\approx7.3\ \text{W/m}^2\text{K}$$

Convection depends on airflow:
- Propwash in a forced‑air stack: $h_c\approx50$, so $h_{eff}\approx57$
- Enclosed stack, natural convection only: $h_c\approx12$, so $h_{eff}\approx19$

**Other boundaries:**
- Standoff mounts: Robin condition through the screw/grommet, $R_{mount}\approx40$ K/W each.
- Motor wire pads: conduction into the wire as a fin, with $G_{wire}=\sqrt{hPkA_c}$.

---

## 3. Finite‑Element Formulation

### 3.1 Weak (Galerkin) form
Find $T\in H^1(\Omega)$ such that for all test functions $w$:

$$\int_\Omega \nabla w\cdot\mathbf k\nabla T\,d\Omega+\int_{\Gamma_R}h_{eff}\,wT\,d\Gamma=\int_\Omega wq\,d\Omega+\int_{\Gamma_R}h_{eff}T_\infty w\,d\Gamma$$

### 3.2 Discretization
Expand the temperature in shape functions, $T\approx\sum N_jT_j$:

$$[\mathbf K_c+\mathbf K_h]\{\mathbf T\}=\{\mathbf F_q\}+\{\mathbf F_h\}$$

with

$$K_{c,ij}=\int_\Omega\nabla N_i\cdot\mathbf k\nabla N_j\,d\Omega,\qquad K_{h,ij}=\int_{\Gamma}h N_iN_j\,d\Gamma$$

### 3.3 Element matrices (thin‑shell 3‑