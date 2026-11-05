# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the earlier context for "Cycle 11," so this formulation starts from stated baseline assumptions. Swap in your actual values (MOSFET part, stack-up, f_sw, airflow) and the equations carry through unchanged.

---

## 0. Baseline Assumptions

| Parameter | Value |
|---|---|
| Board | 36 × 36 mm, 1.6 mm, 6-layer, 2 oz (70 µm) Cu, ~70% Cu fill |
| Supply | 6S Li-ion, V_bus = 22.2 V |
| Current | 80 A total, so **20 A per channel** (4 channels) |
| Bridge | 3-phase, 6 N-MOSFETs per channel (24 total) |
| MOSFET | R_DS(on) = 1.5 mΩ at 25 °C, α ≈ 0.006 K⁻¹, t_r + t_f ≈ 30 ns, Q_g ≈ 50 nC, Q_rr ≈ 50 nC, V_f ≈ 0.8 V |
| PWM | f_sw = 48 kHz, dead time t_dt = 200 ns |
| Shunt | 0.5 mΩ per channel |
| Ambient | T_∞ = 30 °C |

---

## 1. Heat-Generation Physics (Source Terms)

### 1.1 Conduction loss (temperature-dependent)

At any instant two FETs carry the phase current (one high-side, one low-side):

$$
P_{cond}=2\,I_{ph,rms}^2\,R_{DS(on)}(T_j),\qquad R_{DS(on)}(T_j)=R_{25}\left[1+\alpha\,(T_j-25)\right]
$$

At T_j ≈ 105 °C, R_DS(on) ≈ 2.2 mΩ:

$$
P_{cond}=2(20)^2(2.2\times10^{-3})=1.76\ \text{W/channel}
$$

### 1.2 Switching loss (hard-switched leg)

$$
P_{sw}=\tfrac12 V_{bus} I\,(t_r+t_f)\,f_{sw}=\tfrac12(22.2)(20)(30\,\text{ns})(48\,\text{kHz})=0.32\ \text{W}
$$

### 1.3 Dead-time body-diode conduction

$$
P_{dt}=V_f\,I\,(2t_{dt})\,f_{sw}=0.8(20)(400\,\text{ns})(48\,\text{kHz})=0.31\ \text{W}
$$

### 1.4 Reverse recovery and gate drive

$$
P_{rr}=Q_{rr}V_{bus}f_{sw}\approx0.05\ \text{W},\qquad P_g=n\,Q_gV_{GS}f_{sw}\approx0.07\ \text{W}
$$

### 1.5 Shunt and copper I²R

$$
P_{sh}=I^2R_{sh}=0.20\ \text{W},\qquad P_{Cu}=\sum I_k^2\frac{\rho_{Cu}(T)\,\ell_k}{w_k t_k}\approx0.30\ \text{W}
$$

### 1.6 Loss budget

| Source | Per channel (W) | Board (W) |
|---|---|---|
| Conduction | 1.76 | 7.04 |
| Switching | 0.32 | 1.28 |
| Dead-time diode | 0.31 | 1.24 |
| Q_rr + gate | 0.12 | 0.48 |
| Shunt + traces | 0.50 | 2.00 |
| MCU / LDO / gate driver | — | 0.30 |
| **Total** | **≈3.0** | **≈12.3 W** |

This corresponds to ~99.3% efficiency on a 1.78 kW electrical input.

---

## 2. Governing Conduction Equation

Transient, anisotropic heat diffusion:

$$
\rho c_p\frac{\partial T}{\partial t}=\nabla\cdot(\mathbf{k}\,\nabla T)+\dot q'''(\mathbf{x},T),\qquad
\mathbf{k}=\text{diag}(k_{xy},k_{xy},k_z)
$$

The steady state sets ∂T/∂t = 0.

### 2.1 Effective PCB conductivity

Treat the copper layers as a parallel stack (in-plane) and a series stack (through-plane). Effective Cu thickness: 6 × 70 µm × 0.7 ≈ 0.29 mm; FR4 ≈ 1.31 mm.

$$
k_{xy}=\frac{\sum k_i t_i}{\sum t_i}=\frac{385(0.29)+0.3(1.31)}{1.6}\approx 70\ \text{W/m·K}
$$

$$
k_z=\frac{\sum t_i}{\sum t_i/k_i}=\frac{1.6}{0.29/385+1.31/0.3}\approx0.37\ \text{W/m·K}
$$

Under FET pads with thermal vias (0.3 mm drill, 25 µm plating, 1 mm pitch, about 2.3% Cu area fraction), the local through-plane conductivity rises to k_z,via ≈ 0.023·385 + 0.3 ≈ **9.2 W/m·K**.

### 2.2 Boundary conditions

**Forced convection (prop wash).** Laminar flat-plate correlation with v = 8 m/s and L = 36 mm:

$$
Re_L=\frac{vL}{\nu}=\frac{8(0.036)}{1.6\times10^{-5}}=1.8\times10^4,\qquad
\bar h=0.664\,\frac{k_{air}}{L}Re_L^{1/2}Pr^{1/3}\approx 57\ \text{W/m}^2\text{K}
$$

**Radiation (linearized):**

$$
h_r=4\varepsilon\sigma T_m^3=4(0.9)(5.67\times10^{-8})(330)^3\approx7.3\ \text{W/m}^2\text{K}
$$

**Combined Robin condition on the top and bottom faces:**

$$
-\mathbf{n}\cdot\mathbf{k}\nabla T=h_{eff}(T-T_\infty),\qquad h_{eff}\approx 64\ \text{W/m}^2\text{K}
$$

In a sealed stack with no airflow, h_eff drops to about 15 W/m²K.

**Mounting.** Model the standoffs and frame contact as conductances G_mount ≈ 0.05 W/K each at the four M3 holes. The board edges are adiabatic.

---

## 3. Finite-Element Formulation

### 3.1 2.5-D plate reduction

The Biot number (h t / k_xy ≈ 10⁻³) is very small, so through-thickness gradients are negligible except under the FETs. Integrating through the thickness gives:

$$
\nabla_{xy}\cdot\left(k_{xy}t\,\nabla_{xy}T\right)-(h_{top}+h_{bot})(T-T_\infty)+\dot q''(x,y)=0
$$

### 3.2 Weak (Galerkin) form

Find T ∈ H¹(Ω) such that, for all test functions w:

$$
\int_\Omega k_{xy}t\,\nabla w\cdot\nabla T\,d\Omega+\int_\Omega h_{\Sigma}\,wT\,d\Omega
=\int_\Omega w\,\dot q''\,d\Omega+\int_\Omega h_\Sigma\,wT_\infty\,d\Omega+\sum_m G_m w(\mathbf{x}_m)(T_\infty-T)
$$

### 3.3 Discretization

Interpolate T ≈ Σ N_i T_i to obtain:

$$
\left[\mathbf{K}_c+\mathbf{K}_h+\mathbf{K}_m\right]\mathbf{T}=\mathbf{F}_q+\mathbf{F}_h
$$

with:

$$
\mathbf{K}_c^e=\int_{\Omega_e}k_{xy}t\,\mathbf{B}^T\mathbf{B}\,d\Omega,\qquad
\mathbf{K}_h^e=\int_{\Omega_e}h_\Sigma\,\mathbf{N}^T\mathbf{N}\,d\Omega
$$

$$
\mathbf{F}_q^e=\int_{\Omega_e}\dot q''\,\mathbf{N}^T d\Omega,\qquad
\mathbf{F}_h^e=\int_{\Omega_e}h_\Sigma T_\infty\,\mathbf{N}^T d\Omega
$$

**Linear triangle (CST) closed forms**, with element area A and b_i = y_j − y_k, c_i = x_k − x_j:

$$
\mathbf{K}_c^e=\frac{k_{xy}t}{4A}\begin{bmatrix}b_ib_i+c_ic_i & b_ib_j+c_ic_j & b_ib_k+c_ic_k\\ & b_jb_j+c_jc_j & b_jb_k+c_jc_k\\ \text{sym} & & b_kb_k+c_kc_k\end{bmatrix},
\qquad
\mathbf{K}_h^e=\frac{h_\Sigma A}{12}\begin{bmatrix}2&1&1\\1&2&1\\1&1&2\end{bmatrix}
$$

$$
\mathbf{F}_q^e=\frac{\dot q''A}{3}\{1,1,1\}^T
$$

### 3.4 Electro-thermal nonlinearity (Picard iteration)

Because q̇'' depends on T through R_DS(on)(T), iterate:

$$
\mathbf{K}\,\mathbf{T}^{(n+1)}=\mathbf{F}\!\left(\mathbf{T}^{(n)}\right),\qquad \text{stop when }\ \|\mathbf{T}^{(n+1)}-\mathbf{T}^{(n)}\|_\infty<0.01\ \text{K}
$$

**Thermal runaway criterion.** Runaway occurs if the loop gain reaches unity:

