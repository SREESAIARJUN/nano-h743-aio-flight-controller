# Thermal Model: 4-in-1 ESC at 80 A Total Burst (Cycle 20)

## 0. Baseline Assumptions

I don't have the earlier cycles' parameters, so I used the representative values below. They are typical of a 30.5 mm-mount 4-in-1 board. Substitute your own and the equations carry through unchanged.

| Parameter | Symbol | Value |
|---|---|---|
| Battery (6S, full charge) | V_bus | 25.2 V |
| Current per channel (80 A / 4) | I_ph | 20 A |
| MOSFETs | — | 24 (6 per channel, 3 half-bridges) |
| On-resistance at 25 °C | R_ds,25 | 1.5 mΩ |
| R_ds temperature coefficient | α | 0.006 K⁻¹ |
| PWM frequency | f_sw | 48 kHz |
| Rise + fall time | t_r + t_f | 40 ns |
| Dead time | t_d | 300 ns |
| Body-diode forward voltage | V_f | 0.8 V |
| Reverse-recovery charge | Q_rr | 50 nC |
| PCB | — | 36 × 36 mm, 6 layers, 2 oz copper, 1.6 mm thick, ~70 % copper fill |
| Ambient | T_∞ | 30 °C |

---

## 1. Dissipation Physics (Heat Sources)

### 1.1 Temperature-dependent conduction loss

In trapezoidal or synchronous commutation, the phase current flows through two FETs in series: the high side of one leg and the low side of another.

$$
R_{ds}(T_j)=R_{ds,25}\,[1+\alpha(T_j-25)]
$$

$$
P_{cond}=I_{ph}^2\cdot 2R_{ds}(T_j)
$$

- At 25 °C: 400 × 2 × 1.5 mΩ = 1.20 W per channel.
- At T_j ≈ 100 °C (R_ds × 1.45): **1.74 W per channel**.

### 1.2 Switching loss (hard-switched edge)

$$
P_{sw}=\tfrac12 V_{bus} I_{ph}(t_r+t_f)f_{sw}=0.5(25.2)(20)(40\,\text{ns})(48\,\text{kHz})=\mathbf{0.48\ W}
$$

### 1.3 Reverse recovery and dead time

$$
P_{rr}=V_{bus}Q_{rr}f_{sw}=\mathbf{0.06\ W}
$$

$$
P_{dt}=2V_fI_{ph}t_df_{sw}=\mathbf{0.46\ W}
$$

### 1.4 Gate drive

$$
P_g = Q_gV_{gs}f_{sw} \approx 0.02\ \text{W}
$$

This is mostly dissipated in the driver IC, so it is neglected here.

### 1.5 Copper Joule heating (distributed volumetric source)

$$
q_{Cu}'''=\rho_{Cu}(T)\,|\mathbf J|^2,\qquad \rho_{Cu}(T)=1.72\times10^{-8}[1+0.0039(T-20)]\ \Omega\,\text{m}
$$

- Per-channel phase planes: about 0.5 mΩ → **0.2 W**.
- Shared bus (80 A through about 0.2 mΩ of pads and planes) → **1.28 W**.

### 1.6 Logic and regulators

The MCUs and the 5 V/10 V buck regulators add about **0.5 W**.

### 1.7 Total loss budget (at T_j ≈ 100 °C)

| Source | Per channel | Board total |
|---|---|---|
| Conduction | 1.74 W | 6.96 W |
| Switching + recovery + dead time | 1.00 W | 4.00 W |
| Phase copper | 0.20 W | 0.80 W |
| Bus copper | — | 1.28 W |
| Logic | — | 0.50 W |
| **Total Q** | **2.94 W** | **≈ 13.5 W** |

Because conduction loss rises with T_j, **Q = Q(T)**. This makes the problem nonlinear, and it is solved iteratively in §4.

---

## 2. Governing Conduction Equations

### 2.1 Strong form (steady state, anisotropic)

$$
\nabla\cdot\big(\mathbf k\,\nabla T\big)+q'''(\mathbf x,T)=0\quad\text{in }\Omega
$$

$$
\mathbf k=\text{diag}(k_{xy},k_{xy},k_z)
$$

### 2.2 Effective PCB conductivities (laminate homogenization)

The effective copper thickness is t_Cu = 6 layers × 70 µm × 0.7 fill = 0.294 mm. The FR4 thickness is t_FR4 = 1.306 mm, with k_FR4 = 0.3 W/m·K and k_Cu = 390 W/m·K.

In-plane (layers act in parallel):

$$
k_{xy}=\frac{\sum k_it_i}{\sum t_i}=\frac{0.294(390)+1.306(0.3)}{1.6}\approx \mathbf{72\ W/m\,K}
$$

Through-plane (layers act in series):

$$
k_{z}=\frac{\sum t_i}{\sum t_i/k_i}\approx \mathbf{0.37\ W/m\,K}
$$

Under the FET pads, use a via-field correction:

$$
k_z^{via}=k_z+\phi_{via}k_{Cu}
$$

where φ_via is the copper area fraction of the via array. Plated vias with φ ≈ 3 % give k_z ≈ 12 W/m·K.

### 2.3 Boundary conditions

Top and bottom surfaces (Robin condition, convection plus linearized radiation):

$$
-\mathbf n\cdot\mathbf k\nabla T = h_{eff}(T-T_\infty),\qquad h_{eff}=h_c+h_r
$$

$$
h_r=\varepsilon\sigma(T^2+T_\infty^2)(T+T_\infty)\approx 4\varepsilon\sigma \bar T^3\approx 6\ \text{W/m}^2\text{K}\quad(\varepsilon=0.9)
$$

Convection coefficient h_c by condition:

| Condition | h_c (W/m²·K) |
|---|---|
| Still air | 10–15 |
| Hover propwash | 50–100 |
| Forward flight | 100–200 |

Mounting standoffs (Dirichlet-like or contact-resistance sink to the frame):

$$
-\mathbf n\cdot\mathbf k\nabla T=\frac{T-T_{frame}}{R''_{contact}}
$$

Component-to-board coupling (lumped, applied at the pad nodes):

$$
T_j=T_{pad}+P_{FET}R_{\theta jc}
$$

---

## 3. Finite-Element Formulation

### 3.1 Weak form

Multiply the strong form by a test function w and integrate by parts. Find T ∈ H¹(Ω) such that, for all w,

$$
\int_\Omega \nabla w\cdot\mathbf k\nabla T\,d\Omega+\int_{\Gamma_R}h_{eff}\,wT\,d\Gamma
=\int_\Omega w\,q'''\,d\Omega+\int_{\Gamma_R}h_{eff}T_\infty w\,d\Gamma
$$

### 3.2 Galerkin discretization

Approximate the temperature with shape functions: T ≈ Σ N_a T_a. Each element contributes:

$$
K^e_{ab}=\int_{\Omega^e}(\nabla N_a)^{\!T}\mathbf k\,\nabla N_b\,d\Omega
\qquad
H^e_{ab}=\int_{\Gamma^e_R}h_{eff}N_aN_b\,d\Gamma
$$

$$
F^e_a=\int_{\Omega^e}N_a q'''\,d\Omega+\int_{\Gamma^e_R}h_{eff}T_\infty N_a\,d\Gamma
$$

For 8-node hexahedra, evaluate these with 2×2×2 Gauss quadrature:

$$
\mathbf K^e=\sum_g \mathbf B_g^T\mathbf k\mathbf B_g\,|\mathbf J_g|w_g
$$

where B_g holds the shape-function gradients and J_g is the Jacobian at Gauss point g.

### 3.3 Global nonlinear system

$$
\big[\mathbf K+\mathbf H(\mathbf T)\big]\mathbf T=\mathbf F(\mathbf T)
$$

The nonlinearity enters through h_r(T), R_ds(T_j), and ρ_Cu(T).

### 3.4 Solving the nonlinearity (Picard iteration)

1. Start with T⁽⁰⁾ = T_∞.
2. Evaluate the heat sources and h_r at the current temperature.
3. Solve the linear system for the next iterate:

$$
\mathbf T^{(n+1)}=\big[\mathbf K+\mathbf H(\mathbf T^{(n)})\big]^{-1}\mathbf F(\mathbf T^{(n)})
$$

4. Repeat until the update is small:

$$
\|\mathbf T^{(n+1)}-\mathbf T^{(n)}\|_\infty<0.01\ \text{K}
$$

Convergence is typically reached in 4–6 iterations. If dQ/dT · R_θ approaches 1, switch to Newton's method, which adds the Jacobian term ∂F/∂T. That condition is the onset of **thermal runaway**.

### 3.5 Meshing and model reduction

**Meshing guidance:**

- Use at least 2 elements through each copper layer under the FETs.
- Size elements ≤ 0.25 mm near the pads.
- Coarsen to about 2 mm at the board edges.

**Reduced 2.5-D plate model.** Because the board is thin, the problem collapses to a 2-D plate with a fin-like loss term (thickness t):

$$
t\,k_{xy}\nabla^2_{xy}T-(h_{top}+h_{bot})(T-T_\infty)+q''(x,y)=0
$$

Its characteristic spreading length is:

$$