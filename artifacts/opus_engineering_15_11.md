# Thermal Model of a 4‑in‑1 ESC at 80 A Total Burst Current

I don't have the parameters from earlier design cycles, so this formulation uses typical 4‑in‑1 values, listed in §0. Each number is a parameter you can replace. The equations stay valid when you do.

---

## 0. Assumed Baseline

| Parameter | Symbol | Value |
|---|---|---|
| Board size (30.5 mm mount) | L×W | 36 × 36 mm |
| Stack-up | | 4-layer, 2 oz Cu (70 µm/layer), 1.6 mm total, ~70 % Cu fill |
| Bus voltage (6S, full charge) | V_bus | 25.2 V |
| Total burst current | I_tot | 80 A, i.e. 20 A per channel |
| MOSFETs | | 6 per channel (24 total), R_ds,on = 1.5 mΩ at 25 °C, α ≈ 0.005 /K |
| PWM frequency | f_sw | 48 kHz |
| Rise and fall time | t_r, t_f | 20 ns each |
| Dead time | t_d | 300 ns |
| Body diode forward voltage | V_f | 0.8 V |
| Gate charge and drive voltage | Q_g, V_gs | 50 nC, 10 V |
| Current shunt | R_sh | 0.5 mΩ per channel |
| Ambient temperature | T_∞ | 25 °C |
| Convection with prop wash, both faces | h_c | 50 W/m²K |
| Linearized radiation (ε ≈ 0.9) | h_rad | ≈ 7 W/m²K |
| Total effective film coefficient | h | ≈ 57–60 W/m²K (use 60) |

---

## 1. Heat-Generation Physics (Source Terms)

### 1.1 Conduction loss (temperature-dependent)

In 6-step commutation, two FETs per channel conduct at any instant: one high-side and one low-side.

$$
P_{cond} = 2\, I_{ch}^2\, R_{ds,on}(T_j), \qquad R_{ds,on}(T_j)=R_{25}\,[1+\alpha (T_j-25)]
$$

With T_j ≈ 100 °C, R_ds,on ≈ 2.2 mΩ:

$$
P_{cond} = 2(20)^2(0.0022) \approx 1.76\ \text{W per channel}
$$

### 1.2 Switching loss (one switching device per channel)

$$
P_{sw} = \tfrac12 V_{bus} I_{ch}(t_r+t_f) f_{sw} = 0.5(25.2)(20)(40\text{ ns})(48\text{ kHz}) \approx 0.48\ \text{W}
$$

### 1.3 Dead-time and body-diode loss

$$
P_{dt} = V_f\, I_{ch}\, (2t_d)\, f_{sw} = 0.8(20)(600\text{ ns})(48\text{ kHz}) \approx 0.46\ \text{W}
$$

### 1.4 Gate-drive loss

$$
P_g = 2\, Q_g V_{gs} f_{sw} \approx 0.05\ \text{W}
$$

### 1.5 Shunt loss

$$
P_{sh} = I_{ch}^2 R_{sh} = 0.20\ \text{W}
$$

### 1.6 Distributed copper Joule heating

The current density field comes from solving the electrical conduction problem:

$$
\nabla\cdot(\sigma \nabla \phi)=0,\qquad q'''_{Cu} = \sigma |\nabla\phi|^2,\qquad \sigma(T)=\frac{\sigma_0}{1+0.0039(T-25)}
$$

As a lumped estimate, the bus plane resistance R_bus ≈ 0.25 mΩ gives:

$$
P_{Cu} \approx 80^2(0.00025) = 1.6\ \text{W}
$$

### 1.7 Loss budget

| Source | Per channel | ×4 / board |
|---|---|---|
| Conduction | 1.76 W | 7.04 W |
| Switching | 0.48 W | 1.94 W |
| Dead-time | 0.46 W | 1.84 W |
| Gate | 0.05 W | 0.19 W |
| Shunt | 0.20 W | 0.80 W |
| Bus copper | — | 1.60 W |
| MCU/LDO/drivers | — | 0.30 W |
| **Total P_tot** | **~2.95 W** | **≈ 13.7 W** |

Efficiency is about 1 − 13.7/(25.2·80) ≈ 99.3 %. The loss is small electrically, but large for a 13 cm² board.

---

## 2. Governing Conductive Heat-Transfer Equations

### 2.1 Full 3-D form (orthotropic, nonlinear)

$$
\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot\big(\mathbf{k}\,\nabla T\big) + q'''(\mathbf{x},T)
$$

The conductivity tensor is orthotropic:

$$
\mathbf{k}=\text{diag}(k_{xy},k_{xy},k_z)
$$

### 2.2 Effective laminate conductivity (rule of mixtures)

In-plane conductivity uses a parallel combination of layers:

$$
k_{xy}=\frac{\sum k_i t_i}{\sum t_i}=\frac{385(0.28)(0.7)+0.3(1.32)}{1.6}\approx 47\ \text{W/mK}
$$

Through-plane conductivity uses a series combination:

$$
k_z=\frac{\sum t_i}{\sum t_i/k_i}=\frac{1.6}{0.28/385+1.32/0.3}\approx 0.36\ \text{W/mK}
$$

Under the FET pads, thermal via arrays raise k_z locally. Model them as a homogenized region:

$$
k_{z,via}\approx k_z + \phi_{via}\,k_{Cu}
$$

For example, a via fill fraction φ_via of 5 % gives about 19 W/mK.

### 2.3 Boundary conditions

On the top and bottom faces (Robin condition, convection plus linearized radiation):

$$
-\mathbf{n}\cdot\mathbf{k}\nabla T = h\,(T-T_\infty),\qquad h = h_c + 4\varepsilon\sigma_{SB}\bar T^3
$$

The board edges are adiabatic:

$$
\mathbf{n}\cdot\nabla T=0
$$

At the mounting standoffs, optionally add a contact conductance h_mount (frame heat-sinking).

### 2.4 Reduced 2.5-D plate model (recommended for design iteration)

The Biot number in thickness is h t / k_z ≈ 0.27. The board is therefore not perfectly isothermal through its thickness. Even so, in-plane spreading dominates, so integrating over the thickness is acceptable:

$$
(\rho c_p t)_{eff}\frac{\partial T}{\partial t} = \nabla\cdot\big(k_{xy} t\,\nabla T\big) - 2h\,(T-T_\infty) + q''(x,y,T)
$$

Here q'' is the areal heat flux: FET footprints, shunts, and the copper Joule map.

---

## 3. Finite-Element Formulation

### 3.1 Weak form (Galerkin)

Find T ∈ H¹(Ω) such that, for all test functions w:

$$
\int_\Omega k_{xy}t\,\nabla w\cdot\nabla T\,dA + \int_\Omega 2h\,w\,T\,dA = \int_\Omega w\,q''\,dA + \int_\Omega 2h\,w\,T_\infty\,dA
$$

### 3.2 Discretization

Write T ≈ Σ N_j T_j, using bilinear quads or linear triangles, with a mesh refined to about 0.25 mm under the FETs.

The element conduction matrix is:

$$
\mathbf{K}^e = \int_{\Omega_e} \mathbf{B}^T (k_{xy}t)\,\mathbf{B}\,dA
$$

where B = ∇N.

The element convection matrix is:

$$
\mathbf{H}^e = \int_{\Omega_e} 2h\,\mathbf{N}^T\mathbf{N}\,dA
$$

The element load vector is:

$$
\mathbf{f}^e = \int_{\Omega_e}\mathbf{N}^T\big(q'' + 2hT_\infty\big)\,dA
$$

The element capacitance matrix is:

$$
\mathbf{C}^e = \int_{\Omega_e}(\rho c_p t)\,\mathbf{N}^T\mathbf{N}\,dA
$$

For a linear triangle, the explicit element matrices are:

$$
\mathbf{K}^e=\frac{k_{xy}t}{4A_e}\big[\mathbf{b}\mathbf{b}^T+\mathbf{c}\mathbf{c}^T\big]
$$

$$
\mathbf{H}^e=\frac{2hA_e}{12}\begin{bmatrix}2&1&1\\1&2&1\\1&1&2\end{bmatrix}
$$

### 3.3 Assembled systems

The steady-state system is:

$$
(\mathbf{K}+\mathbf{H})\,\mathbf{T} = \mathbf{F}(\mathbf{T})
$$

The transient (burst) system uses backward Euler:

$