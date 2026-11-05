# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the specifications from earlier design cycles, so this uses explicit assumed parameters (§0). Replace them with your Cycle 7 values. The equations stay the same; only the numbers change.

---

## 0. Baseline Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Board size (30.5 mm mount) | L × W | 36 × 36 mm, 1.6 mm thick |
| Stack-up | — | 6-layer, 4 × 2 oz Cu (70 µm) power layers, ~60 % fill |
| Battery | V_bus | 6S, 22.2 V nominal |
| Current | I_tot, I_m | 80 A total = 20 A per motor |
| MOSFETs | — | 24 × PQFN 3.3×3.3 (6 per motor) |
| On-resistance | R_ds,25 | 1.5 mΩ at 25 °C, α_R ≈ 0.006 /K |
| Switching edges | t_r, t_f | 20 ns each |
| PWM frequency | f_sw | 48 kHz |
| Dead time | t_dt | 200 ns per edge |
| Body diode drop | V_f | 0.8 V |
| Gate charge / drive | Q_g, V_gs | 50 nC, 10 V |
| Ambient | T_∞ | 25 °C |
| Emissivity (solder mask) | ε | 0.9 |
| Forced convection (prop wash) | h_c | 50–100 W/m²K |
| Still-air convection | h_c | ~5–10 W/m²K |

---

## 1. Heat Generation Physics (Source Terms)

### 1.1 MOSFET conduction loss
In 6-step or FOC operation with synchronous rectification, about two FETs (one high-side, one low-side) carry phase current at any instant:

$$
P_{cond,m} = 2\,I_m^2\,R_{ds}(T_j), \qquad R_{ds}(T_j) = R_{ds,25}\left[1+\alpha_R (T_j - 25)\right]
$$

At T_j ≈ 90 °C, R_ds ≈ 2.1 mΩ:
$$P_{cond,m} = 2(20)^2(2.1\times10^{-3}) \approx 1.68\ \text{W}$$

### 1.2 Switching loss (one actively switched leg)
$$
P_{sw,m} = \tfrac12 V_{bus} I_m (t_r+t_f) f_{sw} + \tfrac12 C_{oss}V_{bus}^2 f_{sw}
$$
$$= 0.5(22.2)(20)(40\,\text{ns})(48\,\text{kHz}) + \approx0.01 \approx 0.44\ \text{W}$$

### 1.3 Dead-time (body-diode) loss
$$
P_{dt,m} = V_f\,I_m\,(2t_{dt})\,f_{sw} = 0.8(20)(400\,\text{ns})(48\,\text{kHz}) \approx 0.31\ \text{W}$$

### 1.4 Gate-drive loss (dissipated mostly in the driver and R_g)
$$
P_{g,m} = 2\,Q_g V_{gs} f_{sw} \approx 0.05\ \text{W}
$$

**Per-motor FET cluster:** P_m ≈ 2.5 W, or about 0.42 W per FET on average. Actual per-FET loss is unequal and depends on commutation state.

### 1.5 Board-level losses

| Source | Model | Value |
|---|---|---|
| Phase copper | $I_m^2 R_{trace}$, R ≈ 0.3 mΩ, ×4 motors | 0.48 W |
| Battery pads/planes | $I_{bat}^2 R_{bus}$, ~80 A peak, 0.2 mΩ | ~1.3 W (upper bound) |
| Bulk/MLCC ripple | $I_{C,rms}^2 \,ESR$, $I_{C,rms}\approx I\sqrt{D(1-D)}$ | ~0.3 W |
| MCUs + regulator (buck) | — | ~0.3 W |

**Total:**
$$
P_{tot} \approx 4(2.5) + 0.48 + 1.3 + 0.3 + 0.3 \approx 12.4\ \text{W}
$$

---

## 2. Governing Conductive Heat Transfer Equation

### 2.1 3-D form (anisotropic PCB)
$$
\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{k}\nabla T) + \dot q'''(\mathbf{x},T),
\qquad
\mathbf{k} = \mathrm{diag}(k_\parallel, k_\parallel, k_\perp)
$$

The effective layered conductivities follow parallel (in-plane) and series (through-plane) mixing:

$$
k_\parallel = \frac{\sum_i k_i t_i}{\sum t_i} = \frac{385(168\,\mu\text{m}) + 0.3(1432\,\mu\text{m})}{1.6\,\text{mm}} \approx 41\ \text{W/mK}
$$
$$
k_\perp = \frac{\sum t_i}{\sum t_i/k_i} \approx 0.34\ \text{W/mK}\quad(\text{no vias})
$$

Thermal vias under the FET pads raise local $k_\perp$ in proportion to the via copper area fraction $\phi_v$:
$$
k_{\perp,via} \approx \phi_v k_{Cu} + (1-\phi_v)k_\perp
$$
For example, φ_v = 5 % gives roughly 20 W/mK.

### 2.2 Thin-plate (2.5-D shell) reduction
The board is thin relative to its in-plane length (Biot number in thickness ≪ 1), so integrate through the thickness:

$$
(\rho c_p t)_{eff}\frac{\partial T}{\partial t} = \nabla_{xy}\cdot\left(k_\parallel t\,\nabla_{xy}T\right) - (h_{top}+h_{bot})(T-T_\infty) + q''(x,y,T)
$$

Here $q''$ is the areal heat flux from component footprints.

### 2.3 Boundary conditions

- **Robin (convection + linearized radiation)** on both faces:
$$
-k\frac{\partial T}{\partial n} = h_{eff}(T-T_\infty),\qquad h_{eff} = h_c + h_r,\quad h_r = 4\varepsilon\sigma T_m^3 \approx 8.8\ \text{W/m}^2\text{K}\ (T_m=350\,\text{K})
$$
- **Mounting standoffs (4×):** conductance $G_{s}$ to the frame. Treat as adiabatic if nylon standoffs with soft grommets are used.
- **Battery leads:** Dirichlet or conductive-fin sink, with $G_{lead} = \sqrt{hPkA_c}\,\tanh(mL)$.

### 2.4 Junction sub-model (lumped, coupled to FEM nodes)
$$
T_j = T_{board,node} + P_{FET}\left(R_{\theta jc} + R_{\theta,TIM/solder}\right)
$$
With R_θjc ≈ 1.5 K/W, ΔT_jc ≈ 0.6 K at steady state. At this power level the junction is board-limited, not package-limited.

---

## 3. Finite-Element Formulation

### 3.1 Weak (Galerkin) form
Find $T \in H^1(\Omega)$ such that for all test functions $w$:

$$
\int_\Omega (\rho c_p t)\,w\,\dot T\,d\Omega
+ \int_\Omega t\,\nabla w\cdot\mathbf{k}\nabla T\,d\Omega
+ \int_\Omega h_{\Sigma}\,w\,T\,d\Omega
= \int_\Omega w\,q''\,d\Omega + \int_\Omega h_\Sigma\, w\,T_\infty\,d\Omega
$$

where $h_\Sigma = h_{top}+h_{bot}$.

### 3.2 Discretization
Use $T \approx \mathbf{N}\mathbf{T}_e$. Assembly gives:

$$
\mathbf{C}\dot{\mathbf{T}} + (\mathbf{K}+\mathbf{H})\mathbf{T} = \mathbf{Q}(\mathbf{T}) + \mathbf{H}\,\mathbf{T}_\infty
$$

**Element matrices (linear 3-node triangle, area $A_e$):**

$$
\mathbf{K}_e = t\,A_e\,\mathbf{B}^T\mathbf{D}\mathbf{B},\quad
\mathbf{B} = \frac{1}{2A_e}\begin{bmatrix} y_{23} & y_{31} & y_{12}\\ x_{32} & x_{13} & x_{21}\end{bmatrix},\quad
\mathbf{D} = \begin{bmatrix}k_\parallel & 0\\0 & k_\parallel\end{bmatrix}
$$

$$
\mathbf{H}_e = \frac{h_\Sigma A_e}{12}\begin{bmatrix}2&1&1\\1&2&1\\1&1&2\end{bmatrix},\qquad
\mathbf{C}_e = \frac{(\rho c_p t)A_e}{12}\begin{bmatrix}2&1&1\\1&2&1\\1&1&2\end{