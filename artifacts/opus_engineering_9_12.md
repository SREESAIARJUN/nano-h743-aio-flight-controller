# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the earlier cycles of this design, so the parameters below are assumed values. Replace them with your schematic and BOM values, and the equations will carry through unchanged.

## 0. Assumed Design Basis

| Parameter | Value |
|---|---|
| Board | 36 × 36 mm, 1.6 mm thick, 4-layer, 2 oz (70 µm) copper per layer |
| Supply | 6S, V_bus = 25.2 V maximum |
| Current | 80 A total, 20 A per motor channel |
| Power stage | 24 MOSFETs (6 per channel), R_ds,on(25 °C) = 1.5 mΩ, Q_g = 40 nC, Q_oss = 30 nC |
| Switching | f_sw = 48 kHz, t_r + t_f = 30 ns, dead time 100 ns, body-diode V_f = 0.8 V |
| Environment | T_∞ = 35 °C, prop-wash convection h = 60 W/m²K |
| Commutation | Six-step trapezoidal. Two phases conduct at a time, so one high-side and one low-side FET are always in the current path. |

---

## 1. Heat-Generation Physics

### 1.1 Temperature-dependent on-resistance
$$R_{ds}(T_j)=R_{25}\,[1+\alpha(T_j-25)],\qquad \alpha\approx0.005\ \mathrm{K^{-1}}$$

At T_j = 100 °C, this gives R_ds ≈ 2.06 mΩ.

This dependence creates electro-thermal feedback: higher temperature raises resistance, which raises loss, which raises temperature. The FEM solution in Section 3.4 handles this with an iterative loop.

### 1.2 Loss per channel (I = 20 A)

| Mechanism | Expression | Value |
|---|---|---|
| Conduction | $P_c = I^2\cdot 2R_{ds}(T_j)$ | 1.65 W |
| Switching (one leg PWMs at a time) | $P_{sw}=\tfrac12 V I (t_r+t_f) f_{sw}$ | 0.36 W |
| Output capacitance | $P_{oss}=Q_{oss}V f_{sw}$ | 0.04 W |
| Dead-time diode conduction | $P_{dt}=V_f I\,(2t_{dt}) f_{sw}$ | 0.15 W |
| Gate drive | $P_g = 2Q_g V_{gs} f_{sw}$ | 0.04 W |
| Copper traces (≈1 mΩ effective) | $I^2 R_{Cu}$ | 0.40 W |
| Current shunt (0.5 mΩ) | $I^2R_{sh}$ | 0.20 W |
| **Channel total** | | **≈ 2.84 W** |

**Board total:** 4 × 2.84 W + 0.5 W (MCU and regulator) ≈ **12 W**.

As a volumetric source in the FET die/pad region:
$$\dot q'''=\frac{P_{FET}}{V_{die}}$$

Each conducting FET dissipates about 0.8–1.0 W while it is in the current path.

---

## 2. Governing Conduction Equations

### 2.1 Anisotropic heat equation in the PCB
$$\rho c_p\frac{\partial T}{\partial t}=\nabla\cdot(\mathbf{k}\nabla T)+\dot q''',\qquad \mathbf{k}=\mathrm{diag}(k_x,k_x,k_z)$$

### 2.2 Effective laminate conductivities
Copper thickness t_Cu = 0.28 mm, FR-4 thickness t_FR4 = 1.32 mm.

In-plane (layers act as parallel conductors):
$$k_x=\frac{\sum k_i t_i}{t}=\frac{385(0.28)+0.3(1.32)}{1.6}\approx 68\ \mathrm{W/mK}$$

Through-plane (layers act in series):
$$k_z=\frac{t}{\sum t_i/k_i}\approx0.36\ \mathrm{W/mK}$$

Thermal vias under the FET pads change the through-plane value, where φ is the copper area fraction of the via field:
$$k_z^{via}=\varphi k_{Cu}+(1-\varphi)k_z$$

Each 0.3 mm via with 25 µm plating contributes 0.0216 mm² of copper. A via array with φ ≈ 5 % raises k_z to about 19 W/mK, a 50× improvement.

### 2.3 Boundary conditions
Convection plus radiation on the top and bottom surfaces:
$$-\mathbf{k}\nabla T\cdot\mathbf{n}=h(T-T_\infty)+\varepsilon\sigma(T^4-T_\infty^4)$$

Radiation can be linearized as $h_{rad}=4\varepsilon\sigma\bar T^3\approx 7\ \mathrm{W/m^2K}$ (solder mask, ε ≈ 0.9). This gives h_eff ≈ 67 W/m²K.

At the mounting screws and solder pads, conduction into the frame and wires applies:
$$-\mathbf{k}\nabla T\cdot\mathbf n = T/R_{contact}$$

---

## 3. Finite-Element Formulation

### 3.1 Weak form
Find T such that for all test functions w:
$$\int_\Omega \rho c_p w\,\dot T\,d\Omega+\int_\Omega\nabla w\cdot\mathbf{k}\nabla T\,d\Omega+\int_{\Gamma_h}h_{eff}\,wT\,d\Gamma=\int_\Omega w\,\dot q'''d\Omega+\int_{\Gamma_h}h_{eff}\,wT_\infty d\Gamma$$

### 3.2 Galerkin discretization
Approximate the temperature field with shape functions, $T\approx\sum N_jT_j$. The element matrices are:

$$K^e_{ij}=\int_{\Omega_e}\nabla N_i\cdot\mathbf{k}\nabla N_j\,d\Omega+\int_{\Gamma_e}h_{eff}N_iN_j\,d\Gamma$$
$$C^e_{ij}=\int_{\Omega_e}\rho c_pN_iN_j\,d\Omega,\qquad F^e_i=\int_{\Omega_e}N_i\dot q'''d\Omega+\int_{\Gamma_e}h_{eff}T_\infty N_i\,d\Gamma$$

Assembling gives:
$$\mathbf C\dot{\mathbf T}+\mathbf K(\mathbf T)\mathbf T=\mathbf F(\mathbf T)$$

### 3.3 Recommended layered-shell discretization (2.5-D)
Model each copper layer ℓ as a 2-D sheet with conductance k_Cu t_ℓ. Couple adjacent layers through the dielectric with a per-area conductance:
$$G_z = k_z^{(local)}/t_{diel}$$

Use the via-enhanced k_z under the FET pads. Refine the mesh to 0.25 mm under the FETs and use about 1 mm elsewhere.

### 3.4 Nonlinear steady-state solution (Picard iteration)
1. Initialize **T⁽⁰⁾** = T_∞.
2. Compute R_ds(T⁽ⁿ⁾), then the losses **F**(T⁽ⁿ⁾), and h_rad(T⁽ⁿ⁾).
3. Solve **K T⁽ⁿ⁺¹⁾ = F**.
4. Repeat until ‖ΔT‖_∞ < 0.01 K. This typically takes 4–6 iterations.

For transients, use backward Euler:
$$(\mathbf C/\Delta t+\mathbf K)\mathbf T^{n+1}=\mathbf F+\mathbf C\mathbf T^n/\Delta t$$

---

## 4. Analytical Cross-Check of the Steady State

### 4.1 Lumped board temperature
Exposed area is about 2.6 × 10⁻³ m². Applying a 1.3× multiplier for component surfaces:
$$hA \approx 67(2.6\times10^{-3})(1.3)\approx0.23\ \mathrm{W/K}$$
$$\bar T_{board}=T_\infty+\frac{P}{hA}=35+\frac{12}{0.23}\approx 87\ ^\circ\mathrm C$$

### 4.2 Heat-spreading length
Treating the board as a fin plate cooled on both sides:
$$L_c=\sqrt{\frac{k_x t}{2h_{eff}}}=\sqrt{\frac{68(1.6\times10^{-3})}{134}}\approx 28\ \mathrm{mm}$$

L_c is comparable to the board size. The board therefore spreads heat reasonably well, but each FET cluster still forms a distinct hotspot.

### 4.3 Local rise around one FET cluster (≈2.4 W)
Modelling the cluster as a disk source in an infinite fin plate gives:
$$\Delta T(r)=\frac{P}{2\pi k_xt}K_0\!\left(\frac{r}{L_c}\right)$$

Evaluated at the cluster edge (r ≈ 2 mm, so K_0(0.07) ≈ 2.8):
$$\Delta T\approx\frac{2.4}{2\pi(0.109)}(2.8)\approx 9.8\ \mathrm K$$

### 4.4 Junction temperature
The junction-to-board stack, with vias, has R_jc + R_via ≈ 3 K/W. At 0.9 W per FET this adds ≈ 2.7 K.

$$T_j\approx 87+9.8+2.7+\text{(neighbor superposition }\approx 8)\approx \mathbf{108\ ^\circ C}$$

---

## 5. Steady-State Temperature Distribution (h = 60 W/m²K, T∞ = 35 °C)

| Location | T (°C) |
|