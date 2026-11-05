# Thermal Model of a 4‑in‑1 ESC at 80 A Total Burst Current

Your earlier cycles aren't visible to me, so this model states its assumptions explicitly. Substitute your own board parameters where they differ.

---

## 0. Baseline Assumptions

| Parameter | Value |
|---|---|
| Board | 38 × 38 mm, 1.6 mm, 4‑layer, 2 oz (70 µm) Cu on every layer |
| Supply | 6S, V_bus ≈ 25 V (fresh pack) |
| Current split | 80 A total → **20 A DC per ESC channel** |
| Power stage | 6 N‑MOSFETs per channel (24 total), R_ds,on(25 °C) = 1.5 mΩ |
| PWM | f_sw = 48 kHz, t_r + t_f ≈ 40 ns, dead time 100 ns |
| Commutation | 6‑step trapezoidal, synchronous rectification |
| Duty at burst | D ≈ 0.9 |
| Cooling | Prop‑wash airflow ≈ 10 m/s, ambient T∞ = 30 °C |

---

## 1. Heat‑Generation Physics (Volumetric Source q)

### 1.1 Motor-side current
In 6-step drive the phase (motor) current exceeds the DC-link current:

$$I_m \approx \frac{I_{dc}}{D} = \frac{20}{0.9} \approx 22.2\ \text{A}$$

### 1.2 Temperature-dependent on-resistance
This is the term that couples the loss model to the temperature field:

$$R_{ds}(T) = R_{25}\left[1+\alpha\,(T_j-25)\right], \qquad \alpha \approx 0.006\ \text{K}^{-1}$$

### 1.3 Loss components per ESC channel (evaluated at T_j ≈ 90 °C, R_ds ≈ 2.09 mΩ)

| Mechanism | Expression | Value |
|---|---|---|
| Conduction (2 FETs always in path) | $2 I_m^2 R_{ds}(T)$ | 2.06 W |
| Switching | $\tfrac12 V_{bus} I_m (t_r+t_f) f_{sw}$ | 0.53 W |
| Dead-time body diode | $2 V_f I_m t_{dt} f_{sw}$ | 0.17 W |
| Reverse recovery | $Q_{rr} V_{bus} f_{sw}$ | 0.06 W |
| Gate drive | $Q_g V_{gs} f_{sw}\cdot n_{sw}$ | 0.05 W |
| **Per channel** | | **≈ 2.87 W** |

### 1.4 Board-level losses

| Source | Expression | Value |
|---|---|---|
| 4 channels | 4 × 2.87 W | 11.5 W |
| Power planes / pads (R ≈ 0.3 mΩ) | $I_{tot}^2 R_{plane}$ | 1.9 W |
| Current-sense shunt (0.25 mΩ) | $I_{tot}^2 R_{sh}$ | 1.6 W |
| **Total Q** | | **≈ 15 W** |

Within each channel the high-side switching FET is the hottest device, at about 1.1 W.

---

## 2. Governing Conduction Equation

### 2.1 Transient form
$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot\left(\mathbf{k}\,\nabla T\right) + q(\mathbf{x},T)$$

### 2.2 Steady state
$$\nabla\cdot\left(\mathbf{k}\,\nabla T\right) + q(\mathbf{x},T) = 0$$

### 2.3 Orthotropic PCB conductivity
The board conducts very differently in-plane and through-plane, so **k** is treated as a diagonal tensor:

$$\mathbf{k}=\mathrm{diag}(k_{xy},k_{xy},k_z)$$

In-plane, the layers act as parallel paths:

$$k_{xy}=\frac{\sum k_i t_i}{\sum t_i}=\frac{385(0.28)+0.3(1.32)}{1.6}\approx 67.6\ \text{W/m·K}$$

Through-plane, the layers act in series:

$$k_z=\frac{\sum t_i}{\sum t_i/k_i}\approx 0.36\ \text{W/m·K}$$

The anisotropy ratio is about 190:1. This is why **thermal-via arrays under the FETs are essential**. Model them as local regions with

$$k_{z,via} \approx k_{Cu}\,\phi_{via} + k_{FR4}(1-\phi_{via}),$$

where φ_via is the via copper area fraction (e.g. 0.05–0.1, giving k_z ≈ 20–40 W/m·K locally).

### 2.4 Boundary conditions
On the board faces (convection plus linearized radiation):

$$-\mathbf{k}\nabla T\cdot\mathbf{n} = h_{c}(T-T_\infty) + \varepsilon\sigma(T^4-T_\infty^4) \;\approx\; (h_c+h_r)(T-T_\infty)$$

On the edges, use adiabatic conditions or a small h. The mounting-screw contact can be treated as a Robin condition with a contact conductance.

---

## 3. Convection Coefficients

**Forced convection (prop wash)**, using a flat-plate laminar correlation:

$$Re=\frac{UL}{\nu}=\frac{10(0.038)}{1.6\times10^{-5}}\approx 2.4\times10^4 \;(\text{laminar})$$

$$Nu = 0.664\,Re^{1/2}Pr^{1/3}\approx 91 \;\Rightarrow\; h_c=\frac{Nu\,k_{air}}{L}\approx 62\ \text{W/m}^2\text{K}$$

**Radiation:**

$$h_r = 4\varepsilon\sigma T_m^3 \approx 8\ \text{W/m}^2\text{K}$$

**Effective coefficient:** h ≈ 70 W/m²K on both faces, with total area A = 2.89×10⁻³ m².

If the bottom face is shadowed by a stacked flight controller, reduce h_bot accordingly.

---

## 4. Finite-Element Formulation

### 4.1 Reduced 2.5-D plate model
The thin board can be collapsed to a 2-D plate with through-thickness losses folded in:

$$t\,k_{xy}\nabla^2 T - (h_{top}+h_{bot})(T-T_\infty) + q''(x,y,T)=0$$

Here q'' is the areal heat flux under each FET footprint.

### 4.2 Weak (Galerkin) form
Find T such that, for all test functions w:

$$\int_\Omega t\,k_{xy}\,\nabla w\cdot\nabla T\,d\Omega + \int_\Omega h_{tot}\,w\,T\,d\Omega = \int_\Omega w\,q''\,d\Omega + \int_\Omega h_{tot}\,w\,T_\infty\,d\Omega$$

### 4.3 Element matrices
With shape functions N and gradient matrix B = ∇N:

$$\mathbf{K}_e = \int_{\Omega_e} t\,k_{xy}\,\mathbf{B}^T\mathbf{B}\,d\Omega + \int_{\Omega_e} h_{tot}\,\mathbf{N}^T\mathbf{N}\,d\Omega$$

$$\mathbf{f}_e = \int_{\Omega_e}\mathbf{N}^T q''\,d\Omega + \int_{\Omega_e}\mathbf{N}^T h_{tot}T_\infty\,d\Omega$$

### 4.4 Assembly and nonlinear solution
Assemble the global system **K T = f(T)**. Because the losses depend on temperature through R_ds(T), solve iteratively. A Picard iteration works:

$$\mathbf{K}\,\mathbf{T}^{(n+1)} = \mathbf{f}\!\left(R_{ds}(\mathbf{T}^{(n)})\right)$$

Convergence is fast, typically 3–5 iterations, as long as $\partial P/\partial T < hA$. If that inequality is violated, the iteration diverges; this is the numerical signature of **thermal runaway**.

### 4.5 Full 3-D option
For more detail, use layered hex elements:
- Mesh each Cu and FR4 layer explicitly.
- Model the FET package as die → lead frame → solder (R_θJC ≈ 1.2 K/W).
- Use a mesh of ≤ 0.25 mm under the packages and coarsen to about 1 mm elsewhere.

---

## 5. Steady-State Solution

### 5.1 Is the board nearly isothermal? (fin parameter)
The spreading length tells us whether a lumped model is valid:

$$m=\sqrt{\frac{h_{tot}}{k_{xy}t}}=\sqrt{\frac{140}{67.6\times0.0016}}\approx 36\ \text{m}^{-1}, \qquad \frac{1}{m}\approx 28\ \text{mm}$$

The spreading length is comparable to the board size, so the board is near-isothermal and a lumped estimate is valid for the mean temperature.

### 5.2 Mean board temperature
$$\bar T = T_\infty + \frac{Q}{hA} = 30 + \frac{15}{0.202} \approx 104\ ^\circ\text{C}$$

### 5.3 Local hot spot from plate spreading
Treating each channel cluster as a source of radius r_i ≈ 5 mm spreading to r_o ≈ 10 mm:

$$\Delta T_{spread}\approx\frac{P_{ch}}{2\pi k_{xy}t}\ln\frac{r_o}{r_i}=\frac{2.87}{0.68}(0.693)\approx 3\ \text{K}$$

### 5.4 Junction of the hottest FET
$$T_j \approx \bar T + \Delta T_{spread} + P_{FET}(R_{\theta JC}