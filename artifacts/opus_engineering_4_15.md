# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

## 0. Assumptions

I don't have the specifics of "Cycle 4," so the model uses a representative baseline. Every parameter is explicit and can be replaced with your design values.

| Parameter | Symbol | Value |
|---|---|---|
| Board | — | 30.5 × 30.5 mm (36 × 36 mm outline), 1.6 mm FR-4, 6 layers × 2 oz Cu |
| Bus voltage | V_dc | 22.2 V nominal (6S) |
| Total DC current | I_dc | 80 A (20 A per channel) |
| Topology | — | 4 × 3-phase bridges = 24 MOSFETs (40 V, R_ds,on(25 °C) = 1.2 mΩ) |
| Control | — | FOC/SVPWM, f_sw = 48 kHz, dead time t_d = 150 ns |
| Ambient | T_∞ | 25 °C |
| Cooling | — | Prop-wash forced convection, ≈ 8 m/s |

## 1. Heat-Source Physics (Loss Model)

### 1.1 Phase current from DC current

Per channel, the electrical power is P_e = V_dc · I_dc,ch = 22.2 × 20 = 444 W.

With SVPWM at M ≈ 1, the phase voltage is:

$$V_{ph,rms} = \frac{M V_{dc}}{\sqrt{3}\sqrt{2}} \approx 9.06\ \text{V}$$

The phase RMS current is then:

$$I_{ph,rms} = \frac{P_e}{3 V_{ph,rms}\cos\varphi} \approx \frac{444}{3(9.06)(0.95)} \approx 17.2\ \text{A}$$

The peak and rectified-average phase currents are:

$$\hat I = 24.3\ \text{A}, \qquad \bar{|i|} = \frac{2}{\pi}\hat I = 15.5\ \text{A}$$

### 1.2 MOSFET losses per half-bridge leg

**Conduction.** One switch in each leg always carries the phase current:

$$P_{cond} = I_{ph,rms}^2\, R_{ds}(T_j), \qquad R_{ds}(T_j) = R_{ds,25}\left[1+\alpha_R (T_j - 25)\right]$$

- α_R ≈ 0.0055 K⁻¹.
- At T_j ≈ 110 °C, R_ds ≈ 1.76 mΩ, rounded to 1.9 mΩ to include package and bond resistance.
- Result: P_cond ≈ 17.2² × 1.9 × 10⁻³ ≈ 0.56 W.

**Switching (hard-switched transitions).**

$$P_{sw} = \tfrac12 V_{dc}\,\bar{|i|}\,(t_r+t_f)\,f_{sw} = \tfrac12(22.2)(15.5)(20\,\text{ns})(48\,\text{kHz}) \approx 0.17\ \text{W}$$

**Output capacitance and reverse recovery.**

$$P_{oss+rr} = (E_{oss} + Q_{rr}V_{dc})\,f_{sw} \approx 0.05\ \text{W}$$

**Dead-time body-diode conduction.**

$$P_{dt} = V_F\,\bar{|i|}\,(2t_d)\,f_{sw} = 0.8(15.5)(300\,\text{ns})(48\,\text{kHz}) \approx 0.18\ \text{W}$$

**Gate drive.** P_g = Q_g V_gs f_sw · 2 ≈ 0.05 W. Most of this is dissipated in the gate driver, not in the FET.

**Total per leg:**

$$P_{leg} \approx 0.56 + 0.17 + 0.05 + 0.18 \approx 0.96\ \text{W}$$

This splits roughly 0.45 W high-side and 0.51 W low-side (the low side carries most of the diode loss).

### 1.3 Board-level loss budget

| Source | Expression | Loss |
|---|---|---|
| MOSFETs (12 legs) | 12 · P_leg | 11.5 W |
| Copper bus and phase planes | I_dc² R_Cu,eff (≈ 0.3 mΩ) | 1.9 W |
| Bus capacitor ESR | Σ I_rip,rms² ESR (I_rip ≈ 0.5 I_dc,ch) | ≈ 1.0 W |
| Regulators, MCU, drivers | — | ≈ 0.6 W |
| **Total** | | **Q_tot ≈ 15 W** |

That is about 0.84 % of the 1776 W throughput (≈ 99.2 % efficiency), consistent with real 4-in-1 boards.

## 2. Governing Conduction Equations

### 2.1 Full 3D anisotropic heat equation

$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{K}\nabla T) + q'''(\mathbf{x},T)$$

The conductivity tensor is:

$$\mathbf{K}=\mathrm{diag}(k_{xy},k_{xy},k_z)$$

### 2.2 Effective laminate conductivity

Use the rule of mixtures, with copper fraction t_Cu = 6 × 70 µm = 0.42 mm and t_FR4 = 1.18 mm.

**In-plane (layers in parallel):**

$$k_{xy} = \frac{\sum k_i t_i}{\sum t_i} = \frac{390(0.42) + 0.3(1.18)}{1.6} \approx 103\ \text{W/m·K}$$

**Through-plane (layers in series):**

$$k_z = \frac{\sum t_i}{\sum t_i/k_i} = \frac{1.6}{0.42/390 + 1.18/0.3} \approx 0.41\ \text{W/m·K}$$

Thermal via arrays under the FET pads raise k_z locally. For via fill fraction φ_v with copper plating:

$$k_{z,via} \approx \phi_v k_{Cu} + (1-\phi_v)k_z \;\Rightarrow\; \phi_v = 0.08 \to k_{z,via} \approx 31\ \text{W/m·K}$$

### 2.3 Electro-thermal source coupling

Joule heating in the copper planes comes from the conduction problem:

$$\nabla\cdot(\sigma(T)\nabla\phi)=0, \qquad q'''_{Cu} = \sigma(T)|\nabla\phi|^2, \qquad \sigma(T)=\frac{\sigma_0}{1+\alpha_{Cu}(T-T_0)}$$

- σ₀ = 5.8 × 10⁷ S/m.
- α_Cu = 0.00393 K⁻¹.

MOSFET losses enter as volumetric sources over the die/pad footprint Ω_k:

$$q'''_{k} = \frac{P_{FET,k}(T_k)}{V_{\Omega_k}}$$

### 2.4 Boundary conditions (Robin, top and bottom surfaces)

$$-\mathbf{n}\cdot\mathbf{K}\nabla T = h_{eff}(T - T_\infty), \qquad h_{eff} = h_c + h_r$$

**Forced convection** (flat-plate correlation, L = 0.03 m, v = 8 m/s):

$$Re = \frac{vL}{\nu} = \frac{8(0.03)}{1.6\times10^{-5}} = 1.5\times10^4$$

$$Nu = 0.664\,Re^{1/2}Pr^{1/3} = 72$$

$$h_c = \frac{Nu\,k_{air}}{L} \approx 63\ \text{W/m}^2\text{K}$$

Multiplying by about 1.4 for prop-wash turbulence and component roughness gives h_c ≈ 90 W/m²K.

**Linearized radiation:**

$$h_r = 4\varepsilon\sigma_{SB}\bar T^3 = 4(0.9)(5.67\times10^{-8})(350)^3 \approx 8.8\ \text{W/m}^2\text{K}$$

**Total:**

$$h_{eff} \approx 99\ \text{W/m}^2\text{K}$$

For an enclosed stack with no airflow, h_eff drops to about 15–25 W/m²K. This is the dominant sensitivity in the whole model.

Mounting standoffs and wires act as minor conductive sinks. Model them as lumped conductances G_s(T − T_∞), each about 0.01–0.03 W/K.

### 2.5 Reduced 2.5D shell model

Because k_xy/k_z ≈ 250 and the board is thin, integr