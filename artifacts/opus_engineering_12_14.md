# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have your earlier cycles' data, so this uses a stated baseline design. Every number below is a parameter you can replace. The structure (loss model → governing equations → FEM → electrothermal solution) stays the same.

---

## 0. Baseline Assumptions

| Parameter | Value |
|---|---|
| Board | 30.5 × 30.5 mm, 1.6 mm, 6-layer (outer 2 oz = 70 µm, inner 4 × 1 oz = 35 µm) |
| Supply | 6S, V_bus = 25.2 V (worst case, full charge) |
| Current | 80 A total = 20 A RMS phase current per channel × 4 |
| Power stage | 6 MOSFETs/channel (24 total), 3×3 mm DFN, R_DS(on),25 = 1.5 mΩ |
| Switching | f_sw = 48 kHz, t_r + t_f = 40 ns, dead time 100 ns, V_SD = 0.8 V |
| Environment | T∞ = 40 °C (inside the stack), forced convection from prop wash |

---

## 1. Heat-Generation Physics (Volumetric Sources q‴)

### 1.1 MOSFET conduction loss (temperature-dependent)

With 6-step or FOC and synchronous rectification, two legs conduct at any instant:

$$P_{cond,ch} = 2\,I^2 R_{DS}(T),\qquad R_{DS}(T)=R_{25}\,[1+\alpha_R (T-25)],\ \alpha_R\approx 0.005\ \text{K}^{-1}$$

At 25 °C: 2 · 20² · 1.5 mΩ = **1.20 W per channel** (4.8 W total). This term rises with temperature.

### 1.2 Switching loss

One leg hard-switches per PWM period:

$$P_{sw} = \tfrac12 V_{bus} I (t_r+t_f) f_{sw} = \tfrac12(25.2)(20)(40\text{ ns})(48\text{ kHz}) = 0.48\text{ W/ch}$$

### 1.3 Dead-time body-diode loss

$$P_{diode} = V_{SD}\, I \cdot 2t_{dead} f_{sw} = 0.8\cdot20\cdot 200\text{ ns}\cdot 48\text{ kHz} = 0.15\text{ W/ch}$$

### 1.4 Gate-drive and C_oss loss

$$P_{gate} = Q_g V_{GS} f_{sw}\times n_{switching},\quad P_{oss}=\tfrac12 C_{oss}V^2 f_{sw}$$

Together these are about **0.3 W** total.

### 1.5 Copper Joule heating (distributed)

Sheet resistance of 2 oz copper:

$$R_\square = \rho_{Cu}/t = 1.72\times10^{-8}/70\,\mu\text{m} = 0.25\ \text{m}\Omega/\square$$

- The bus and phase pours use about 3 paralleled layers over about 2 squares effective, giving roughly 0.2 mΩ.
- At 80 A this gives P_Cu ≈ **1.3 W**, scaling with [1 + 0.0039(T − 25)].
- The local volumetric source in a trace is q‴ = |J|²ρ(T).

### 1.6 Other sources

- Bulk and MLCC ESR from ripple current I_rip ≈ I√(D(1−D)): about 0.5 W.
- MCUs and LDO: about 0.3 W.

### 1.7 Total loss as a function of temperature (linearized)

$$P(T) \approx 9.76 + 0.029\,(T-25)\ \text{W}$$

---

## 2. Governing Conduction Equation

Transient anisotropic heat equation in the board volume Ω:

$$\rho c_p \frac{\partial T}{\partial t} = \nabla\cdot(\mathbf{k}\nabla T) + q'''(\mathbf{x},T)$$

At steady state:

$$\nabla\cdot(\mathbf{k}\nabla T) + q''' = 0$$

### 2.1 Orthotropic PCB conductivity (layer homogenization)

Total copper thickness is t_Cu = 280 µm and total FR4 thickness is t_FR4 = 1320 µm.

- **In-plane (parallel paths):**
$$k_{xy} = \frac{\sum k_i t_i}{\sum t_i} = \frac{385(0.28)+0.3(1.32)}{1.6} \approx 67.6\ \text{W/m·K}$$
- **Through-plane (series paths):**
$$k_z = \frac{\sum t_i}{\sum t_i/k_i} \approx 0.36\ \text{W/m·K}$$

The in-plane to through-plane ratio is about 190:1. In-plane spreading dominates, and through-plane transport depends on vias.

### 2.2 Thermal vias under the FET pads

For a 0.3 mm drill with 25 µm plating:

$$R_{via} = \frac{L}{k_{Cu}A_{ring}} = \