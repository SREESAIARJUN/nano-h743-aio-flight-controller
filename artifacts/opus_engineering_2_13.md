# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the "Cycle 1" context, so this formulation is self-contained, and every assumption is listed in §0. Replace the parameters with your actual board data (MOSFET datasheet, stack-up, airflow) and the equations still apply.

---

## 0. Assumptions

| Parameter | Symbol | Value |
|---|---|---|
| Board | — | 30 × 30 mm, 1.6 mm thick, 6-layer, 2 oz (70 µm) copper per layer |
| Supply | V_bus | 6S Li-ion, ≈25 V under load |
| Current per channel | I_ph | 80 A / 4 = **20 A** |
| MOSFETs | — | 6 per channel (24 total), 5×6 mm package |
| MOSFET on-resistance | R_ds,on | 1.5 mΩ at 25 °C, tempco α_R ≈ 0.006 /K |
| Switching | f_sw, t_r + t_f | 48 kHz, 20 ns |
| Dead time | t_dt | 200 ns |
| Body-diode forward voltage | V_f | 0.8 V |
| Recovered charge | Q_rr | 50 nC |
| Gate | Q_g, V_gs | 40 nC, 10 V |
| Duty at burst | D | ≈0.9 (battery current ≈72 A) |
| Cooling | — | Ambient 35 °C, prop-wash forced convection h_c ≈ 80 W/m²K, emissivity ε ≈ 0.9 |

---

## 1. Heat-Generation Physics (Source Terms)

### 1.1 MOSFET conduction loss

With synchronous rectification, two FETs (one high-side, one low-side) carry the phase current at any instant:

$$P_{cond,ch} = 2\,I_{ph}^2\,R_{ds,on}(T_j),\qquad R_{ds,on}(T_j)=R_{25}\,[1+\alpha_R(T_j-25)]$$

At 25 °C: P = 2 · 20² · 1.5 mΩ = **1.20 W per channel**. This rises to ≈1.8 W at T_j ≈ 115 °C.

### 1.2 Switching loss

$$P_{sw}=\tfrac12 V_{bus} I_{ph}(t_r+t_f) f_{sw} = 0.5\cdot25\cdot20\cdot20\,\text{ns}\cdot48\,\text{kHz}=0.24\ \text{W}$$

### 1.3 Reverse recovery and gate drive

$$P_{rr}=V_{bus}Q_{rr}f_{sw}=0.06\ \text{W},\qquad P_g \approx 2\,Q_gV_{gs}f_{sw}\approx0.04\ \text{W}$$

Only part of the gate power is dissipated in the FET; the rest is lost in the driver.

### 1.4 Dead-time body-diode conduction

$$P_{dt}=2V_fI_{ph}t_{dt}f_{sw}=2\cdot0.8\cdot20\cdot200\,\text{ns}\cdot48\,\text{kHz}=0.31\ \text{W}$$

### 1.5 Copper (trace and plane) Joule heating

These losses require a coupled electrical solution. The current distribution satisfies:

$$\nabla\cdot(\sigma(T)\nabla\phi)=0,\qquad q_{Cu}=\sigma|\nabla\phi|^2,\qquad \sigma(T)=\frac{\sigma_0}{1+\alpha_{Cu}(T-T_0)},\ \alpha_{Cu}=0.00393\,/\text{K}$$

As a lumped estimate, take the effective bus resistance as about 0.3 mΩ:

$$P_{Cu}\approx 72^2 \cdot 0.3\,\text{m}\Omega \approx 1.6\ \text{W}$$

This term is often underestimated. With a 1 oz, 2-layer board it can exceed 5 W.

### 1.6 Other sources

- Ceramic decoupling-capacitor ESR (ripple current I_rms ≈ I_ph·√(D(1−D))): ≈0.3 W total
- MCUs and regulators: ≈0.3 W

### 1.7 Loss budget

| Source | Per channel | Board total |
|---|---|---|
| Conduction (at 25 °C) | 1.20 W | 4.8 W |
| Switching + Q_rr + gate | 0.34 W | 1.4 W |
| Dead time | 0.31 W | 1.2 W |
| Copper, capacitors, logic | — | 2.2 W |
| **Total at 25 °C** | | **≈9.6 W** |
| **Total at operating temperature** | | **≈12.2 W** (see §4) |

---

## 2. Conductive Heat-Transfer Formulation

### 2.1 Governing equation (3-D, anisotropic)

$$\rho c_p\frac{\partial T}{\partial t}=\nabla\cdot(\mathbf{k}\nabla T)+q(\mathbf{x},T),\qquad \mathbf{k}=\text{diag}(k_{xy},k_{xy},k_z)$$

At steady state:

$$\nabla\cdot(\mathbf{k}\nabla T)+q=0$$

### 2.2 Effective PCB conductivity (laminate homogenization)

The stack-up has 0.42 mm of copper (k = 385 W/mK) and 1.18 mm of FR-4 (k = 0.3 W/mK).

In-plane, the layers act in parallel:

$$k_{xy}=\frac{\sum k_it_i}{\sum t_i}=\frac{385(0.42)+0.3(1.18)}{1.6}\approx 101\ \text{W/mK}$$

Through-plane, the layers act in series:

$$k_z=\frac{\sum t_i}{\sum t_i/k_i}=\frac{1.6}{0.42/385+1.18/0.3}\approx 0.41\ \text{W/mK}$$

The 250:1 anisotropy is why **thermal vias** under the FET drain pads matter. A via array with copper fill fraction φ_v raises the local through-plane conductivity:

$$k_z^{via}\approx \varphi_v k_{Cu}+(1-\varphi_v)k_z$$

### 2.3 Boundary con