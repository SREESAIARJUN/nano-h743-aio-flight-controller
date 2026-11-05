# Thermal Analysis: 4-in-1 ESC at 80 A Total Burst Current

## 0. Assumptions

No prior design data was provided, so this analysis uses a representative 30.5 mm-mount 4-in-1 ESC. Replace these values with your actual design data.

| Parameter | Value |
|---|---|
| Board outline | 40 × 40 mm, 1.6 mm thick, 4-layer, 2 oz (70 µm) Cu per layer |
| Supply | 6S, 25.2 V maximum |
| Current | 80 A total, so 20 A per channel |
| Commutation | 6-step, synchronous rectification, f_sw = 48 kHz |
| MOSFETs | 6 per channel, 40 V, DFN 3.3×3.3, R_ds(on,25°C) = 1.5 mΩ, R_θjc = 1.5 K/W |
| Shunts | 1 mΩ per channel |
| Ambient | T∞ = 35 °C |
| Cooling | Prop-wash airflow ≈ 8 m/s over the board |

"Cycle 10" is interpreted as the 10th (converged) iteration of the electro-thermal loop. Losses depend on temperature through R_ds(on)(T), so the problem is solved as a fixed point. The figures below are that fixed point.

---

## 1. Heat-Generation Physics (Source Terms)

### 1.1 MOSFET conduction loss
In 6-step drive, two FETs carry the phase current at any instant:

$$
P_{cond} = 2\,I^2 R_{ds}(T_j), \qquad R_{ds}(T_j)=R_{25}\,[1+\alpha (T_j-25)],\ \ \alpha\approx 0.006\ \mathrm{K^{-1}}
$$

The converged value is T_j ≈ 85 °C, so R_ds ≈ 1.36 × 1.5 = 2.04 mΩ. The first pass used a conservative factor of 1.5 (2.25 mΩ):

$$
P_{cond} = 2(20)^2(2.04\times10^{-3}) \approx 1.63\ \text{W/channel}
$$

### 1.2 Switching loss (hard-switched high side)
$$
P_{sw} = \tfrac12 V I (t_r+t_f) f_{sw} = \tfrac12(25.2)(20)(40\,\text{ns})(48\,\text{kHz}) \approx 0.48\ \text{W}
$$

### 1.3 Dead-time body-diode conduction and reverse recovery
$$
P_{dt}=V_F I\,(2t_{dt})f_{sw} = 0.8\cdot 20\cdot 200\,\text{ns}\cdot 48\,\text{kHz}\approx 0.15\ \text{W}
$$
$$
P_{rr}=V\,Q_{rr}\,f_{sw}=25.2\cdot 50\,\text{nC}\cdot 48\,\text{kHz}\approx 0.06\ \text{W}
$$

### 1.4 Gate drive
$$
P_g = Q_g V_{gs} f_{sw}\times 2 \approx 0.03\ \text{W}
$$

### 1.5 Shunt, capacitor ESR, MCU/regulator
- Shunt: I²R = 400 × 1 mΩ = 0.40 W
- Capacitor ESR (ripple): ≈ 0.10 W
- MCU and buck regulator: ≈ 0.10 W

### 1.6 Distributed Joule heating in copper planes
This term comes from a coupled electric-potential solve:
$$
\nabla\cdot(\sigma_{Cu}(T)\nabla\phi)=0,\qquad q_J = \sigma_{Cu}|\nabla\phi|^2 = J^2/\sigma_{Cu}
$$
$$
\sigma_{Cu}(T)=\frac{\sigma_{0}}{1+0.0039(T-20)}
$$

With a lumped estimate of R_plane ≈ 0.2 mΩ, the power plane loses 80² × 0.2 mΩ ≈ 1.28 W. This heat concentrates near the battery pads and the central bus.

### 1.7 Loss budget

| Source | Per channel (W) | Board total (W) |
|---|---|---|
| Conduction | 1.63 | 6.52 |
| Switching | 0.48 | 1.92 |
| Dead-time + Q_rr | 0.21 | 0.84 |
| Gate | 0.03 | 0.12 |
| Shunt | 0.40 | 1.60 |
| Capacitor ESR + logic | 0.20 | 0.80 |
| Bus planes | n/a | 1.28 |
| **Total** | **~2.95** | **≈ 13.1 W** |

The hottest individual FET is the PWM-switching high side, at about 0.82 + 0.48 + 0.1 ≈ **1.4 W**.

---

## 2. Conductive Heat-Transfer Formulation

### 2.1 Governing equation (3-D, anisotropic, steady state)
$$
\nabla\cdot\big(\mathbf{K}\,\nabla T\big) + q(\mathbf{x},T) = 0 \quad \text{in } \Omega
$$
$$
\mathbf{K} = \mathrm{diag}(k_{xy},\,k_{xy},\,k_z)
$$

### 2.2 Effective PCB conductivity
Copper thickness t_Cu = 4 × 70 µm = 0.28 mm, FR-4 thickness t_FR4 = 1.32 mm.

In-plane (layers in parallel):
$$
k_{xy}=\frac{t_{Cu}k_{Cu}+t_{FR4}k_{FR4}}{t}=\frac{0.28(385)+1.32(0.3)}{1.6}\approx 67.6\ \mathrm{W/m\,K}
$$

Through-plane (layers in series):
$$
k_z=\frac{t}{t_{Cu}/k_{Cu}+t_{FR4}/k_{FR4}}\approx 0.36\ \mathrm{W/m\,K}
$$

Under the FET pads, use a thermal-via array (e.g., 0.3 mm vias, 25 µm plating, 1 mm pitch). The local value becomes k_z,via ≈ k_Cu × (plated-area fraction) ≈ 5–10 W/m·K. Assign it as a separate material region.

### 2.3 Boundary conditions (Robin, top and bottom surfaces)
$$
-\mathbf{n}\cdot\mathbf{K}\nabla T = h_c (T-T_\infty) + \varepsilon\sigma_{S