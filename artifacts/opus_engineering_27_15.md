# Thermal Model of a 4-in-1 ESC at 80 A Total Burst Current

I don't have the context from earlier cycles, so this uses a **typical 30.5 × 30.5 mm 4-in-1 ESC** with the parameters below. Swap in your own values; every equation is written so you can do that.

## 0. Assumptions

| Parameter | Value |
|---|---|
| Board | 30 × 30 mm, 1.6 mm thick, 6 layers × 2 oz Cu (70 µm), ~70 % copper fill |
| Supply | 6S, 25.2 V maximum |
| Current | 80 A total, so 20 A per ESC channel |
| MOSFETs | 24 × 3.3 × 3.3 mm DFN (6 per channel), R_DS(on),25 = 1.5 mΩ, α = 0.0045 /K |
| Switching | f_sw = 48 kHz, t_r + t_f = 20 ns, dead time 100 ns, V_f = 0.8 V, Q_rr = 50 nC |
| Current shunt | Shared, 0.2 mΩ |
| Cooling | Prop-wash air at ~8 m/s, T_∞ = 25 °C |

---

## 1. Heat-Generation Physics (Source Terms)

### 1.1 MOSFET conduction loss
With trapezoidal (six-step) commutation, two FETs conduct at any instant (one high-side, one low-side, with synchronous rectification):

$$P_{cond} = 2\,I^2 R_{DS}(T_j),\qquad R_{DS}(T)=R_{25}\,[1+\alpha(T-25)]$$

At T ≈ 100 °C: R_DS ≈ 2.0 mΩ, so P_cond ≈ 2 · 400 · 2.0×10⁻³ ≈ **1.6 W per channel**.

### 1.2 Switching loss
Only one PWM-switched FET is active per channel:

$$P_{sw}=\tfrac12 V I (t_r+t_f) f_{sw}= 0.5\cdot25.2\cdot20\cdot20\text{ ns}\cdot48\text{ kHz}\approx \mathbf{0.24\ W}$$

### 1.3 Dead-time body-diode and reverse-recovery losses

$$P_{dt}=V_f I\,(2t_{dt}) f_{sw}\approx 0.15\text{ W},\qquad P_{rr}=Q_{rr}V f_{sw}\approx 0.06\text{ W}$$

### 1.4 Copper (Joule) loss

$$P_{Cu}=I^2 R_{trace},\qquad R_{trace}\approx\frac{\rho_{Cu}(T)\,\ell}{w\,t_{Cu}}$$

- Phase and bus planes, ≈0.5 mΩ per channel: **0.2 W per channel**.
- Shared shunt: 80² · 0.2 mΩ = **1.28 W**. This is a concentrated hot spot.

### 1.5 Logic
MCU, LDO and gate drivers: ≈ **0.3 W**.

### 1.6 Total

$$P_{tot}\approx 4(1.6+0.24+0.15+0.06+0.2) + 1.28 + 0.3 \approx \mathbf{10.6\text{–}10.9\ W}$$

The source depends on temperature through R_DS(T) and ρ_Cu(T). This makes the problem **nonlinear** and requires iteration (Section 3.4).

---

## 2. Effective (Orthotropic) Board Conductivity

The PCB is a laminate, so it conducts heat very differently in-plane and through its thickness.

**In-plane** (copper and FR-4 conduct in parallel):

$$k_\parallel=\frac{\sum k_i t_i}{t}=\frac{385(0.42)(0.7)+0.3(1.31)}{1.6}\approx \mathbf{71\ W/m\,K}$$

**Through-plane** (layers in series):

$$k_\perp=\frac{t}{\sum t_i/k_i}\approx \mathbf{0.37\ W/m\,K}$$

Under the FET pads, thermal-via arrays raise the through-plane value locally to about 5–10 W/m·K.

---

## 3. Finite-Element Formulation

### 3.1 Governing equation (3-D, steady state)

$$\nabla\cdot(\mathbf{k}\,\nabla T)+\dot q'''(T)=0,\qquad \mathbf{k}=\mathrm{diag}(k_\parallel,k_\parallel,k_\perp)$$

**Boundary conditions:**

- Convective and radiative faces:
  $$-\mathbf{k}\nabla T\cdot\mathbf{n}=h_c(T-T_\infty)+\varepsilon\sigma(T^4-T_\infty