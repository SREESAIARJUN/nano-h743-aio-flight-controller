# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundations

## Design Document, Cycle 31: Motor Control Core

---

## 0. System Context and Assumptions

| Parameter | Symbol | Design Value (per axis) | Notes |
|---|---|---|---|
| Motor type | n/a | Hollow-shaft gimbal BLDC (SPMSM), e.g. GBM2804-class | Surface magnets, so $L_d \approx L_q$ |
| Pole pairs | $p$ | 7 (12N14P) | |
| Phase resistance | $R_s$ | 5–10 Ω | High-turn gimbal winding |
| Phase inductance | $L_s$ | 1.5–3 mH | |
| PM flux linkage (peak, per phase) | $\lambda_{pm}$ | ~3–6 mWb | **Must be measured per motor batch** |
| DC bus | $V_{dc}$ | 12 V (3S Li-ion, 9.9–12.6 V) | |
| PWM | $f_{sw}$ | 20 kHz, center-aligned | ESP32-S3 MCPWM, inaudible |
| Current loop rate | $f_c$ | 10–20 kHz | ISR on Core 1 |
| Position sensor | n/a | 14-bit magnetic encoder (AS5048A/MT6701) over SPI | |
| Compute | n/a | Xtensa LX7 @ 240 MHz, single-precision FPU | |

**Physical note before the math.** Gimbal motors are direct-drive, low-speed, high-resistance machines. Field weakening (Section 5) is rarely needed in steady stabilization. It earns its place only for fast re-centering slews and "follow-mode" snaps at low battery. I derive it rigorously anyway and quantify when it actually helps (Section 5.6), because with high $R_s$ the textbook lossless result is misleading.

---

## 1. Space-Vector Formalism

### 1.1 Definition

Let $x \in \{v, i, \lambda\}$ denote any three-phase quantity. Define the rotation operator $a = e^{j2\pi/3}$. The **space vector** is:

$$
\vec{x} = k\left(x_a + a\,x_b + a^2 x_c\right), \qquad \vec{x} = x_\alpha + j x_\beta
$$

The scale factor $k$ is a design choice:
- $k = 2/3$ gives the **amplitude-invariant** form.
- $k = \sqrt{2/3}$ gives the **power-invariant** form.

### 1.2 Proof of amplitude invariance for $k = 2/3$

Take a balanced set:

$$
x_a = X\cos\theta,\quad x_b = X\cos(\theta - 2\pi/3),\quad x_c = X\cos(\theta + 2\pi/3)
$$

Write $\cos\phi = \tfrac{1}{2}(e^{j\phi} + e^{-j\phi})$ and substitute:

$$
\sum_{n=0}^{2} a^n \cos\!\left(\theta - \tfrac{2\pi n}{3}\right)
= \tfrac12 e^{j\theta}\sum_{n=0}^{2} a^n a^{-n} + \tfrac12 e^{-j\theta}\sum_{n=0}^{2} a^{2n}
= \tfrac32 e^{j\theta} + \tfrac12 e^{-j\theta}\underbrace{(1 + a^2 + a^4)}_{=\,0}
$$

Therefore:

$$
\vec{x} = \tfrac{2}{3}\cdot\tfrac{3}{2}X e^{j\theta} = X e^{j\theta} \qquad \blacksquare
$$

The vector magnitude equals the phase peak. This is what we want: current limits in firmware map directly to the motor datasheet's peak phase current.

---

## 2. Clarke Transform

### 2.1 Matrix form

Expand $a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$, then separate real and imaginary parts:

$$
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_0 \end{bmatrix}
= \frac{2}{3}
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12 \\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\
\tfrac12 & \tfrac12 & \tfrac12
\end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c \end{bmatrix}
$$

The zero-sequence component $x_0$ is included so the matrix is invertible.

### 2.2 Reduced two-sensor form (used in firmware)

For a wye-connected motor with no neutral return, KCL gives $i_a + i_b + i_c = 0$, so $i_0 \equiv 0$. Substitute $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a - \tfrac12 i_b + \tfrac12 i_a + \tfrac12 i_b\right) = i_a
$$

$$
i_\beta = \tfrac23\cdot\tfrac{\sqrt3}{2}\left(i_b + i_a + i_b\right) = \frac{i_a + 2 i_b}{\sqrt3}
$$

$$
\boxed{\,i_\alpha = i_a, \qquad i_\beta = \frac{1}{\sqrt3}\left(i_a + 2i_b\right)\,}
$$

This needs only two shunts and costs 1 multiply and 2 adds.

**Shunt-quality improvement.** With three low-side shunts, sample all three and use the two with the longest low-side on-time (the phases *not* at max duty). Reconstruct the third from KCL. This avoids blind sampling windows near 100 % duty.

### 2.3 Inverse Clarke

$$
x_a = x_\alpha,\qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

### 2.4 Power in amplitude-invariant coordinates

The transform is not orthogonal, since $K K^T \ne I$, so power picks up a factor of 3/2:

$$
p = v_a i_a + v_b i_b + v_c i_c = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right) + 3 v_0 i_0
$$

This 3/2 factor propagates into the torque equation (Section 3.3). Forgetting it is the single most common FOC scaling bug.

---

## 3. Park Transform and the Machine Model

### 3.1 Derivation

Let $\theta_e = p\,\theta_m - \theta_{\text{offset}}$ be the electrical rotor angle, with the d-axis aligned to the magnet north pole. Projecting onto a frame rotating with the rotor is multiplication by $e^{-j\theta_e}$:

$$
\vec{x}_{dq} = \vec{x}_{\alpha\beta}\,e^{-j\theta_e}
$$

$$
(x_d + j x_q) = (x_\alpha + j x_\beta)(\cos\theta_e - j\sin\theta_e)
$$

$$
\boxed{
\begin{aligned}
x_d &= \phantom{-}x_\alpha\cos\theta_e + x_\beta\sin\theta_e \\
x_q &= -x_\alpha\sin\theta_e + x_\beta\cos\theta_e
\end{aligned}}
$$

The **inverse Park** uses multiplication by $e^{+j\theta_e}$:

$$
v_\alpha = v_d\cos\theta_e - v_q\sin\theta_e,\qquad
v_\beta = v_d\sin\theta_e + v_q\cos\theta_e
$$

Park is a pure rotation, so it is orthogonal and magnitude-preserving: $|\vec{x}_{dq}| = |\vec{x}_{\alpha\beta}|$.

### 3.2 Voltage equations in the dq frame

Start from the stator equation in the stationary frame:

$$
\vec{v}_s = R_s\vec{i}_s + \frac{d\vec{\lambda}_s}{dt}
$$

Substitute $\vec{\lambda}_s = \vec{\lambda}_{dq}e^{j\theta_e}$. The product rule produces the **speed-voltage** term:

$$
\frac{d\vec\lambda_s}{dt} = \left(\frac{d\vec\lambda_{dq}}{dt} + j\omega_e\vec\lambda_{dq}\right)e^{j\theta_e}
$$

With $\lambda_d = L_d i_d + \lambda_{pm}$ and $\lambda_q = L_q i_q$:

$$
\boxed{
\begin{aligned}
v_d &= R_s i_d + L_d\frac{di_d}{dt} - \omega_e L_q i_q \\
v_q &= R_s i_q + L_q\frac{di_q}{dt} + \omega_e L_d i_d + \omega_e\lambda_{pm}
\end{aligned}}
$$

### 3.3 Torque

Apply the power balance with the 3/2 factor and extract the term multiplying $\omega_m = \omega_e/p$:

$$
T_e = \tfrac32\,p\left[\lambda_{pm}\,i_q + (L_d - L_q)\,i_d\,i_q\right]
\;\xrightarrow{\;L_d = L_q\;}\;
T_e = \underbrace{\tfrac32 p\lambda_{pm}}_{K_t}\,i_q
$$

For the SPMSM gimbal motor, $i_d$ produces **no torque**. The control law is therefore:
- $i_d^* = 0$ below base speed. This is MTPA for an SPMSM.
- $i_d^* < 0$ only when field weakening.

### 3.4 Current-loop design (follows directly from 3.2)

Apply decoupling feed-forward:

$$
v_d^{ff} = -\omega_e L_q i_q, \qquad v_q^{ff} = \omega_e(L_d i_d + \lambda_{pm})
$$

Each axis then reduces to the first-order plant $G(s) = 1/(L s + R)$.

Choose a PI controller $C(s) = K_p