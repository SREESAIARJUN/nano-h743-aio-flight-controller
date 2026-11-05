# 3-Axis Gimbal FOC on ESP32-S3: Mathematical Foundations (Cycle 12)

**Scope:** Clarke and Park transforms, PMSM dynamics in the rotor frame, SVPWM duty-cycle synthesis, and field weakening. Each section ends with the equation that goes into firmware.

---

## 0. Conventions and Plant Assumptions

| Symbol | Meaning |
|---|---|
| $x_a, x_b, x_c$ | Phase quantities (current $i$ or voltage $v$) |
| $\theta_e = p\,\theta_m$ | Electrical angle ($p$ = pole pairs; gimbal motors typically have $p = 7$–$14$) |
| $\omega_e = \dot\theta_e$ | Electrical angular speed |
| $\psi_m$ | Permanent-magnet flux linkage (V·s/rad) |
| $R_s, L_d, L_q$ | Stator resistance and dq-axis inductances |
| $V_{dc}$ | DC bus voltage |
| $a = e^{j2\pi/3}$ | 120° rotation operator |

**Assumptions:**
- Wye-connected winding with an isolated neutral, so $i_a + i_b + i_c = 0$.
- Sinusoidal back-EMF.
- No magnetic saturation (relaxed in §4.6).

**Gimbal context.** Gimbal BLDCs are surface-mounted PMSMs (SPMSM), so $L_d \approx L_q = L_s$. They have high $R_s$ (5–20 Ω) and operate at low speed and high torque. This matters in §4, because it makes field weakening a marginal feature for this product.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-Vector Definition

Define the complex stator space vector:

$$
\vec{x}_s = K\left(x_a + a\,x_b + a^2 x_c\right) = x_\alpha + j\,x_\beta
$$

The scale factor $K$ is chosen by the invariance we want to preserve.

### 1.2 Deriving $K$ for Amplitude Invariance

Consider a balanced set:

$$
x_a = X\cos\theta,\quad x_b = X\cos(\theta - 2\pi/3),\quad x_c = X\cos(\theta + 2\pi/3)
$$

Write each cosine using Euler's identity, $\cos\phi = \tfrac12(e^{j\phi} + e^{-j\phi})$:

$$
x_a + a x_b + a^2 x_c = \frac{X}{2}\Big[\underbrace{e^{j\theta}(1 + a a^{-1} + a^2 a^{-2})}_{3e^{j\theta}} + \underbrace{e^{-j\theta}(1 + a\cdot a + a^2\cdot a^2)}_{e^{-j\theta}(1 + a^2 + a^4)}\Big]
$$

Since $a^4 = a$, the second bracket becomes $1 + a + a^2 = 0$. Therefore:

$$
x_a + a x_b + a^2 x_c = \tfrac{3}{2}X e^{j\theta}
$$

Choosing $K = \tfrac{2}{3}$ gives $\vec{x}_s = X e^{j\theta}$. The vector magnitude then equals the phase peak amplitude. This is the **amplitude-invariant Clarke transform**, which we use throughout.

### 1.3 Matrix Form

Expand $a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

$$
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_0 \end{bmatrix}
= \frac{2}{3}\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12 \\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\
\tfrac12 & \tfrac12 & \tfrac12
\end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c \end{bmatrix}
$$

### 1.4 Two-Sensor Reduction

Substitute $x_c = -x_a - x_b$:

$$
x_\alpha = \tfrac{2}{3}\left(x_a + \tfrac12 x_a + \tfrac12 x_b - \tfrac12 x_b\right) = x_a
$$

$$
x_\beta = \tfrac{2}{3}\cdot\tfrac{\sqrt3}{2}\left(x_b + x_a + x_b\right) = \frac{x_a + 2x_b}{\sqrt3}
$$

Firmware form:

$$
\boxed{\,i_\alpha = i_a,\qquad i_\beta = \tfrac{1}{\sqrt3}\,(i_a + 2 i_b)\,}
$$

This needs two shunts, which is the ESP32-S3 ADC budget per motor.

### 1.5 Power Bookkeeping

The amplitude-invariant transform is not power-invariant:

$$
p = v_a i_a + v_b i_b + v_c i_c = \tfrac{3}{2}\left(v_\alpha i_\alpha + v_\beta i_\beta\right)
$$

The power-invariant variant uses $K = \sqrt{2/3}$, which makes the matrix orthonormal. The $\tfrac32$ factor reappears in the torque equation (§2.4).

### 1.6 Inverse Clarke

$$
x_a = x_\alpha,\qquad x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\qquad x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

---

## 2. Park Transform (αβ → dq) and the PMSM Model

### 2.1 Derivation

Express the stationary vector in a frame rotating with the rotor flux at angle $\theta_e$:

$$
\vec{x}_{dq} = \vec{x}_s\, e^{-j\theta_e}
$$

Expanding $(x_\alpha + jx_\beta)(\cos\theta_e - j\sin\theta_e)$ gives:

$$
\boxed{\begin{aligned}
x_d &= \;\;\,x_\alpha\cos\theta_e + x_\beta\sin\theta_e\\
x_q &= -x_\alpha\sin\theta_e + x_\beta\cos\theta_e
\end{aligned}}
\qquad
\boxed{\begin{aligned}
x_\alpha &= x_d\cos\theta_e - x_q\sin\theta_e\\
x_\beta &= x_d\sin\theta_e + x_q\cos\theta_e
\end{aligned}}
$$

The rotation matrix $\mathbf{R}(\theta)$ is orthogonal ($\mathbf{R}^{-1} = \mathbf{R}^T$), so Park preserves both magnitude and power scaling. A balanced sinusoidal current $I e^{j(\omega t + \phi)}$ maps to the DC vector $I e^{j\phi}$ when $\theta_e = \omega t$. This turns the AC tracking problem into DC regulation, which a PI controller handles with zero steady-state error.

### 2.2 Stator Voltage Equation

In the stationary frame:

$$
\vec v_s = R_s \vec i_s + \frac{d\vec\psi_s}{dt}
$$

Substitute $\vec\psi_s = \vec\psi_{dq}e^{j\theta_e}$ and apply the product rule:

$$
\frac{d\vec\psi_s}{dt} = \left(\frac{d\vec\psi_{dq}}{dt} + j\omega_e\vec\psi_{dq}\right)e^{j\theta_e}
$$

Multiply through by $e^{-j\theta_e}$:

$$
\vec v_{dq} = R_s\vec i_{dq} + \frac{d\vec\psi_{dq}}{dt} + j\omega_e\vec\psi_{dq}
$$

The rotor-frame flux is $\psi_d = L_d i_d + \psi_m$ and $\psi_q = L_q i_q$. Split into real and imaginary parts:

$$
\boxed{\begin{aligned}
v_d &= R_s i_d + L_d\frac{di_d}{dt} - \omega_e L_q i_q\\
v_q &= R_s i_q + L_q\frac{di_q}{dt} + \omega_e L_d i_d + \omega_e\psi_m
\end{aligned}}
$$

The $j\omega_e\vec\psi$ term produces:
- the cross-coupling terms $\mp\omega_e L i$, which are cancelled by decoupling feedforward;
- the back-EMF $\omega_e\psi_m$.

### 2.3 Current-Loop Design (Consequence)

With decoupling feedforward $v_d^{ff} = -\omega_e L_q i_q$ and $v_q^{ff} = \omega_e(L_d i_d + \psi_m)$, each axis reduces to the first-order plant $1/(L s + R)$.

Use PI pole-zero cancellation for a closed-loop bandwidth $\omega_{bw}$:

$$
K_p = L\,\omega_{bw},\qquad K_i = R\,\omega_{bw}
$$

The closed loop becomes $\omega_{bw}/(s + \omega_{bw})$. Keep $\omega_{bw} \le \omega_{PWM}/20$. At 20 kHz this g