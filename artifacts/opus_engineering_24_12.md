# 3-Axis Gimbal on ESP32-S3: FOC Transforms, SVPWM, and Field Weakening

**Document:** GMB-S3-ARCH-024, Motor Control Mathematical Foundations

This document derives the Clarke and Park transforms, the PMSM model in the rotor frame, SVPWM duty-cycle equations, MTPA, and field weakening. Every equation used in firmware is derived here. The final sections map the math onto ESP32-S3 hardware and give reference code.

---

## 0. System Context and Assumptions

| Item | Assumption |
|---|---|
| Motors | 3× gimbal PMSM/BLDC (e.g., GBM2804/GM4108 class), 7–11 pole pairs, R_s ≈ 5–15 Ω, L ≈ 1–4 mH, surface-mounted magnets (L_d ≈ L_q) |
| Driver | 3-PWM-input half-bridge IC (DRV8313 / TMC6300 class), dead time handled in hardware |
| Sensing | 14-bit absolute magnetic encoder per axis (MT6701/AS5048A over SSI/SPI); low-side or in-line shunts on at least 2 phases |
| Bus | V_dc = 12–16.8 V (3S–4S) |
| MCU | ESP32-S3, dual Xtensa LX7 at 240 MHz, single-precision FPU, 2× MCPWM units |

**Notation.** Space vectors are complex: x = x_α + j·x_β. θ_e is the electrical angle, θ_e = p·θ_m, where p is the number of pole pairs. ω = dθ_e/dt.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-Vector Definition

Let a = e^{j2π/3}. The stator space vector of any three-phase quantity is

$$
\vec{x}_s = k\left(x_a + a\,x_b + a^2 x_c\right)
$$

Here k is a scaling constant chosen by convention.

### 1.2 Choosing k: Amplitude Invariance

Take a balanced set:

- x_a = X cos θ
- x_b = X cos(θ − 2π/3)
- x_c = X cos(θ + 2π/3)

Write each cosine as cos φ = (e^{jφ} + e^{−jφ})/2:

$$
\sum_{n=0}^{2} a^n X\cos\!\left(\theta - \tfrac{2\pi n}{3}\right)
= \frac{X}{2}\sum_{n=0}^{2}\left(e^{j\theta} + e^{-j\theta}\,a^{2n}\right)
= \frac{X}{2}\left(3e^{j\theta} + e^{-j\theta}\underbrace{(1 + a^2 + a^4)}_{=0}\right)
= \frac{3}{2}X e^{j\theta}
$$

Two choices of k follow:

- **k = 2/3 (amplitude-invariant):** |x_s| = X, so the vector length equals the phase peak. This is the firmware convention.
- **k = √(2/3) (power-invariant):** the transform matrix is orthonormal.

### 1.3 Component Form

Expand a = −½ + j√3/2 and a² = −½ − j√3/2:

$$
x_\alpha = \tfrac{2}{3}\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right),\qquad
x_\beta = \tfrac{2}{3}\cdot\tfrac{\sqrt3}{2}\left(x_b - x_c\right) = \tfrac{1}{\sqrt3}(x_b - x_c)
$$

The full matrix, including the zero-sequence row, is

$$
\begin{bmatrix}x_\alpha\\x_\beta\\x_0\end{bmatrix}
=\frac{2}{3}
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12\\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2}\\
\tfrac12 & \tfrac12 & \tfrac12
\end{bmatrix}
\begin{bmatrix}x_a\\x_b\\x_c\end{bmatrix}
$$

### 1.4 Two-Shunt Reduction

The star point is floating, so i_a + i_b + i_c = 0. Substitute i_c = −i_a − i_b:

$$
i_\alpha = \tfrac23\left(i_a + \tfrac12 i_a\right) = i_a,\qquad
i_\beta = \tfrac{1}{\sqrt3}(i_b + i_a + i_b) = \frac{i_a + 2i_b}{\sqrt3}
$$

Only two ADC channels are needed per motor. If three shunts are fitted, use the two phases whose low-side on-time is longest in the current PWM sector. This handles the measurement-window problem at high modulation.

### 1.5 Power Scaling

With amplitude-invariant scaling:

$$
p = v_a i_a + v_b i_b + v_c i_c = \tfrac32\,\mathrm{Re}\{\vec v_s \vec i_s^{\,*}\} + 3v_0 i_0 = \tfrac32(v_\alpha i_\alpha + v_\beta i_\beta)
$$

The zero-sequence term vanishes because i_0 = 0. This factor of 3/2 is the origin of the 3/2 in the torque equation (§3).

### 1.6 Inverse Clarke

Zero sequence is free, so x_0 = 0:

$$
x_a = x_\alpha,\qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

---

## 2. Park Transform (αβ → dq)

### 2.1 Derivation

Define a frame rotating with the rotor flux, with the d-axis aligned to the magnet north pole at angle θ_e. A vector expressed in this frame is

$$
\vec x_{dq} = \vec x_s\, e^{-j\theta_e}
$$

Expanding (x_α + j·x_β)(cos θ − j·sin θ):

$$
\boxed{\;x_d = x_\alpha\cos\theta_e + x_\beta\sin\theta_e,\qquad
x_q = -x_\alpha\sin\theta_e + x_\beta\cos\theta_e\;}
$$

### 2.2 Inverse Park

$$
x_\alpha = x_d\cos\theta_e - x_q\sin\theta_e,\qquad
x_\beta = x_d\sin\theta_e + x_q\cos\theta_e
$$

The rotation matrix R(θ) is orthogonal, so R⁻¹ = Rᵀ. Magnitudes are preserved: |x_dq| = |x_αβ|.

### 2.3 Why the Rotation Works

In steady state, balanced sinusoidal currents form a vector of const