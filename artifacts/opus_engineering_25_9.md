# 3-Axis Gimbal FOC on ESP32-S3: Mathematical Derivation of the Transforms, SVPWM Duty Equations and Field-Weakening

This covers the reference-frame mathematics, the PWM synthesis and the speed-extension control law for the motor-control layer. Every equation below maps to a block in the firmware. Section 6 maps them onto the ESP32-S3 hardware.

---

## 0. Notation and Motor Model Assumptions

| Symbol | Meaning |
|---|---|
| $x_a, x_b, x_c$ | Phase quantities (voltage or current) |
| $\theta_e = p\,\theta_m$ | Electrical angle ($p$ = pole pairs; typically 7 for a gimbal motor such as GBM2804) |
| $\omega = \dot\theta_e$ | Electrical angular velocity |
| $R_s, L_d, L_q$ | Phase resistance and synchronous inductances |
| $\psi_f$ | Permanent-magnet flux linkage (per phase, peak) |
| $V_{dc}$ | DC bus voltage |
| $T_s$ | PWM period |

Assumptions:
- Balanced, star-connected, isolated neutral.
- Sinusoidal back-EMF.
- No saturation, so $L_d$ and $L_q$ are constant.

Gimbal motors are surface-mount PMSMs, so $L_d \approx L_q = L_s$. The general IPM case ($L_d \neq L_q$) is still carried through because it changes the torque and field-weakening results.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-vector definition

Define the rotation operator $a = e^{j2\pi/3}$. The stator space vector is

$$
\vec{x} = x_\alpha + j x_\beta = K\left(x_a + a\,x_b + a^2 x_c\right)
$$

$K$ is a free scaling constant. Expand $a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$, then separate real and imaginary parts:

$$
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix}
= K\begin{bmatrix} 1 & -\tfrac12 & -\tfrac12 \\[2pt] 0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2}\end{bmatrix}
\begin{bmatrix} x_a\\x_b\\x_c\end{bmatrix}
$$

### 1.2 Choosing $K$ (amplitude invariance)

Apply a balanced set: $x_a = X\cos\phi$, $x_b = X\cos(\phi - 2\pi/3)$, $x_c = X\cos(\phi + 2\pi/3)$.

Write each cosine as $\tfrac{X}{2}(e^{j\cdot}+e^{-j\cdot})$:

$$
x_a + a x_b + a^2 x_c = \frac{X}{2}\Big[e^{j\phi}(1 + 1 + 1) + e^{-j\phi}(1 + a^2 a^{-1}\cdot a^{\,0}\ldots)\Big]
$$

More cleanly:
- The positive-sequence terms combine as $e^{j\phi}(1 + a\cdot a^{-1} + a^2 \cdot a^{-2}) = 3e^{j\phi}$.
- The negative-sequence terms give $e^{-j\phi}(1 + a^2 + a^4) = e^{-j\phi}(1 + a + a^2) = 0$.

Hence

$$
\vec{x} = K \cdot \tfrac{3}{2} X e^{j\phi}
$$

There are two standard choices of $K$:

- **$K = 2/3$ (amplitude-invariant).** Then $|\vec x| = X$, so the peak phase quantity equals the vector magnitude. **This is the firmware choice**, because current limits, ADC scaling and $V_{dc}$ limits stay in physical phase units.
- **$K = \sqrt{2/3}$ (power-invariant).** This makes the transform orthonormal, so $p = v_\alpha i_\alpha + v_\beta i_\beta$.

With $K = 2/3$, power picks up a factor:

$$
p = \tfrac32\,(v_\alpha i_\alpha + v_\beta i_\beta)
$$

### 1.3 Reduced form for the firmware

Because the neutral is isolated, $i_a + i_b + i_c = 0$. Substitute $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a + \tfrac12 i_a + \tfrac12 i_b - \tfrac12 i_b\right) = i_a
$$

$$
i_\beta = \tfrac23\cdot\tfrac{\sqrt3}{2}\,(i_b - i_c) = \tfrac{1}{\sqrt3}(i_b + i_a + i_b) = \frac{i_a + 2 i_b}{\sqrt3}
$$

Two shunts are therefore sufficient. The cost is two multiply-adds.

---

## 2. Park Transform (αβ → dq)

### 2.1 Definition

Rotate the stationary vector into a frame aligned with the rotor flux at angle $\theta_e$:

$$
\vec{x}_{dq} = \vec{x}_{\alpha\beta}\, e^{-j\theta_e}
$$

$$
\begin{bmatrix} x_d\\x_q\end{bmatrix} =
\begin{bmatrix}\cos\theta_e & \sin\theta_e\\ -\sin\theta_e & \cos\theta_e\end{bmatrix}
\begin{bmatrix} x_\alpha\\x_\beta\end{bmatrix}
$$

The inverse transform (dq → αβ) is the transpose, since the rotation matrix is orthogonal:

$$
\begin{bmatrix} x_\alpha\\x_\beta\end{bmatrix} =
\begin{bmatrix}\cos\theta_e & -\sin\theta_e\\ \sin\theta_e & \cos\theta_e\end{bmatrix}
\begin{bmatrix} x_d\\x_q\end{bmatrix}
$$

### 2.2 Derivation of the dq machine model

**Stationary-frame stator equation:**

$$
\vec v_{\alpha\beta} = R_s \vec i_{\alpha\beta} + \frac{d\vec\psi_{\alpha\beta}}{dt}
$$

**Change of frame.** Substitute $\vec x_{\alpha\beta} = \vec x_{d