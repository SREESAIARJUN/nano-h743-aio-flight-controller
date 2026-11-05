# 3-Axis Gimbal FOC Core: Mathematical Derivation
**Target:** ESP32-S3 · **Document:** Cycle 13 · **Scope:** Clarke/Park transforms, SVPWM, field weakening

---

## 0. Conventions and Assumptions

| Symbol | Meaning |
|---|---|
| $p$ | pole pairs (typical gimbal motor: 7–11) |
| $\theta_e = p\,\theta_m$ | electrical angle |
| $\omega_e = \dot\theta_e$ | electrical speed |
| $R_s,\ L_d,\ L_q$ | phase resistance and dq inductances |
| $\lambda_m$ | PM flux linkage (peak, per phase) |
| $V_{dc}$ | DC link voltage |
| $T_s$ | PWM period |

**Assumptions:**
- Balanced, star-connected, isolated neutral.
- Sinusoidal back-EMF.
- Linear magnetics (no saturation) unless stated.
- **Amplitude-invariant scaling** ($k = 2/3$) is used throughout. This choice is justified in §1.3.

---

## 1. Clarke Transform (abc → αβ0)

### 1.1 Space-vector definition

Let $a = e^{j2\pi/3}$. The stator space vector is defined as

$$
\vec{x}_s = k\left(x_a + a\,x_b + a^2 x_c\right), \qquad x \in \{v, i, \psi\}
$$

Using $\Re(a) = \Re(a^2) = -\tfrac12$, $\Im(a) = \tfrac{\sqrt3}{2}$ and $\Im(a^2) = -\tfrac{\sqrt3}{2}$, the real and imaginary parts are:

$$
x_\alpha = k\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right), \qquad
x_\beta = k\,\tfrac{\sqrt3}{2}\left(x_b - x_c\right)
$$

A zero-sequence component completes the transform:

$$
x_0 = \frac{k}{\sqrt{2}\cdot\ldots}\ \text{(convention-dependent)}; \quad \text{amplitude-invariant: } x_0 = \tfrac13(x_a + x_b + x_c)
$$

### 1.2 Choosing $k$: proof of amplitude invariance

Take a balanced set:

$$
x_a = X\cos\theta, \quad x_b = X\cos(\theta - 2\pi/3), \quad x_c = X\cos(\theta + 2\pi/3)
$$

Write each cosine as $\tfrac12\left(e^{j\phi} + e^{-j\phi}\right)$. Then

$$
\sum_{n=0}^{2} a^n \cos\!\left(\theta - \tfrac{2\pi n}{3}\right)
= \tfrac12\sum_{n} \left(e^{j\theta} + e^{-j\theta}\,a^{2n}\right)
= \tfrac12\left(3e^{j\theta} + e^{-j\theta}\underbrace{(1 + a^2 + a^4)}_{=0}\right)
= \tfrac32 e^{j\theta}
$$

Therefore:
- $k = \tfrac23$ gives $\vec{x}_s = X e^{j\theta}$. **Peak phase amplitude is preserved.**
- $k = \sqrt{2/3}$ gives the orthonormal, **power-invariant** form.

### 1.3 Power relation

With $k = 2/3$, inverting gives $x_a = x_\alpha + x_0$, and similarly for the other phases. Substituting:

$$
p(t) = v_a i_a + v_b i_b + v_c i_c = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right) + 3 v_0 i_0
$$

**Why amplitude-invariant is chosen:**
- With an isolated neutral, $i_0 \equiv 0$.
- The $\tfrac32$ factor reappears in the torque equation.
- The benefit: $|\vec{i}_{dq}|$ equals the **peak phase current**. Current limits, ADC full-scale, and driver ratings therefore map 1:1 onto dq quantities, with no hidden $\sqrt{3/2}$ factor.

### 1.4 Reduced (2-shunt) form

Using $i_a + i_b + i_c = 0$, so $i_c = -i_a - i_b$:

$$
\boxed{\,i_\alpha = i_a, \qquad i_\beta = \frac{i_a + 2i_b}{\sqrt3}\,}
$$

This needs only two ADC channels, which matters given the ESP32-S3 ADC throughput.

### 1.5 Inverse Clarke

$$
x_a = x_\alpha, \qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta, \qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

---

## 2. Park Transform (αβ → dq)

### 2.1 Definition

Rotate into the rotor frame, with the d-axis aligned to the PM flux:

$$
\vec{x}_{dq} = \vec{x}_{\alpha\beta}\,e^{-j\theta_e}
\;\;\Longrightarrow\;\;
\begin{bmatrix} x_d \\ x_q \end{bmatrix}
=
\begin{bmatrix} \cos\theta_e & \sin\theta_e \\ -\sin\theta_e & \cos\theta_e \end{bmatrix}
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix}
$$

The inverse is the transpose (rotation by $+\theta_e$):

$$
x_\alpha = x_d\cos\theta_e - x_q\sin\theta_e, \qquad
x_\beta = x_d\sin\theta_e + x_q\cos\theta_e
$$

### 2.2 Derivation of the dq machine model

**Step 1: stationary-frame voltage equation.**

$$
\vec{v}^{\,s} = R_s\,\vec{i}^{\,s} + \frac{d\vec{\psi}^{\,s}}{dt}
$$

**Step 2: substitute the rotating-frame quantity.** Let $\vec{\psi}^{\,s} = \vec{\psi}^{\,r} e^{j\theta_e}$. By the product rule:

$$
\frac{d\vec\psi^{\,s}}{dt} = \left(\frac{d\vec\psi^{\,r}}{dt} + j\omega_e\vec\psi^{\,