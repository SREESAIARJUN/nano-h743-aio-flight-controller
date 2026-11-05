# 3-Axis Gimbal FOC on ESP32-S3: Mathematical Foundations (Cycle 17)

**Scope.** This document derives the Clarke/Park transforms, the PMSM model in the rotating frame, SVPWM dwell-time equations (including proof that min-max injection is equivalent), and field weakening with its limits. It ends with ESP32-S3 implementation constraints that affect the math.

**Conventions.** Amplitude-invariant scaling is used throughout. θ is the electrical angle, θ = p·θ_m, where p is the number of pole pairs (a typical gimbal motor such as the GBM2804 has p = 7).

---

## 1. Space Vector Foundation

Define the complex rotation operator a = e^{j2π/3}. The stator space vector of any three-phase quantity x ∈ {v, i, λ} is:

$$
\vec{x}_s = k\left(x_a + a\,x_b + a^2 x_c\right)
$$

### 1.1 Choosing k (amplitude invariance)

Take balanced sinusoids: x_a = X cos θ, x_b = X cos(θ − 2π/3), x_c = X cos(θ + 2π/3).

Write each cosine as ½(e^{jφ} + e^{−jφ}):

$$
\sum_{n=0}^{2} a^{n}\cos\!\left(\theta - \tfrac{2\pi n}{3}\right)
= \tfrac12\sum_{n} \left(e^{j\theta} + e^{-j\theta}a^{2n}\right)
= \tfrac12\left(3e^{j\theta} + e^{-j\theta}\underbrace{(1+a^2+a^4)}_{=0}\right)
= \tfrac32 e^{j\theta}
$$

Therefore:
- **k = 2/3** gives |x⃗_s| = X. This is **amplitude-invariant**: the vector length equals the phase peak.
- **k = √(2/3)** gives the **power-invariant** form, where p = v⃗·i⃗ with no extra factor.

We use k = 2/3. The consequence is that power and torque carry a factor of **3/2**, derived in §3.

### 1.2 Clarke Transform

Split x⃗_s = x_α + j x_β, using Re(a) = −½ and Im(a) = √3/2:

$$
\begin{bmatrix} x_\alpha \\ x_\beta \\ x_0 \end{bmatrix}
=
\frac{2}{3}
\begin{bmatrix}
1 & -\tfrac12 & -\tfrac12 \\
0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \\
\tfrac12 & \tfrac12 & \tfrac12
\end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c \end{bmatrix}
$$

For a wye-connected motor with floating neutral, Kirchhoff's current law gives i_a + i_b + i_c = 0, so i_0 = 0. Substituting i_c = −i_a − i_b yields the **two-sensor form**:

$$
\boxed{\,i_\alpha = i_a,\qquad i_\beta = \frac{i_a + 2 i_b}{\sqrt3}\,}
$$

Proof for i_α: (2/3)(i_a − ½(i_b + i_c)) = (2/3)(i_a + ½ i_a) = i_a.

Proof for i_β: (1/√3)(i_b − i_c) = (1/√3)(i_b + i_a + i_b) = (i_a + 2i_b)/√3.

### 1.3 Inverse Clarke

$$
\begin{aligned}
x_a &= x_\alpha + x_0 \\
x_b &= -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta + x_0 \\
x_c &= -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta + x_0
\end{aligned}
$$

The zero-sequence term x_0 is a **free variable** for voltages, because the floating neutral rejects common-mode voltage. Exploiting this freedom is exactly how SVPWM gains its headroom (§4.4).

---

## 2. Park Transform

The Park transform projects onto a frame rotating with the rotor flux:

$$
\vec{x}_{dq} = \vec{x}_{\alpha\beta}\, e^{-j\theta}
$$

$$
\begin{bmatrix} x_d \\ x_q \end{bmatrix} =
\begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix},
\qquad
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix} =
\begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} x_d \\ x_q \end{bmatrix}
$$

The matrix is orthonormal (R⁻¹ = Rᵀ), so Park preserves magnitude: x_d² + x_q² = x_α² + x_β².

**Check.** For i⃗_αβ = I e^{j(θ+β)}, the result is i⃗_dq = I e^{jβ}. This is DC, which is the entire point: PI controllers achieve zero steady-state error on DC references but not on sinusoids.

---

## 3. PMSM Model in the dq Frame

### 3.1 Voltage equations

**Stationary frame:**

$$
\vec v_s = R\,\vec i_s + \frac{d\vec\lambda_s}{dt}
$$

**Rotor frame.** Substitute x⃗_s = x⃗_dq e^{jθ} and apply the product rule:

$$
\frac{d}{dt}\!\left(\vec\lambda_{dq}e^{j\theta}\right) = \left(\dot{\vec\lambda}_{dq} + j\omega\vec\lambda_{dq}\right)e^{j\theta}
$$

Here ω = dθ/dt is the electrical angular velocity. Dividing through by e^{jθ}:

$$
\vec v_{dq} = R\,\vec i_{dq} + \dot{\vec\lambda}_{dq} + j\omega\,\vec\lambda_{