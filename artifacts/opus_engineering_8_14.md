# 3-Axis Gimbal FOC on ESP32-S3: Mathematical Foundations (Cycle 8)

**Scope:** Clarke/Park transforms, SVPWM duty-cycle synthesis, and field-weakening, derived from first principles and then mapped onto ESP32-S3 hardware constraints.

---

## 0. System Context and Notation

| Symbol | Meaning |
|---|---|
| $p$ | pole-pair count (typical gimbal motor: 7, i.e. 14 poles) |
| $\theta_m, \theta_e = p\,\theta_m$ | mechanical and electrical angle |
| $\omega = \dot\theta_e$ | electrical angular velocity |
| $R_s, L_d, L_q$ | phase resistance and dq inductances |
| $\lambda_m$ | permanent-magnet flux linkage (peak, per phase) |
| $V_{dc}$ | DC bus voltage |
| $T_s$ | PWM period |

**Plant assumption:** a surface-mounted PMSM (SPMSM) with $L_d \approx L_q = L_s$. This is valid for nearly all hollow-shaft gimbal motors such as the 2804, 4108 and 5208 classes. The general IPMSM case is carried along wherever it changes a conclusion.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Geometric derivation

The three stator windings are spatially displaced by $120^\circ$. Define the unit axis vectors in the complex plane:

$$
\hat a = e^{j0},\qquad \hat b = e^{j2\pi/3},\qquad \hat c = e^{j4\pi/3}
$$

The **space vector** of any three-phase quantity $x_{a,b,c}$ is the scaled vector sum:

$$
\vec x = K\left(x_a + x_b\,e^{j2\pi/3} + x_c\,e^{j4\pi/3}\right) = x_\alpha + j x_\beta
$$

Expanding $e^{j2\pi/3} = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $e^{j4\pi/3} = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

$$
\boxed{
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix}
= K
\begin{bmatrix} 1 & -\tfrac12 & -\tfrac12 \\[2pt] 0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c \end{bmatrix}}
$$

### 1.2 Choosing the scaling constant $K$

Apply a balanced set $x_k = X\cos(\theta - 2\pi k/3)$ for $k = 0, 1, 2$. Using the identity

$$
\sum_{k=0}^{2}\cos\!\left(\theta - \tfrac{2\pi k}{3}\right)e^{j2\pi k/3} = \tfrac32\,e^{j\theta},
$$

we get $\vec x = \tfrac32 K X e^{j\theta}$. Two conventions follow:

- **Amplitude-invariant** ($|\vec x| = X$): $K = \tfrac23$. **This is the one adopted here.** Phase-current amplitude equals the αβ vector magnitude, which makes current limits and ADC scaling intuitive.
- **Power-invariant** (the matrix is orthonormal): $K = \sqrt{2/3}$.

**Power consequence of the amplitude-invariant choice.** Power is not preserved one-to-one, so a factor appears:

$$
P = v_a i_a + v_b i_b + v_c i_c = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right)
$$

The $\tfrac32$ factor in the torque equation (Section 2.4) originates exactly here.

### 1.3 Reduced form (star connection, no neutral)

Kirchhoff's current law gives $i_a + i_b + i_c = 0$. Substituting $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a - \tfrac12 i_b + \tfrac12 i_a + \tfrac12 i_b\right) = i_a
$$

$$
i_\beta = \tfrac23\cdot\tfrac{\sqrt3}{2}\left(i_b + i_a + i_b\right) = \frac{i_a + 2 i_b}{\sqrt3}
$$

$$
\boxed{\,i_\alpha = i_a,\qquad i_\beta = \tfrac{1}{\sqrt3}\left(i_a + 2i_b\right)\,}
$$

**Implementation consequence:** only two phase-current sensors are needed per motor. That is 6 ADC channels for 3 axes instead of 9, which matters on the ESP32-S3 (see Section 5).

### 1.4 Zero-sequence component

The full invertible transform carries a third row, $x_0 = \tfrac13(x_a + x_b + x_c)$. For currents in a floating star this is identically zero.

For voltages it is **not** zero. It is precisely the degree of freedom that SVPWM exploits (Section 3.5).

---

## 2. Park Transform (αβ → dq)

### 2.1 Derivation as a frame rotation

We want a frame that rotates synchronously with the rotor flux, at angle $\theta_e$. Any vector expressed in the stationary frame is re-expressed in the rotating frame by multiplying by $e^{-j\theta_e}$:

$$
\vec x_{dq} = \vec x_{\alpha\beta}\,e^{-j\theta_e}
$$

$$
\boxed{
\begin{bmatrix} x_d \\ x_q \end{bmatrix} =
\begin{bmatrix} \cos\theta_e & \sin\theta_e \\ -\sin\theta_e & \cos\theta_e \end{bmatrix}
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix}}
\qquad
\boxed{
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix} =
\begin{bmatrix} \cos\theta_e & -\sin\theta_e \\ \sin\