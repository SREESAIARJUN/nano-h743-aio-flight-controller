# 3-Axis Gimbal on ESP32-S3, Cycle 4: FOC Mathematical Foundations

**Scope:** Clarke and Park transforms, dq machine model, SVPWM, and field weakening (FW), all derived from first principles and then mapped onto ESP32-S3 constraints.

**Assumed motor class:** gimbal-type outrunner SPMSM. This means high pole count, high phase resistance, and (L_d \approx L_q).

**Assumed drive topology:** 3 motors, each driven by 3-PWM inverters (DRV8313 / TMC6300 class).

---

## 0. Notation and Assumptions

| Symbol | Meaning |
|---|---|
| (x_a, x_b, x_c) | Phase quantities (voltage or current) |
| (\vec{x}) | Complex space vector |
| (\theta_e = p\,\theta_m) | Electrical angle, with (p) pole pairs |
| (\omega_e = d\theta_e/dt) | Electrical angular velocity |
| (\psi_m) | PM flux linkage (peak, per phase) |
| (R_s, L_d, L_q) | Phase resistance and dq inductances |
| (V_{dc}, T_s) | DC bus voltage and PWM period |

Model assumptions:
- Wye connection with floating neutral, so (i_a+i_b+i_c=0).
- Sinusoidal back-EMF.
- Linear magnetics (no saturation).
- Iron loss neglected.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-vector definition

Let (a = e^{j2\pi/3}). Define the space vector with a scaling constant (k):

[
\vec{x} = k\left(x_a + a\,x_b + a^2 x_c\right)
]

### 1.2 Choosing k (amplitude invariance)

Take a balanced set:
[
x_a = X\cos\theta,\quad x_b = X\cos(\theta - 2\pi/3),\quad x_c = X\cos(\theta + 2\pi/3)
]

Use Euler's form (\cos\phi = \tfrac12(e^{j\phi}+e^{-j\phi})):

[
x_a + a x_b + a^2 x_c = \frac{X}{2}\Big[\underbrace{e^{j\theta}(1+1+1)}_{3e^{j\theta}} + e^{-j\theta}\underbrace{(1 + a\cdot a + a^2\cdot a^2)}_{1+a^2+a^4 = 1+a^2+a = 0}\Big] = \frac{3}{2}X e^{j\theta}
]

Two scaling choices follow:
- **(k = 2/3)** gives (\vec{x} = X e^{j\theta}). This is **amplitude-invariant**: the vector length equals the phase peak.
- **(k = \sqrt{2/3})** gives a **power-invariant** form.

**Design decision:** use **amplitude-invariant (k=2/3)**. Current limits (A peak) and voltage limits then map directly to phase quantities. The cost is a 3/2 factor in the power and torque equations.

### 1.3 Real-matrix form

Take the real and imaginary parts, using (a = -\tfrac12 + j\tfrac{\sqrt3}{2}) and (a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}):

[
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix} =
\frac{2}{3}\begin{bmatrix} 1 & -\tfrac12 & -\tfrac12 \\ 0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2}\end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c\end{bmatrix}
]

### 1.4 Reduced form using the zero-sum constraint

Substitute (x_b + x_c = -x_a) and (x_c = -x_a - x_b):

[
x_\alpha = \tfrac23\left(x_a + \tfrac12 x_a\right) = x_a,\qquad
x_\beta = \tfrac{1}{\sqrt3}\left(x_b - x_c\right) = \frac{x_a + 2x_b}{\sqrt3}
]

**Implementation consequence:** only **two** current sensors (phases a and b) are needed. A third sensor buys only fault detection: the check (|i_a+i_b+i_c| > \epsilon) flags a ground or sensor fault.

### 1.5 Inverse Clarke

[
x_a = x_\alpha,\qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
]

### 1.6 Power in the αβ frame

Instantaneous power is (P = v_a i_a + v_b i_b + v_c i_c). With the amplitude-invariant transform:

[
P = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right) = \tfrac32\,\mathrm{Re}\{\vec v\,\vec i^{\,*}\}
]

The 3/2 factor is the price of amplitude invariance.

---

## 2. Park Transform (αβ → dq)

### 2.1 Definition

Rotate into a frame aligned with the rotor flux (d-axis on the magnet north pole):

[
\vec{x}_{dq} = \vec{x}_{\alpha\beta}\,e^{-j\theta_e}
]

In matrix form:

[
\begin{bmatrix} x_d \\ x_q\end{bmatrix} =
\begin{bmatrix} \cos\theta_e & \sin\theta_e \\ -\sin\theta_e & \cos\theta_e\end{bmatrix}
\begin{bmatrix} x_\alpha \\ x_\beta\end{bmatrix}
]

The inverse (rotation by (+\theta_e)) is:

[
x_\alpha = x_d\cos\theta_e - x_q\sin\theta_e,\qquad x_\beta = x_d\sin\theta_e + x_q\cos\theta_e
]

### 2.2 Stator voltage equation in dq: where the cross-coupling comes from

Start from the stator