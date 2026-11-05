# 3-Axis Gimbal FOC Core: Mathematical Foundations

**Cycle 19, Motor Control Architecture**

This document derives the transforms and modulation law from first principles, adds the corrections that matter for gimbal motors, and maps the results to ESP32-S3 hardware.

The headline result is that **for high-resistance gimbal motors, the textbook field-weakening equations (which neglect R) are wrong.** Section 5 gives the exact form that includes R.

---

## 0. Plant Model and Assumptions

**Motor:** surface-mount PMSM (typical gimbal BLDC, e.g. 2804/4108 class, 7–11 pole pairs, wound for high resistance).

**Assumptions:**
1. Three-phase, wye-connected, floating neutral, so $i_a + i_b + i_c = 0$.
2. Sinusoidal back-EMF, linear magnetics (no saturation), negligible iron loss.
3. Surface magnets, so $L_d \approx L_q = L$ (salient form retained where it costs nothing).

**Phase-domain model:**

$$
\mathbf{v}_{abc} = R\,\mathbf{i}_{abc} + \frac{d\boldsymbol{\lambda}_{abc}}{dt},\qquad
\boldsymbol{\lambda}_{abc} = \mathbf{L}_{abc}(\theta)\,\mathbf{i}_{abc} + \psi_m
\begin{bmatrix}\cos\theta\\ \cos(\theta-\tfrac{2\pi}{3})\\ \cos(\theta+\tfrac{2\pi}{3})\end{bmatrix}
$$

Here $\theta = p\,\theta_m$ is the electrical angle, $p$ the pole-pair count, and $\psi_m$ the peak per-phase magnet flux linkage.

**Illustrative parameter set** (used in all numerical examples):

| Symbol | Value | Note |
|---|---|---|
| $R$ | 10 Ω | phase |
| $L$ | 2 mH | synchronous, phase |
| $p$ | 7 | pole pairs |
| $K_V$ | 220 rpm/V | line-line peak convention |
| $\psi_m$ | ≈ 3.58 mWb | derived below |
| $V_{dc}$ | 12 V | 3S LiPo nominal |
| $I_{max}$ | 1.0 A | thermal limit |

**Deriving $\psi_m$ from $K_V$.** The line-line peak back-EMF is $\sqrt{3}\,\omega_e\psi_m$, with $\omega_e = p\,\omega_m$ and $\omega_m = 2\pi K_V V/60$. Therefore:

$$\psi_m = \frac{60}{\sqrt3\cdot 2\pi\cdot K_V\cdot p} = \frac{60}{1.732\cdot 6.283\cdot 220\cdot 7}\approx 3.58\ \text{mWb}$$

---

## 1. Clarke Transform

### 1.1 Space-vector definition

Let $a = e^{j2\pi/3}$. Define the complex space vector

$$\vec{x} = k\left(x_a + a\,x_b + a^2 x_c\right)$$

### 1.2 Choosing the scaling constant $k$

Apply a balanced set $x_a = X\cos\theta$, $x_b = X\cos(\theta-2\pi/3)$, $x_c = X\cos(\theta+2\pi/3)$. Write each cosine as $\tfrac12(e^{j\phi}+e^{-j\phi})$:

$$
x_a + a x_b + a^2 x_c = \tfrac{X}{2}\Big[e^{j\theta}(1 + a\,a^{-1} + a^2 a^{-2}) + e^{-j\theta}(1 + a\cdot a + a^2\cdot a^2)\Big]
$$

The first bracket is $3$. The second is $1 + a^2 + a^4 = 1 + a^2 + a = 0$. Hence

$$x_a + a x_b + a^2 x_c =