# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundations and Implementation

**Cycle 18 Design Document: Motor Control Core**

| Field | Value |
|---|---|
| Scope | Clarke/Park transforms, SVPWM duty synthesis, field weakening, ESP32-S3 mapping |
| Target plant | 3 × surface-mount PMSM gimbal motors (high-R, high pole count, SPM, L_d ≈ L_q) |
| Target MCU | ESP32-S3 (dual Xtensa LX7 @ 240 MHz, single-precision FPU, 2 × MCPWM) |

---

## 0. Conventions and Assumptions

| Symbol | Meaning |
|---|---|
| x_a, x_b, x_c | Phase quantities (voltage, current, or flux) |
| θ_e = p·θ_m | Electrical angle, where p is the number of pole pairs |
| ω = dθ_e/dt | Electrical angular velocity (rad/s) |
| R, L_d, L_q | Phase resistance and d/q-axis inductances |
| ψ_m | Permanent-magnet flux linkage (V·s/rad, phase peak) |
| V_dc | DC bus voltage |
| T_s | PWM period |

The motor is assumed to be wye-connected with an isolated neutral, so i_a + i_b + i_c = 0. The spatial operator is a = e^{j2π/3}.

---

## 1. Clarke Transform (abc → αβ)

### 1.1 Space-Vector Definition

Three physical windings displaced by 120° in space produce a resultant MMF/flux vector in the stator plane. This is defined as the complex space vector

$$
\vec{x} = k\left(x_a + a\,x_b + a^2 x_c\right) = x_\alpha + j x_\beta
$$

where k is a scaling constant still to be chosen.

### 1.2 Determining k

Substitute a balanced sinusoidal set:

$$
x_a = X\cos\theta,\quad x_b = X\cos(\theta - \tfrac{2\pi}{3}),\quad x_c = X\cos(\theta + \tfrac{2\pi}{3})
$$

Write each cosine as cos φ = ½(e^{jφ} + e^{-jφ}):

$$
x_a + a x_b + a^2 x_c = \frac{X}{2}\Big[e^{j\theta}(1 + a\,a^{-1} + a^2 a^{-2}) + e^{-j\theta}(1 + a\,a + a^2 a^2)\Big]
$$

The first bracket equals 3. The second bracket is 1 + a² + a⁴ = 1 + a² + a = 0. Therefore

$$
x_a + a x_b + a^2 x_c = \tfrac{3}{2}X e^{j\theta}
$$

Two choices of k follow:

- **Amplitude-invariant (k = 2/3).** Then |x⃗| = X, the phase peak. **This is the convention adopted here**, because current limits, flux, and PI gains then map directly to phase quantities.
- **Power-invariant (k = √(2/3)).** The matrix becomes orthonormal, but amplitudes are scaled by √(3/2).

### 1.3 Component Form

Use a = −½ + j√3/2 and a² = −½ − j√3/2:

$$
x_\alpha = \tfrac{2}{3}\left(x_a - \tfrac12 x_b - \tfrac12 x_c\right),\qquad
x_\beta = \tfrac{2}{3}\cdot\tfrac{\sqrt3}{2}(x_b - x_c) = \tfrac{1}{\sqrt3}(x_b - x_c)
$$

The full matrix, including the zero-sequence row x_0, is

$$
\begin{bmatrix}x_\alpha\\x_\beta\\x_0\end{bmatrix}=\frac23
\begin{bmatrix}1&-\tfrac12&-\tfrac12\\0&\tfrac{\sqrt3}{2}&-\tfrac{\sqrt3}{2}\\\tfrac12&\tfrac12&\tfrac12\end{bmatrix}
\begin{bmatrix}x_a\\x_b\\x_c\end{bmatrix}
$$

### 1.4 Reduced Two-Sensor Form

Impose x_c = −x_a − x_b (isolated neutral). Then:

- x_α = (2/3)(x_a + ½x_a) = **x_a**
- x_β = (1/√3)(x_b + x_a + x_b) = **(x_a + 2x_b)/√3**

Only two current sensors are therefore required.

### 1.5 Inverse Clarke

$$
x_a = x_\alpha,\quad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\quad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

### 1.6 Power Relation

Because the amplitude-invariant transform is not orthonormal (T Tᵀ ≠ I), instantaneous power carries a 3/2 factor:

$$
P = v_a i_a + v_b i_b + v_c i_c = \tfrac32\left(v_\alpha i_\alpha + v_\beta i_\beta\right) + 3 v_0 i_0
$$

With an isolated neutral, i_0 = 0, so the last term vanishes. This 3/2 propagates into the torque equation (§2.4).

---

## 2. Park Transform (αβ → dq)

### 2.1 Definition

Project the stationary vector onto a frame rotating with the rotor flux at angle θ_e:

$$
\vec{x}_{dq} = \vec{x}_{\alpha\beta}\,e^{-j\theta_e}
$$

$$
\begin{bmatrix}x_d\\x_q\end{bmatrix}=
\begin{bmatrix}\cos\theta_e&\sin\theta_e\\-\sin\theta_e&\cos\theta_e\end{bmatrix}
\begin{bmatrix}x_\alpha\\x_\beta\end{bmatrix}
$$

**Inverse Park:**

$$
x_\alpha = x_d\cos\theta_e - x_q\sin\theta_e,\qquad x_\beta = x_d\sin\theta_e + x_q\cos\theta_e
$$

This is a pure rotation, so it is orthonormal and preserves magnitude: |x_dq| = |x_αβ|.

### 2.2 Derivation of the dq Machine Model

The stator voltage equation in the stationary frame is

$$
\vec v_{\alpha\beta} = R\,\vec i_{\alpha\beta} + \frac{d\vec\psi_{\alpha\beta}}{dt}
$$

Substitute v⃗_αβ = v⃗_dq e^{jθ_e} and ψ⃗_αβ = ψ⃗_dq e^{jθ_e}. Differentiating the product gives

$$
\frac{d}{dt}\left(\vec\psi_{dq}e^{j\theta_e}\right) = \left(\dot