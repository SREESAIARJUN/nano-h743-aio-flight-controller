# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Foundations

**Cycle 23: Clarke/Park Transforms, SVPWM, and Field Weakening**

---

## 0. System Context and Assumptions

Every derivation below rests on these assumptions, and they also constrain the firmware.

| Parameter | Typical gimbal value (e.g., GBM2804-100T class) | Consequence |
|---|---|---|
| Pole pairs $p$ | 7 | $\omega_e = 7\,\omega_m$ |
| Phase resistance $R_s$ | 5–12 Ω | Resistive drop is **not** negligible in the voltage budget (see §4.6) |
| Phase inductance $L_s$ | 1–4 mH | Surface PM, so $L_d \approx L_q = L_s$ |
| $\psi_m$ (flux linkage) | ~5–10 mWb | High back-EMF constant at low speed |
| $V_{dc}$ | 12 V (3S) | Linear SVPWM limit $\approx 6.93$ V phase peak |
| Driver | DRV8313 / L6234 class (3× PWM + EN) | No complementary gate signals needed |
| Control rate | 10–20 kHz PWM, 5–10 kHz current loop | ESP32-S3 single-precision FPU is adequate |

**Machine model assumptions:** balanced three-phase windings, sinusoidal back-EMF, no magnetic saturation (linear $L$), negligible iron loss, and a star-connected neutral (so $i_a+i_b+i_c=0$).

---

## 1. The Space Vector Concept

Define the complex unit operator $a = e^{j2\pi/3}$. For any three-phase quantity $x \in \{v, i, \psi\}$, the **space vector** is

$$
\vec{x}_s = k\left(x_a + a\,x_b + a^2 x_c\right)
$$

The scaling constant $k$ is a design choice with physical consequences, derived next.

### 1.1 Choosing $k$

Consider balanced sinusoids $x_a = X\cos\theta$, $x_b = X\cos(\theta - 2\pi/3)$, $x_c = X\cos(\theta + 2\pi/3)$.

Using $\cos\phi = \tfrac12(e^{j\phi}+e^{-j\phi})$:

$$
x_a + a x_b + a^2 x_c = \frac{X}{2}\Big[\underbrace{(e^{j\theta}+e^{j\theta}+e^{j\theta})}_{3e^{j\theta}} + \underbrace{(e^{-j\theta} + e^{-j\theta}e^{j4\pi/3} + e^{-j\theta}e^{-j4\pi/3})}_{e^{-j\theta}(1+a^2+a)=0}\Big] = \frac{3X}{2}e^{j\theta}
$$

So $\vec{x}_s = \tfrac{3k}{2}X e^{j\theta}$.

- **Amplitude-invariant:** $k = 2/3$ gives $|\vec{x}_s| = X$. The vector magnitude equals the phase peak. **This is the convention used in this design.**
- **Power-invariant:** $k = \sqrt{2/3}$ makes the transform orthonormal, and $P = \vec v\cdot\vec i$ exactly.

### 1.2 Power Under the Amplitude-Invariant Convention

Instantaneous power is $p = v_a i_a + v_b i_b + v_c i_c$. With $k = 2/3$ and zero-sequence components absent, it can be shown that

$$
p = \tfrac{3}{2}\left(v_\alpha i_\alpha + v_\beta i_\beta\right)
$$

This $\tfrac32$ factor propagates into the torque equation (§2.4). Forgetting it is the most common source of torque-constant errors in FOC firmware.

---

## 2. Clarke and Park Transforms

### 2.1 Clarke Transform (abc → αβ)

Take real and imaginary parts of $\vec x_s = \tfrac23(x_a + a x_b + a^2 x_c)$, using $a = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $a^2 = -\tfrac12 - j\tfrac{\sqrt3}{2}$:

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

The third row (zero-sequence, $x_0$) completes the basis so the matrix is invertible.

**Reduced two-sensor form.** Imposing $i_c = -i_a - i_b$:

$$
i_\alpha = \tfrac23\left(i_a - \tfrac12 i_b - \tfrac12(-i_a - i_b)\right) = i_a
$$

$$
i_\beta = \tfrac23\cdot\tfrac{\sqrt3}{2}\left(i_b - (-i_a - i_b)\right) = \frac{i_a + 2 i_b}{\sqrt3}
$$

This form saves one ADC channel per motor. That matters on the ESP32-S3, where ADC throughput across 3 motors is the binding constraint (§5.3).

**Inverse Clarke** (with $x_0 = 0$):

$$
x_a = x_\alpha,\qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

### 2.2 Park Transform (αβ → dq)

Let the rotor's electrical angle be $\theta_e$, with the d-axis aligned to the PM flux. A rotating reference frame is obtained by multiplying by $e^{-j\theta_e}$:

$$
\vec x_{dq} = x_d + j x_q = \vec x_{\alpha\beta}\,e^{-j\theta_e}
$$

Expanding $(x_\alpha + jx_\beta)(\cos\theta_e - j\sin\theta_e)$:

$$
\begin{bmatrix} x_d \\ x_q\end{bmatrix} =
\begin{bmatrix} \cos\theta_e & \sin\theta_e \\ -\sin\theta_e & \cos\theta_e\end{bmatrix}
\begin{bmatrix} x_\alpha \\ x_\beta\end{bmatrix}
$$

**Inverse Park:**

$$
\begin{bmatrix} x_\alpha \\ x_\beta\end{bmatrix} =
\begin{bmatrix} \cos\theta_e & -\sin\theta_e \\ \sin\theta_e & \cos\theta_e\end{bmatrix}
\begin{bmatrix} x_d \\ x_q\end{bmatrix}
$$

**Verification.** Substituting the balanced currents from §1.1 with $\theta = \theta_e + \gamma$ gives $i_d = I\cos\gamma$ and $i_q = I\sin\gamma$. These are **DC quantities** in steady state, which is why PI controllers can achieve zero steady-state error in this frame.

### 2.3 Derivation of the dq Voltage Equations

Start from the stator voltage law in the stationary frame:

$$
\vec v_s = R_s \vec i_s + \frac{d\vec\psi_s}{dt}
$$

Substitute $\vec\psi_s = \vec\psi_{dq}e^{j\theta_e}$ and $\vec v_s = \vec v_{dq}e^{j\theta_e}$, with $\dot\theta_e = \omega_e$. By the product rule:

$$
\frac{d\vec\psi_s}{dt} = e^{j\theta_e}\left(\frac{d\vec\psi_{dq}}{dt} + j\omega_e\vec\psi_{dq}\right)
$$

Cancel $e^{j\theta_e}$ throughout:

$$
\vec v_{dq} = R_s\vec i_{dq} + \frac{d\vec\psi_{dq}}{dt} + j\omega_e\vec\psi_{dq}
$$

The flux linkages are $\psi_d = L_d i_d + \psi_m$ and $\psi_q = L_q i_q$. Separating real and imaginary parts, and noting $j(\psi_d + j\psi_q) = -\psi_q + j\psi_d$:

$$
\boxed{\;
\begin{aligned}
v_d &= R_s i_d + L_d\frac{di_d}{dt} - \omega_e L_q i_q \\
v_q &= R_s i_q + L_q\frac{di_q}{dt} + \omega_e L_d i_d + \omega_e\psi_m
\end{aligned}\;}
$$

The terms $-\omega_e L_q i_q$ and $\omega_e L_d i_d$ are **cross-coupling** terms. The term $\omega_e\psi_m$ is the **back-EMF**.

### 2.4 Electromagnetic Torque

Electrical power entering the coupling field is $\tfrac32(e_d i_d + e_q i_q)$, where the speed-voltage terms are $e_d = -\omega_e\psi_q$ and $e_q = \omega_e\psi_d$. Dividing by mechanical speed $\omega_m = \omega_e/p$:

$$
T_e = \frac{3}{2}p\left(\psi_d i_q - \psi_q i_d\right) = \frac32 p\left[\psi_m i_q + (L_d - L_q)i_d i_q\right]
$$

For the SPM gimbal motor ($L_d = L_q$), this reduces to

$$
T_e = \tf