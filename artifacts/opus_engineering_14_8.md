# 3-Axis Gimbal on ESP32-S3: FOC Mathematical Core

**Cycle 14 — Clarke/Park Transforms, SVPWM Duty Cycles, and Field Weakening**

---

## 0. Conventions and System Context

| Symbol | Meaning |
|---|---|
| $a,b,c$ | stator phase quantities (star-connected, isolated neutral) |
| $\alpha,\beta$ | stationary orthogonal frame, $\alpha$ aligned with phase $a$ |
| $d,q$ | rotor-synchronous frame, $d$ aligned with the PM flux |
| $\theta_e = p\,\theta_m$ | electrical angle ($p$ = pole pairs) |
| $\omega_e = \dot\theta_e$ | electrical angular velocity |
| $\psi_m$ | PM flux linkage (peak, per phase) |
| $V_{dc}$ | DC bus voltage |

**Transform convention.** Unless stated otherwise, this document uses the **amplitude-invariant** transform. A phase current of peak $I$ maps to $|\vec i_{dq}| = I$. Firmware and datasheet values then share one scale.

**Hardware constraint.** The ESP32-S3 has two MCPWM units. Each unit has 3 operators, and each operator has 2 outputs, for **12 PWM outputs** in total.

- Three axes need **9 phase PWMs**.
- Complementary gate-driving (6 outputs per motor) would exhaust the MCPWM after 2 motors.
- **Architecture decision:** use 3-PWM integrated drivers (e.g., DRV8313/TMC6300-class) with internal dead time. Each operator drives two phase legs.
- Timer sync between MCPWM0 and MCPWM1 uses the sync-in/sync-out mechanism, so all 9 legs are center-aligned on a common carrier.

---

## 1. The Clarke Transform

### 1.1 Geometric Derivation

Model the three windings as spatially displaced by $0, \tfrac{2\pi}{3}, \tfrac{4\pi}{3}$. The stator produces a **space vector**:

$$
\vec{x}_s = k\left(x_a + x_b\,e^{j2\pi/3} + x_c\,e^{j4\pi/3}\right) = x_\alpha + j x_\beta
$$

Use $e^{j2\pi/3} = -\tfrac12 + j\tfrac{\sqrt3}{2}$ and $e^{j4\pi/3} = -\tfrac12 - j\tfrac{\sqrt3}{2}$, then separate real and imaginary parts:

$$
\begin{bmatrix} x_\alpha \\ x_\beta \end{bmatrix}
= k\begin{bmatrix} 1 & -\tfrac12 & -\tfrac12 \\[2pt] 0 & \tfrac{\sqrt3}{2} & -\tfrac{\sqrt3}{2} \end{bmatrix}
\begin{bmatrix} x_a \\ x_b \\ x_c\end{bmatrix}
$$

### 1.2 Determining the Scale Factor $k$

**Amplitude invariance.** Take a balanced set $x_a = X\cos\phi$, $x_b = X\cos(\phi - \tfrac{2\pi}{3})$, $x_c = X\cos(\phi+\tfrac{2\pi}{3})$. Write each cosine with Euler's formula:

$$
\sum_{n=0}^{2} X\cos\!\left(\phi - \tfrac{2\pi n}{3}\right)e^{j2\pi n/3}
= \frac{X}{2}\sum_n \left(e^{j\phi} + e^{-j\phi}e^{j4\pi n/3}\right)
= \frac{3X}{2}e^{j\phi}
$$

The second sum vanishes because $\sum_n e^{j4\pi n/3} = 0$. So $|\vec x_s| = \tfrac{3}{2}kX$.

> Requiring $|\vec x_s| = X$ gives $k = \tfrac{2}{3}$.

**Power invariance.** Requiring $T^\top T = I$ on the full 3×3 transform (including zero sequence) gives $k = \sqrt{2/3}$. Then $p = v_a i_a + v_b i_b + v_c i_c = v_\alpha i_\alpha + v_\beta i_\beta$ with no extra factor.

**Consequence under amplitude invariance.** Power picks up a factor of 3/2:

$$
p = \tfrac{3}{2}\left(v_\alpha i_\alpha + v_\beta i_\beta\right)
$$

### 1.3 Zero Sequence and the Reduced Form

The full transform appends a zero-sequence row: $x_0 = \tfrac13(x_a+x_b+x_c)$.

For a star-connected motor with isolated neutral, $i_a+i_b+i_c = 0$, so $i_0 \equiv 0$. Substitute $i_c = -i_a - i_b$:

$$
\boxed{\,i_\alpha = i_a,\qquad i_\beta = \frac{1}{\sqrt3}\left(i_a + 2 i_b\right)\,}
$$

Only **two current sensors** are required (two ADC channels per axis). That is 6 channels for 3 axes, well within the S3's two SAR ADCs.

Note that the zero-sequence *voltage* is **not** zero in general. Section 3.5 exploits this degree of freedom.

### 1.4 Inverse Clarke (Amplitude-Invariant, Zero Sequence = 0)

$$
x_a = x_\alpha,\qquad
x_b = -\tfrac12 x_\alpha + \tfrac{\sqrt3}{2}x_\beta,\qquad
x_c = -\tfrac12 x_\alpha - \tfrac{\sqrt3}{2}x_\beta
$$

---

## 2. The Park Transform and dq Machine Model

### 2.1 Rotation

Express the space vector in a frame rotating at $\theta_e$:

$$
\vec x_{dq} = \vec x_{\alpha\beta}\,e^{-j\theta_e}
\;\Longrightarrow\;
\begin{bmatrix} x_d \\ x_q \end{bmatrix}
= \underbrace{\begin{bmatrix} \cos\theta_e & \sin\theta_e \\ -\sin\theta_e & \cos\theta_e\end{bmatrix}}_{R(-\theta_e)}
\begin{bmatrix} x_\alpha \\ x_\beta\end{bmatrix}
$$

The inverse uses $R(\theta_e) = R(-\theta_e)^\top$:

$$
x_\alpha = x_d\cos\theta_e - x_q\sin\theta_e,\qquad
x_\beta = x_d\sin\theta_e + x_q\cos\theta_e
$$

### 2.2 Derivation of the dq Voltage Equations (the Origin of Cross-Coupling)

Stationary-frame stator equation:

$$
\vec v_{\alpha\beta} = R_s\vec i_{\alpha\beta} + \frac{d\vec\psi_{\alpha\beta}}{dt}
$$

Substitute $\vec\psi_{\alpha\beta} = \vec\psi_{dq}e^{j\theta_e}$ and apply the product rule:

$$
\frac{d}{dt}\left(\vec\psi_{dq}e^{j\theta_e}\right) = \left(\dot{\vec\psi}_{dq} + j\omega_e\vec\psi_{dq}\right)e^{j\theta_e}
$$

Multiply through by $e^{-j\theta_e}$:

$$
\vec v_{dq} = R_s\vec i_{dq} + \dot{\vec\psi}_{dq} + j\omega_e\vec\psi_{dq}
$$

The flux linkages are $\psi_d = L_d i_d + \psi_m$ and $\psi_q = L_q i_q$. Since $j(\psi_d + j\psi_q) = -\psi_q + j\psi_d$:

$$
\boxed{
\begin{aligned}
v_d &= R_s i_d + L_d\frac{di_d}{dt} - \omega_e L_q i_q\\
v_q &= R_s i_q + L_q\frac{di_q}{dt} + \omega_e L_d i_d + \omega_e\psi_m
\end{aligned}}
$$

The term $j\omega_e\vec\psi_{dq}$ (the derivative of the rotation operator) produces the speed-dependent cross-coupling and the back-EMF $\omega_e\psi_m$. Feed-forward decoupling cancels these terms:

$$
v_d^{*} = \mathrm{PI}_d(i_d^*-i_d) - \omega_e L_q i_q,\qquad
v_q^{*} = \mathrm{PI}_q(i_q^*-i_q) + \omega_e(L_d i_d + \psi_m)
$$

### 2.3 Torque via Power Balance

The electrical input power in dq (amplitude-invariant) is $p = \tfrac32(v_d i_d + v_q i_q)$. Substitute the voltage equations and group terms:

- Copper loss: $\tfrac32 R_s(i_d^2+i_q^2)$
- Change in stored magnetic energy: the $\dot\psi$ terms
- Electromechanical power: $\tfrac32\omega_e(\psi_d i_q - \psi_q i_d)$

The air-gap power equals $T_e\,\omega_m$ with $\omega_m = \omega_e/p$, so:

$$
\boxed{T_e = \tfrac32\,p\left[\psi_m i_q + (L_d - L_q)\,i_d i_q\right]}
$$

**Gimbal motors are surface-mounted PM** ($L_d \approx L_q$). The reluctance term vanishes, leaving $T_e = \tfrac32 p\psi_m i_q$. This is linear in $i_q$, which is ideal for a stabilization loop.

### 2.4 Discretization Note

The PWM update applies $v^*$ one period late. Compensate the angle for rotation during the computational delay:

$$
\theta_{\text{applied}} = \theta_{\text{meas}} + 1.5\,\omega_e T_s
$$

The 1.5 accounts for 1 sample of computation delay plus ½ sample of the PWM zero-order-hold average. At gimbal speeds the correction is small but nonzero during fast re-centering slews.

---

## 3. Space Vector Modulation

### 3.1 Inverter Voltage Vectors

Each leg $x \in \{a,b,c\}$ has switch state $S_x \in \{0,1\}$ and pole voltage $S_x V_{dc}$ relative to the negative rail. Apply the amplitude-invariant Clarke transform to the pole voltages. The common-mode part drops out automatically because $1 + e^{j2\pi/3} + e^{j4\pi/3} = 0$:

$$
\vec V(S_a,S_b,S_c) = \tfrac{2}{3}V_{dc}\left(S_a + S_b e^{j2\pi/3} + S_c e^{j4\pi/3}\right)
$$

This yields:

- **Six active vectors** $\vec V_1\ldots\vec V_6$ of