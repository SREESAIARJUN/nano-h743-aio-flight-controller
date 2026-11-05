# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 22)

This controller does three things:

- **Keeps the target centred** in the image.
- **Predicts target motion**, so it does not lag behind a moving subject.
- **Moves smoothly**, with bounded jerk-free acceleration, so the footage looks cinematic.

Pan (yaw) and tilt (pitch) are treated as two decoupled axes with identical structure. The derivation below is for one axis, angle θ.

---

## 1. Measurement Model

The vision tracker returns a target pixel centroid (u, v). The image centre is (c_x, c_y), and the focal lengths are f_x, f_y in pixels.

**Angular error (exact pinhole form):**

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\phi = \arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

**Absolute target bearing in the gimbal base frame:**

$$
\theta_t = \theta_g + e
$$

Here θ_g is the measured gimbal joint angle from the encoder or IMU. Working with the absolute bearing decouples target motion from gimbal motion. This is essential for a clean feedforward signal.

**Latency compensation.** Vision measurements arrive τ_v seconds late (exposure, inference and transport). Use the gimbal angle at capture time, θ_g(t − τ_v), taken from a short ring buffer:

$$
\theta_t(t-\tau_v) = \theta_g(t-\tau_v) + e(t-\tau_v)
$$

---

## 2. Target State Estimator (Feedforward Source)

Raw differentiation of θ_t is too noisy. Use a constant-velocity Kalman filter per axis.

**State:** $x = [\theta_t,\ \omega_t]^T$

**Model:**

$$
x_{k+1} = \begin{bmatrix}1 & T\\ 0 & 1\end{bmatrix}x_k + w_k,\quad
Q = q\begin{bmatrix}T^3/3 & T^2/2\\ T^2/2 & T\end{bmatrix},\quad
z_k = [1\ \ 0]\,x_k + v_k
$$

**Prediction to the present.** After the delayed update, predict forward by τ_v to obtain the current target estimate:

$$
\hat\theta_t(t) = \hat\theta_t + \hat\omega_t\,\tau_v,\qquad \hat\omega_t(t)=\hat\omega_t
$$

**Choosing q:**

- Small q gives a smooth feedforward but slow reaction to manoeuvres.
- Large q gives a responsive but jittery feedforward.

**Alternative.** A simpler α-β filter works if compute is tight:

$$
\hat\theta \leftarrow \hat\theta + \alpha r,\qquad \hat\omega \leftarrow \hat\omega + \frac{\beta}{T} r,\qquad r = z - \hat\theta_{pred}
$$

---

## 3. Smooth Follow Reference (Dead-zone + Soft Shaping)

For a "visual follow" look, the gimbal should not chase every pixel of jitter.

**Dead-zone with smooth re-entry:**

$$
\tilde e = \begin{cases}
0, & |e| \le e_{dz}\\[2pt]
\operatorname{sign}(e)\,\dfrac{(|e|-e_{dz})^2}{2\,e_{s}}, & e_{dz}<|e|\le e_{dz}+e_s\\[6pt]
\operatorname{sign}(e)\left(|e|-e_{dz}-\tfrac{e_s}{2}\right), & |e|>e_{dz}+e_s
\end{cases}
$$

This shaping is C¹-continuous, so it introduces no kink and no twitch.

**Optional framing offset.** For rule-of-thirds composition, track e − e_frame instead of e.

---

## 4. Control Law (Velocity Command Output)

Most gimbals accept rate commands, so the controller outputs a desired gimbal angular rate:

$$
\boxed{\;\omega_{cmd} = \underbrace{K_{ff}\,\hat\omega_t}_{\text{feedforward}} + \underbrace{K_p\,\tilde e + K_i\!\int\!\tilde e\,dt + K_d\,\dot{\tilde e}_f}_{\text{PID on residual}}\;}
$$

- **Feedforward term.** K_ff ∈ [0.8, 1.0] cancels the target's motion. The PID then only corrects residual error, so gains can stay low, which keeps motion smooth.
- **Derivative on filtered error:**

$$
\dot{\tilde e}_f[k] = \frac{2K_d... }{}\quad\Rightarrow\quad
D[k] = \frac{\tau_d}{\tau_d+T}D[k-1] + \frac{K_d}{\tau_d+T}\big(\tilde e[k]-\tilde e[k-1]\big)
$$

  Use τ_d ≈ (2–5)·T_vision.

- **Feedforward fade-in on low confidence.** Scale the feedforward by tracker confidence:

$$
K_{ff}^{eff} = K_{ff}\cdot \operatorname{sat}_{[0,1]}\!\left(\frac{c - c_{min}}{c_{full}-c_{min}}\right)
$$

---

## 5. Velocity & Acceleration Limiting (Smoothness Core)

**Step 1 – Velocity saturation:**

$$
\omega_{sat} = \operatorname{clip}(\omega_{cmd},\,-\omega_{max},\,\omega_{max})
$$

**Step 2 – Acceleration (slew-rate) limit:**

$$
\Delta\omega_{max} = a_{max}\,T,\qquad
\omega_{out}[k] = \omega_{out}[k-1] + \operatorname{clip}\!\big(\omega_{sat}-\omega_{out}[k-1],\,-\Delta\omega_{max},\,\Delta\omega_{max}\big)
$$

**Step 3 (recommended) – Jerk limit** for cinematic quality:

$$
\alpha_{des} = \frac{\omega_{sat}-\omega_{out}[k-1]}{T},\quad
\alpha[k] = \alpha[k-1] + \operatorname{clip}\!\big(\alpha_{des}-\alpha[k-1],\,\pm j_{max}T\big),
$$
$$
\alpha[k] = \operatorname{clip}(\alpha[k],\pm a_{max}),\qquad \omega_{out}[k] = \omega_{out}[k-1] + \alpha[k]\,T
$$

**Overshoot prevention near the target.** A slew-limited system can overshoot because it cannot brake fast enough. Cap the commanded rate using the stopping-distance constraint:

$$
|\omega_{rel}| \le \sqrt{2\,a_{max}\,|e|}
\quad\Rightarrow\quad
\omega_{cmd} \leftarrow \hat\omega_t + \operatorname{clip}\!\big(\omega_{cmd}-\hat\omega_t,\ \pm\sqrt{2a_{max}|\tilde e|}\big)
$$

---

## 6. Anti-Windup (Back-Calculation)

Whenever saturation or slew limiting is active, bleed the integrator:

$$
I[k] = I[k-1] + K_i\,\tilde e[k]\,T + \frac{T}{T_t}\big(\omega_{out}[k] - \omega_{cmd}[k]\big),\qquad T_t \approx \sqrt{T_i T_d}
$$

Also apply:

- A hard clamp |I| ≤ I_max.
- **Freeze the integrator** (do not reset it) during target loss.

---

## 7. Target Loss Behaviour

| Time since loss | Action |
|---|---|
| 0 – t_coast (≈0.5 s) | Coast: ω_cmd = K_ff·ω̂_t, KF predict-only, PID off |
| t_coast – t_decay | Decay feedforward exponentially toward 0 (τ ≈ 0.5 s) |
| > t_decay | Hold position or return to home via the same accel-limited profile |
| Reacquire | Re-init the KF with the new bearing and blend the PID in over ~0.3 s (bumpless) |

---

## 8. Discrete Implementation (Python, one axis)

```python
import math

class AxisTracker:
    def __init__(self, Kp, Ki, Kd, Kff, w_max, a_max, j_max,
                 tau_d, Tt, I_max, e_dz, e_s, q, r):
        self.Kp, self.Ki, self.Kd, self.Kff = Kp, Ki, Kd, Kff
        self.w_max, self.a_max, self.j_max = w_max, a_max, j_max
        self.tau_d, self.Tt, self.I_max = tau_d, Tt, I_max
        self.e_dz, self.e_s = e_dz, e_s
        # KF state
        self.x = [0.0, 0.0]; self.P = [[1, 0], [0, 1]]
        self.q, self.r = q, r
        # controller state
        self.I = self.D = self.e_prev = 0.0
        self.w_out = self.alpha = 0.0

    def shape(self, e):
        a = abs(e); s = math.copysign(1, e)
        if a <= self.e_dz: return 0.0
        if a <= self.e_dz + self.e_s:
            return s * (a - self.e_dz)**2 / (2 * self.e_s)
        return s * (a - self.e_dz - self.e_s / 2)

    def kf_step(self, z, T):
        th, w = self.x
        th += w * T                                   # predict
        P = self.P; q = self.q
        P00 = P[0][0] + T*(P[1][0]+P[0][1]) + T*T*P[1][1] + q*T**3/3
        P01 = P[0][1] + T*P[1][1] + q*T**2/2
        P11 = P[1][1] + q*T
        if z is not None:                             # update
            S = P00 + self.r
            k0, k1 = P00/S, P01/S
            y = z - th
            th += k0*y; w += k1*y
            P00, P01, P11 = (1-k0)*P00, (1-k0)*P01, P11 - k1*P01
        self.x = [th, w]; self.P = [[P00, P01], [P01, P11]]

    def step(self, e_meas, theta_g_at_capture, latency, conf, T):
        valid = e_meas is not None
        z = theta_g_at_capture + e_meas if valid else None
        self.kf_step(z, T)
        w_t = self.x[1]

        e = self.shape(e_meas) if valid else 0.0
        ff_gain = self.Kff * min(max((conf - 0.3) / 0.4, 0), 1)

        # PID (filtered derivative)
        self.D = (self.tau_d*self.D + self.Kd*(e - self.e_prev)) / (self.tau_d + T)
        self.e_prev = e
        w_cmd = ff_gain*w_t + self.Kp*e + self.I + (self.D if valid else 0)

        # stopping-distance cap on relative rate
        cap = math.sqrt(2*self.a_max*abs(e)) + 1e-6
        w_cmd = ff_gain*w_t + max(-cap, min(cap, w_cmd - ff_gain*w_t))

        # velocity -> jerk -> accel limiting
        w_sat = max(-self.w_max, min(self.w_max, w_cmd))
        a_des = (w_sat - self.w_out) / T
        da = max(-self.j_max*T, min(self.j_max*T, a_des - self.alpha))
        self.alpha = max(-self.a_max, min(self.a_max, self.alpha + da))
        self.w_out += self.alpha * T

        # back-calculation anti-windup (integrator frozen when target lost)
        if valid:
            self.I += self.Ki*e*T + (T/self.Tt)*(self.w_out - w_cmd)
            self.I = max(-self.I_max, min(self.I_max, self.I))
        return self.w_out
```

---

## 9. Tuning Procedure

1. **Characterise the plant.** Measure the gimbal's rate-loop bandwidth ω_bw and the vision latency τ_v. Keep the outer loop crossover below ≈ min(ω_bw/5, 1/(4τ_v)).
2. **Set the limits first.** Pick ω_max and a_max from the aesthetic target (typical film use: a_max ≈ 60–120 °/s², j_max ≈ 300–600 °/s³), then confirm they are within motor capability.
3. **Feedforward.** With PID off, track a constant-speed target. Raise K_ff toward 1 until steady drift is minimal, then back off about 5–10% for robustness.
4. **K_p.** Increase until residual error settles in about 0.5–1 s with no visible oscillation. A starting point is K_p ≈ 1/(3τ_v + T_vision).
5. **K_d.** Add a small amount only if overshoot appears; this is often zero with good feedforward.
6. **K_i.** Use a small value, T_i ≈ 4–10 / K_p, just to remove framing bias (for example from a mount offset).
7. **Kalman q.** Tune on real footage: start low, and raise it until manoeuvre lag becomes acceptable.
8. **Dead-zone.** Set e_dz to about 1–2% of FOV and e_s to about 2–3% of FOV.

---

## 10. Validation Metrics (for this cycle)

| Metric | Target |
|---|---|
| RMS framing error (constant-speed target) | < 2% FOV |
| Peak error during 90° target direction change | < 10% FOV |
| Commanded jerk | ≤ j_max (verify from logs) |
| Overshoot on step reacquire | < 5% |
| Loss→reacquire transient | Bumpless (no ω discontinuity) |
| Steady jitter on static target | < 0.05°/s RMS |

---

## Summary of Changes to Carry Into Cycle 22

- Latency-compensated **absolute-bearing Kalman filter** as the feedforward source.
- **C¹ dead-zone shaping** for smooth follow.
- A layered **velocity → jerk → acceleration limiter**, plus a **stopping-distance cap** that prevents overshoot.
- **Back-calculation anti-windup**, plus a **confidence-weighted feedforward and coast/decay** policy for target loss.

If you share the previous cycle's gains, plant parameters, or log traces, I can propose specific numeric adjustments for this iteration.