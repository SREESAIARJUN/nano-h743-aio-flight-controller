# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting

*(Cycle 17 design iteration. I don't have earlier cycles, so this is a self-contained formulation.)*

---

## 1. Problem Setup

**Per axis (pan ψ, tilt θ), decoupled:**

| Symbol | Meaning |
|---|---|
| $(u, v)$ | Target centroid in image (px) |
| $(c_x, c_y)$ | Principal point |
| $f_x, f_y$ | Focal lengths (px) |
| $\psi, \theta$ | Gimbal joint angles (rad) |
| $\omega_{cmd}$ | Commanded joint rate (rad/s) sent to inner rate loop |
| $T_s$ | Control period (e.g., 5 ms, 200 Hz) |
| $\tau_d$ | Vision pipeline latency (capture → detection) |

**Architecture (cascaded):**

```
 Camera ─► Detector ─► Angular Error ─► Target State Estimator (KF)
                              │                    │
                              ▼                    ▼ ω_ff (feedforward)
                        Outer PID ──────► Σ ──► Rate/Accel Limiter ──► ω_cmd ──► Inner Rate Loop (IMU/encoder) ──► Motor
```

The outer loop runs at the camera rate (30–120 Hz) with interpolation to $T_s$. The inner loop (≥ 500 Hz) is assumed already tuned and has bandwidth roughly 5–10× higher than the outer loop.

---

## 2. Measurement: Pixel → Angular Error

Use exact projection rather than small-angle approximation:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = \arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

**Latency compensation.** The image was captured at $t - \tau_d$. Reconstruct the target's absolute bearing using the gimbal angle *at capture time* (from a timestamped encoder buffer):

$$
\lambda_\psi(t-\tau_d) = \psi(t-\tau_d) + e_\psi
$$

This decouples target motion from gimbal motion. That decoupling is essential for an unbiased feedforward.

---

## 3. Target State Estimation (for Feedforward)

Model the absolute target bearing per axis as constant-velocity with acceleration noise:

$$
\mathbf{x} = \begin{bmatrix}\lambda \\ \dot\lambda\end{bmatrix}, \quad
\mathbf{x}_{k+1} = \underbrace{\begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix}}_{F}\mathbf{x}_k + \mathbf{w}_k,
\quad
Q = q\begin{bmatrix}\Delta t^4/4 & \Delta t^3/2\\ \Delta t^3/2 & \Delta t^2\end{bmatrix}
$$

$$
z_k = \lambda_k + n_k, \quad R = \sigma_{px}^2 / f^2
$$

**Measurement handling:**

- Run predict-only at $T_s$, and update when a detection arrives.
- **Out-of-sequence handling:** rewind to $t-\tau_d$, update, then re-propagate forward by $\tau_d$. Equivalently, update the delayed state and apply $F(\tau_d)$.
- **Gating:** reject if the Mahalanobis distance $d^2 = \tilde z^2 / S > \chi^2_{1,0.997} \approx 9$.

**Outputs at time $t$ (latency-compensated):**

$$
\hat\lambda(t), \quad \hat{\dot\lambda}(t)
$$

For a simpler alternative, an α-β filter with $\alpha \approx 0.5$ and $\beta \approx 0.1$ works well on embedded targets.

---

## 4. Control Law

### 4.1 Tracking error (latency-compensated)

$$
e(t) = \hat\lambda(t) - \psi(t)
$$

### 4.2 Smooth-follow deadband (soft)

This prevents jitter when the target is near center. Use a smooth deadzone, not a hard one:

$$
\tilde e = e \cdot \frac{e^2}{e^2 + \delta^2}
$$

Here $\delta$ is roughly 0.3–1° (a "framing tolerance"). The gain is about 0 inside $\delta$ and about 1 outside, and it is $C^1$-continuous.

### 4.3 PID (outer loop, output = rate)

$$
\omega_{pid} = K_p \tilde e + K_i \int \tilde e \, dt + K_d \frac{d}{dt}\left(-\psi_f\right)
$$

- **Derivative on measurement** (not error) avoids derivative kick on target re-acquisition. $\psi_f$ is $\psi$ filtered by a first-order LPF with $\tau_f = K_d / (N K_p)$, where $N \approx 8$–$20$.
- Because the feedforward carries the target velocity, $K_d$ is often small or zero in the outer loop, since the inner rate loop provides damping.

### 4.4 Velocity feedforward

$$
\omega_{ff} = K_{ff} \cdot \hat{\dot\lambda}(t), \qquad K_{ff} \in [0.8, 1.0]
$$

Optional confidence scaling reduces feedforward when the estimator is uncertain:

$$
K_{ff}^{eff} = K_{ff} \cdot \frac{1}{1 + P_{\dot\lambda\dot\lambda}/\sigma_{v,ref}^2}
$$

### 4.5 Total desired rate

$$
\omega_{des} = \omega_{ff} + \omega_{pid}
$$

---

## 5. Rate, Acceleration, and (optional) Jerk Limiting

### 5.1 Rate saturation

$$
\omega_{sat} = \text{clip}(\omega_{des}, -\omega_{max}, \omega_{max})
$$

### 5.2 Acceleration limiting (slew-rate limiter)

$$
\Delta\omega = \text{clip}\left(\omega_{sat} - \omega_{cmd,k-1},\; -a_{max} T_s,\; a_{max} T_s\right)
$$
$$
\omega_{cmd,k} = \omega_{cmd,k-1} + \Delta\omega
$$

### 5.3 Jerk limiting (cinematic smoothness, optional)

Track a commanded acceleration state $\alpha$:

$$
\alpha_{des} = \text{clip}\!\left(\frac{\omega_{sat}-\omega_{cmd}}{T_s}, -a_{max}, a_{max}\right)
$$
$$
\alpha_k = \alpha_{k-1} + \text{clip}(\alpha_{des} - \alpha_{k-1}, -j_{max}T_s, j_{max}T_s)
$$
$$
\omega_{cmd,k} = \omega_{cmd,k-1} + \alpha_k T_s
$$

To avoid overshoot from the jerk limiter, use a braking check. If $|\omega_{sat} - \omega_{cmd}| \le \alpha^2/(2 j_{max})$, ramp $\alpha$ toward 0 instead.

### 5.4 Anti-windup (back-calculation)

The integrator must know about the limiter:

$$
\dot I = K_i \tilde e + K_{aw}(\omega_{cmd} - \omega_{des}), \qquad K_{aw} \approx 1/T_t,\; T_t = \sqrt{T_i T_d} \text{ or } \approx T_i/2
$$

Additionally, freeze integration when the target is lost or when $|e| > e_{int,max}$ (conditional integration).

---

## 6. Mode Logic (Target Loss and Re-acquisition)

| State | Condition | Behavior |
|---|---|---|
| **TRACK** | Detection valid | Full law above |
| **COAST** | Lost for $< t_{coast}$ (≈0.5–1 s) | KF predict-only; $\omega = K_{ff}\hat{\dot\lambda}\cdot e^{-t/\tau_{decay}}$; integrator frozen |
| **HOLD/SEARCH** | Lost for $> t_{coast}$ | Ramp $\omega \to 0$ via accel limiter; optionally run a search pattern |
| **REACQUIRE** | New detection | Reset KF velocity covariance high; ramp $K_{ff}$ from 0→1 over ~0.3 s; bumpless transfer: $I \leftarrow \omega_{cmd} - \omega_{ff} - K_p\tilde e$ |

---

## 7. Tuning Procedure

1. **Inner rate loop first.** Verify bandwidth $\omega_{bw,in}$ (e.g., 30–60 Hz) with step tests.
2. **Set the physical limits:** $\omega_{max}$ from motor and blur constraints. Set $a_{max}$ to the smaller of the motor torque limit ($\tau_{max}/J$) and the aesthetic limit (≈100–300°/s² for smooth video). Set $j_{max}$ to ≈1000–3000°/s³.
3. **Outer P only, feedforward off.** Raise $K_p$ until the outer crossover is about $\omega_{bw,in}/5$. Typically $K_p \approx 2$–$6$ s⁻¹.
4. **Enable feedforward** with $K_{ff}=1$. Steady ramp-tracking error should approach about 0. If overshoot appears on target direction reversals, reduce $K_{ff}$ to 0.85 or increase KF $R$.
5. **Add $K_i$** to remove residual bias (mount misalignment, cable drag): $T_i \approx 4$–$10 / K_p$.
6. **Tune the estimator $q$.** Higher $q$ gives a more responsive but noisier feedforward. Set it to match the target's expected maneuver acceleration, $q \approx a_{target}^2$.
7. **Set deadband $\delta$** last, to the minimum that eliminates visible hunting.

**Stability check with latency:** keep phase margin ≥ 45° including $e^{-s\tau_d}$:

$$
\angle L(j\omega_c) = -90° - \omega_c \tau_d \cdot \tfrac{180}{\pi} - \angle(\text{inner loop}) > -135°
$$

For $\tau_d = 60$ ms, this implies $\omega_c \lesssim 9$ rad/s. Latency compensation (Section 2) mainly protects the feedforward. Feedback bandwidth is still latency-bounded.

---

## 8. Reference Implementation (Python-like, per axis)

```python
import numpy as np

class AxisTracker:
    def __init__(s, Ts, Kp, Ki, Kd, Kff, w_max, a_max, j_max,
                 deadband, Kaw, N=10, q=4.0, r=(2e-3)**2):
        s.Ts, s.Kp, s.Ki, s.Kd, s.Kff = Ts, Kp, Ki, Kd, Kff
        s.w_max, s.a_max, s.j_max = w_max, a_max, j_max
        s.db, s.Kaw = deadband, Kaw
        s.tf = Kd / (N * Kp) if Kd > 0 else 0.0
        s.I = 0.0; s.psi_f = 0.0; s.psi_f_prev = 0.0
        s.w_cmd = 0.0; s.alpha = 0.0
        # KF
        s.x = np.zeros(2); s.P = np.diag([1e-2, 1.0]); s.q, s.r = q, r

    # ---------- estimator ----------
    def kf_predict(s, dt):
        F = np.array([[1, dt], [0, 1]])
        Q = s.q * np.array([[dt**4/4, dt**3/2], [dt**3/2, dt**2]])
        s.x = F @ s.x; s.P = F @ s.P @ F.T + Q

    def kf_update_delayed(s, z_bearing, tau_d):
        # Rewind approximation: map measurement forward by latency
        F = np.array([[1, tau_d], [0, 1]])
        H = np.array([[1, -tau_d]]) @ np.eye(2)   # z ≈ λ(t) - τ·λ̇(t)
        H = np.array([[1.0, -tau_d]])
        S = H @ s.P @ H.T + s.r
        y = z_bearing - (H @ s.x)[0]
        if y*y / S[0,0] > 9.0:      # gate
            return False
        K = (s.P @ H.T) / S[0,0]
        s.x = s.x + K[:,0]*y
        s.P = (np.eye(2) - K @ H) @ s.P
        return True

    # ---------- controller ----------
    def step(s, psi, tracking=True):
        s.kf_predict(s.Ts)
        lam_hat, lam_dot_hat = s.x

        e = lam_hat - psi
        e_s = e * e*e / (e*e + s.db**2)          # soft deadband

        # derivative on measurement, filtered
        if s.tf > 0:
            a = s.Ts / (s.tf + s.Ts)
            s.psi_f += a * (psi - s.psi_f)
        else:
            s.psi_f = psi
        d_meas = -(s.psi_f - s.psi_f_prev) / s.Ts
        s.psi_f_prev = s.psi_f

        w_ff = s.Kff * lam_dot_hat if tracking else 0.0
        w_des = w_ff + s.Kp*e_s + s.I + s.Kd*d_meas

        # rate limit
        w_sat = np.clip(w_des, -s.w_max, s.w_max)

        # jerk-limited acceleration limit
        dw = w_sat - s.w_cmd
        a_des = np.clip(dw / s.Ts, -s.a_max, s.a_max)
        # braking to avoid overshoot
        if abs(dw) <= s.alpha**2 / (2*s.j_max) and np.sign(dw) == np.sign(s.alpha):
            a_des = 0.0
        s.alpha += np.clip(a_des - s.alpha, -s.j_max*s.Ts, s.j_max*s.Ts)
        s.w_cmd += s.alpha * s.Ts

        # integrator with back-calculation anti-windup
        if tracking:
            s.I += s.Ts * (s.Ki*e_s + s.Kaw*(s.w_cmd - w_des))
        return s.w_cmd
```

**Typical starting values (pan axis, 200 Hz, 60 ms vision latency):**

```
Kp=4.0  Ki=0.6  Kd=0.0  Kff=0.95  Kaw=2.0
w_max=3.0 rad/s  a_max=4.0 rad/s²  j_max=40 rad/s³
deadband=0.008 rad (~0.5°)  q=4.0  σ_meas≈2 mrad
```

---

## 9. Validation Checklist

- **Ramp target (constant angular velocity):** steady-state error is about 0 with feedforward and grows linearly without it. This confirms the feedforward is working.
- **Step target:** no overshoot beyond 5%, the acceleration profile is trapezoidal, and the jerk is bounded.
- **Sinusoidal target:** sweep 0.1–2 Hz and plot the closed-loop Bode response, checking gain flatness and phase lag.
- **Dropout test:** occlude for 0.3 s and 2 s, and verify COAST behavior and bumpless reacquire.
- **Noise injection:** add ±3 px detection jitter and confirm no visible micro-jitter in output video (deadband plus KF).
- **Saturation test:** a target faster than $\omega_{max}$ should produce no integrator windup and no overshoot when the target slows.

---

If you share the gimbal's inertia, motor limits, camera FPS/latency, and target class (person, vehicle, drone), I can compute specific gains and check the stability margins numerically.