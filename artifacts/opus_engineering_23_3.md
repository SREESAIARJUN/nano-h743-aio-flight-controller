# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting (Cycle 23)

## 1. Problem Setup

**States (per axis, pan ψ / tilt θ):**
- Gimbal angle $q = [\psi, \theta]^T$ and rate $\dot q$
- Target pixel position $(u, v)$, image center $(c_x, c_y)$, focal lengths $(f_x, f_y)$
- Control output: commanded gimbal rate $\omega_{cmd}$ (rate-loop inner controller assumed)

**Objective:** drive the target's line-of-sight (LOS) to the optical axis with smooth motion. That means no jerky starts or stops, bounded acceleration, and minimal lag on moving targets.

---

## 2. Error Formulation (Image → Angular Error)

Convert pixel offset to angular error, which is linear in angle and independent of distance:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = \arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

**Deadband** (suppresses hunting on detector jitter), with a smooth transition:

$$
\tilde e = \begin{cases} 0 & |e| < \delta \\ \operatorname{sign}(e)\,(|e| - \delta) & \text{otherwise} \end{cases}
$$

Typical values: δ ≈ 0.1–0.3° or 2–5 px equivalent.

---

## 3. Target Velocity Estimation (Feedforward Source)

The target's inertial LOS angle is the gimbal angle plus the image error:

$$
\lambda_k = q_k + e_k
$$

Estimate LOS rate $\hat{\dot\lambda}$ with a **constant-velocity Kalman filter** (preferred) or an α-β filter.

**State:** $x = [\lambda, \dot\lambda]^T$

$$
x_{k+1} = \begin{bmatrix}1 & T\\0 & 1\end{bmatrix}x_k + w_k, \qquad
z_k = \lambda_k + n_k
$$

**α-β alternative** (lightweight):

$$
\begin{aligned}
\lambda_p &= \hat\lambda_{k-1} + T\,\hat{\dot\lambda}_{k-1} \\
r &= z_k - \lambda_p \\
\hat\lambda_k &= \lambda_p + \alpha r \\
\hat{\dot\lambda}_k &= \hat{\dot\lambda}_{k-1} + \tfrac{\beta}{T} r
\end{aligned}
$$

Typical values are α ≈ 0.5, β ≈ 0.1. Lower values give smoother estimates with more lag.

**Latency compensation:** the vision pipeline has delay $\tau_v$ (capture + inference). Propagate the estimate forward:

$$
\hat\lambda_{now} = \hat\lambda_k + \tau_v\,\hat{\dot\lambda}_k
$$

---

## 4. Control Law

$$
\boxed{\;\omega_{raw} = \underbrace{K_{ff}\,\hat{\dot\lambda}}_{\text{feedforward}} + \underbrace{K_p\,\tilde e + K_i\!\int\!\tilde e\,dt + K_d\,\dot e_f}_{\text{PID feedback}}\;}
$$

### 4.1 Feedforward
- $K_{ff} \in [0.8, 1.0]$. Unity means full target-motion cancellation, so the PID only corrects residuals.
- **Confidence gating:** scale feedforward by detection confidence $c \in [0,1]$ and by filter covariance health:
  $$K_{ff,eff} = K_{ff}\cdot c\cdot \mathbb{1}[P_{\dot\lambda\dot\lambda} < P_{max}]$$

### 4.2 Derivative on Measurement with Filtering
Differentiate the error with a first-order low-pass filter to avoid amplifying pixel noise:

$$
\dot e_{f,k} = \frac{\tau_d}{\tau_d + T}\dot e_{f,k-1} + \frac{1}{\tau_d + T}(e_k - e_{k-1})
$$

Use $\tau_d \approx (2\text{–}5)\,T_{vision}$. With feedforward active, $K_d$ can usually be small.

### 4.3 Integral with Anti-Windup
- **Conditional integration:** freeze the integrator when the output is saturated *and* the error has the same sign as the saturation.
- **Back-calculation alternative:**
  $$I_{k} = I_{k-1} + T\left[K_i\tilde e_k + \tfrac{1}{T_t}(\omega_{cmd,k-1} - \omega_{raw,k-1})\right]$$
- **Clamp:** $|I| \le I_{max}$
- **Reset or bleed** $I$ on target loss.

---

## 5. Rate and Acceleration Limiting (Smoothness Layer)

### 5.1 Rate Saturation
$$
\omega_{sat} = \operatorname{clip}(\omega_{raw},\, -\omega_{max},\, \omega_{max})
$$

### 5.2 Acceleration (Slew) Limiting
$$
\omega_{cmd,k} = \omega_{cmd,k-1} + \operatorname{clip}\!\big(\omega_{sat} - \omega_{cmd,k-1},\; -a_{max}T,\; a_{max}T\big)
$$

### 5.3 Optional Jerk Limiting (S-curve, Cinematic Smoothness)
Maintain the commanded acceleration $a_k$:

$$
a_{des} = \frac{\omega_{sat} - \omega_{cmd,k-1}}{T}, \quad
a_k = a_{k-1} + \operatorname{clip}(a_{des} - a_{k-1},\, -j_{max}T,\, j_{max}T), \quad
a_k \leftarrow \operatorname{clip}(a_k, -a_{max}, a_{max})
$$

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + a_k T
$$

### 5.4 Stopping-Distance-Aware Braking (Prevents Overshoot under Accel Limit)
Cap the approach speed so the gimbal can decelerate within the remaining error:

$$
|\omega_{fb}| \le \sqrt{2\,a_{max}\,|\tilde e|}
$$

Apply this cap to the feedback portion before adding feedforward. It turns the response into a time-optimal-like profile without overshoot.

---

## 6. Mechanical and Mode Constraints

| Condition | Action |
|---|---|
| Near pan/tilt hard limit | Scale $\omega_{cmd}\to0$ linearly within margin $\Delta q$ |
| Tilt near ±90° (gimbal lock) | Reduce pan gain by $\cos\theta$ compensation, or switch to a yaw-follow mode |
| Target lost < $T_{coast}$ | Coast on $\hat{\dot\lambda}$ (feedforward only, decaying: $\omega = \hat{\dot\lambda}e^{-t/\tau_c}$) |
| Target lost > $T_{coast}$ | Decelerate at $a_{max}$ to zero, reset $I$, enter search/hold |
| Re-acquisition | Re-init filter with zero velocity; ramp $K_{ff}$ in over ~0.3 s |

**Base-motion compensation (vehicle/drone mount):** subtract the body rate measured by the IMU:

$$
\omega_{cmd}^{gimbal} = \omega_{cmd}^{inertial} - R_{b\to g}\,\omega_{body}
$$

---

## 7. Discrete Implementation (per axis)

```python
class AxisTracker:
    def __init__(s, Kp, Ki, Kd, Kff, w_max, a_max, j_max, I_max,
                 deadband, tau_d, alpha, beta, latency):
        s.__dict__.update(locals()); del s.__dict__['s']
        s.I = s.e_prev = s.de_f = 0.0
        s.lam = s.lam_dot = 0.0
        s.w_cmd = s.a_cmd = 0.0
        s.init = False

    def update(s, e_pix_ang, q, conf, dt, valid=True):
        if not valid:                                   # coast / brake
            target = s.lam_dot * conf
            return s._limit(target, dt)

        # --- LOS estimation (alpha-beta) ---
        z = q + e_pix_ang
        if not s.init:
            s.lam, s.lam_dot, s.init = z, 0.0, True
        lam_p = s.lam + s.lam_dot * dt
        r = z - lam_p
        s.lam = lam_p + s.alpha * r
        s.lam_dot += (s.beta / dt) * r
        ff = s.Kff * conf * s.lam_dot

        # --- error w/ deadband + latency compensation ---
        e = e_pix_ang + s.latency * (s.lam_dot - s.w_cmd)
        e = 0.0 if abs(e) < s.deadband else e - copysign(s.deadband, e)

        # --- filtered derivative ---
        s.de_f = (s.tau_d*s.de_f + (e - s.e_prev)) / (s.tau_d + dt)
        s.e_prev = e

        # --- PID feedback with braking cap ---
        fb = s.Kp*e + s.I + s.Kd*s.de_f
        v_brake = sqrt(2*s.a_max*abs(e))
        fb = clip(fb, -v_brake - abs(ff)*0, v_brake) if e else fb

        w_raw = ff + fb
        w_out = s._limit(w_raw, dt)

        # --- conditional integration (anti-windup) ---
        saturated = abs(w_raw) > s.w_max
        if not (saturated and sign(e) == sign(w_raw)):
            s.I = clip(s.I + s.Ki*e*dt, -s.I_max, s.I_max)
        return w_out

    def _limit(s, w_des, dt):
        w_des = clip(w_des, -s.w_max, s.w_max)
        a_des = (w_des - s.w_cmd) / dt
        s.a_cmd += clip(a_des - s.a_cmd, -s.j_max*dt, s.j_max*dt)
        s.a_cmd = clip(s.a_cmd, -s.a_max, s.a_max)
        s.w_cmd += s.a_cmd * dt
        return s.w_cmd
```

Run the controller at the **control rate** (e.g., 200 Hz). When there is no new vision frame, propagate the estimator forward and hold $e$ via prediction: $e \leftarrow e - (\omega_{cmd} - \hat{\dot\lambda})\,dt$.

---

## 8. Tuning Procedure

1. **Inner rate loop first.** Verify that the gimbal motor tracks $\omega_{cmd}$ with bandwidth at least 5× the outer loop.
2. **Set the limits from the mechanics and the desired look:**
   - $\omega_{max}$: 60–180 °/s
   - $a_{max}$: 100–400 °/s² (lower is more cinematic)
   - $j_{max}$: 1000–3000 °/s³
3. **P only** ($K_{ff}=K_i=K_d=0$). Raise $K_p$ until there is slight overshoot on a static target step, then back off 30%. Constraint: $K_p < \frac{1}{2(\tau_v + T)}$ for delay stability.
4. **Add feedforward.** Use a constant-speed moving target. Increase $K_{ff}$ toward 1 until steady lag is about 0. Tune α/β for a smooth $\hat{\dot\lambda}$.
5. **Add $K_i$** small, at about $K_p/(10\,\tau_{settle})$, to remove residual bias such as cable drag or imbalance.
6. **Add $K_d$** only if overshoot persists. Increase $\tau_d$ if you see jitter.
7. **Validate edge cases:**
   - Occlusion and re-acquisition
   - Target reversal
   - Near-limit behavior
   - Base vibration

---

## 9. Performance Metrics (Cycle 23 Acceptance)

| Metric | Target |
|---|---|
| Static centering error (RMS) | < 0.2° |
| Constant-velocity lag (30 °/s target) | < 0.5° |
| Step settling (10°) | < 0.6 s, overshoot < 5% |
| Peak commanded accel | ≤ $a_{max}$ (by construction) |
| Jitter (rate std at rest) | < 0.5 °/s |
| Re-acquisition transient | No rate discontinuity > $a_{max}T$ |

---

## 10. Summary Architecture

```
Camera → Detector → pixel err → angular err ─┬─► deadband ─► PID ─► braking cap ─┐
                                              │                                   ├─► Σ ─► rate clip ─► accel/jerk limit ─► − body rate ─► Gimbal rate loop
Gimbal encoder q ───► LOS = q + e ─► α-β/KF ─► λ̇̂ ─► ×Kff·conf ──────────────────┘
```

This architecture splits the work between two parts:
- **Feedforward** carries the target motion, giving low lag.
- **PID** cleans up the residuals.

The acceleration, jerk, and braking layers guarantee smooth, overshoot-free motion regardless of how aggressive the gains or target maneuvers are.