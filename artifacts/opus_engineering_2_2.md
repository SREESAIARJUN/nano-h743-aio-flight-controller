# Autonomous Gimbal Visual-Follow Controller: Cycle 2 Formulation

**PID + Velocity Feedforward + Acceleration/Jerk Limiting**

Cycle 1 is taken to be a basic PID on pixel error. Cycle 2 addresses the problems that design typically shows:

- **Lag on moving targets.** Fixed by velocity feedforward from target motion estimation.
- **Jerky starts and stops.** Fixed by acceleration and jerk limiting.
- **Overshoot and windup at saturation.** Fixed by conditional integration with back-calculation.
- **Jitter from detector noise.** Fixed by an adaptive deadband, filtered derivative, and α-β estimation.
- **Base-motion disturbance.** Fixed by IMU rate compensation.

---

## 1. Problem Setup

Each axis (yaw ψ, pitch θ) is handled independently with the same structure. Roll is stabilized separately to the horizon.

**Measurement.** The detector gives a target centroid (u, v) in pixels, with image center (c_x, c_y) and focal length f_x, f_y. Convert this to angular error (exact, not small-angle):

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = \arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

**Control output.** The output is a gimbal angular **rate** command ω_cmd, sent to the inner motor rate loop. The inner loop is assumed to run at least 5× faster, with bandwidth ω_bw.

---

## 2. Target Motion Estimation (Feedforward Source)

The target's inertial angular position is reconstructed as:

$$
\lambda_k = \phi_k + e_k
$$

where φ_k is the gimbal joint angle (from the encoder) plus the base attitude (from the IMU).

An **α-β filter** estimates the target angle and angular velocity:

$$
\begin{aligned}
\hat\lambda^-_k &= \hat\lambda_{k-1} + T\,\hat{\dot\lambda}_{k-1} \\
r_k &= \lambda_k - \hat\lambda^-_k \\
\hat\lambda_k &= \hat\lambda^-_k + \alpha\, r_k \\
\hat{\dot\lambda}_k &= \hat{\dot\lambda}_{k-1} + \frac{\beta}{T}\, r_k
\end{aligned}
$$

Typical gains are α ≈ 0.3–0.5. For critical damping, choose β = α²/(2 − α).

**Latency compensation.** Vision latency τ_v (typically 30–100 ms) is compensated by predicting forward:

$$
e^{pred}_k = e_k + \hat{\dot\lambda}_k\,\tau_v - \omega_{gimbal}\,\tau_v
$$

**Target loss.** If the target is lost, coast on the estimate for up to t_coast (about 0.5 s). After that, decay the feedforward to zero.

---

## 3. Control Law

### 3.1 Feedback Terms

**Proportional term with soft deadband.** The deadband suppresses jitter without creating a discontinuity:

$$
\tilde e = \begin{cases} 0 & |e| < d \\ e - d\,\text{sgn}(e) & \text{otherwise}\end{cases}
\qquad
P_k = K_p\,\tilde e_k
$$

Make d adaptive, for example d = d_0 + k_σ·σ_detector, so it grows with detection noise.

**Integral term with conditional integration.** Accumulate only when the output is not saturated, or when the error would drive it out of saturation. Back-calculation adds a correction:

$$
I_k = I_{k-1} + K_i T\,\tilde e_k + \frac{T}{T_t}\left(\omega^{lim}_{k-1} - \omega^{raw}_{k-1}\right)
$$

Set the tracking time constant as T_t ≈ √(T_i·T_d).

**Derivative on measurement, low-pass filtered.** Taking the derivative of the measurement avoids derivative kick when a new target is acquired:

$$
D_k = \frac{T_f}{T_f + T} D_{k-1} - \frac{K_d}{T_f + T}\left(y_k - y_{k-1}\right), \qquad T_f = \frac{K_d}{N K_p},\; N \approx 5\text{–}10
$$

Here y is the target bearing in the camera frame. In practice, using −ė filtered through the α-β residual is cleaner.

### 3.2 Feedforward Terms

$$
FF_k = K_{ff}\,\hat{\dot\lambda}_k \;-\; \omega_{base,k}
$$

- K_ff is between 0.8 and 1.0. Values below 1 reduce the effect of noise in the estimate.
- The ω_base term (IMU base rate projected onto the axis) cancels platform motion.

### 3.3 Raw Command

$$
\omega^{raw}_k = P_k + I_k + D_k + FF_k
$$

---

## 4. Smoothing: Rate, Acceleration and Jerk Limiting

The limits are applied as a cascade.

**1. Rate saturation:**
$$
\omega^{sat} = \text{clip}(\omega^{raw}, -\omega_{max}, \omega_{max})
$$

**2. Jerk limit on the acceleration request:**
$$
a^{req} = \frac{\omega^{sat} - \omega_{k-1}}{T}, \qquad
a_k = a_{k-1} + \text{clip}(a^{req} - a_{k-1},\, -j_{max}T,\, j_{max}T)
$$

**3. Acceleration limit:**
$$
a_k \leftarrow \text{clip}(a_k, -a_{max}, a_{max})
$$

**4. Braking-distance constraint near mechanical stops.** For pitch, with joint limits:
$$
|\omega_k| \le \sqrt{2\,a_{max}\,(\phi_{limit} - \phi_k)}
$$

**5. Integrate to get the command:**
$$
\omega^{lim}_k = \omega_{k-1} + a_k T
$$

The final value ω^lim is fed back to the anti-windup term in §3.1.

**Error-dependent acceleration.** For a cinematic feel, schedule the acceleration limit on error size:

$$
a_{max}(e) = a_{min} + (a_{hi} - a_{min})\,\text{sat}\!\left(\frac{|e|}{e_{ref}}\right)
$$

Small corrections are then gentle, while large reacquisitions are fast.

---

## 5. Stability and Tuning Guidelines

With the inner rate loop modeled as G_i(s) = ω_bw / (s + ω_bw), the plant from rate command to angle is approximately G(s) ≈ ω_bw / (s(s + ω_bw)), followed by the vision delay e^{−sτ_v}.

| Parameter | Rule of thumb |
|---|---|
| Outer crossover ω_c | ≤ min(ω_bw/5, 1/(3τ_v)) |
| K_p | ≈ ω_c |
| K_i | ≈ K_p·ω_c/5 (keep small; feedforward handles motion) |
| K_d | 0–0.1·K_p (often unnecessary with feedforward) |
| K_ff | 0.9 |
| a_max | 2–6 rad/s² for smooth; 15+ for sport |
| j_max | 10–40 rad/s³ |
| Phase margin target | ≥ 50° including τ_v |

**Tuning order:**
1. Set K_ff = 0 and tune K_p on a static target until there is no oscillation.
2. Add K_i until steady-state offset is removed.
3. Enable feedforward with a moving target and raise K_ff until lag is about zero without lead.
4. Tighten the a_max and j_max limits until motion looks smooth.
5. Verify step response with saturation active, checking that there is no windup overshoot.

---

## 6. Reference Implementation (per axis)

```python
import math

class AxisTracker:
    def __init__(s, T, Kp, Ki, Kd, Kff, N, w_max, a_max, j_max,
                 dead0, alpha, tau_v, coast_t=0.5):
        s.T, s.Kp, s.Ki, s.Kd, s.Kff, s.N = T, Kp, Ki, Kd, Kff, N
        s.w_max, s.a_max, s.j_max = w_max, a_max, j_max
        s.dead0, s.tau_v, s.coast_t = dead0, tau_v, coast_t
        s.alpha = alpha; s.beta = alpha**2 / (2 - alpha)
        s.Tt = max(T, math.sqrt((Kp/Ki if Ki else 1) * (Kd/Kp if Kp else 0)) or T)
        s.I = s.D = s.w = s.a = 0.0
        s.lam = s.lam_dot = 0.0; s.prev_y = None
        s.raw_prev = s.lim_prev = 0.0; s.lost_t = 0.0; s.init = False

    @staticmethod
    def clip(x, lo, hi): return max(lo, min(hi, x))

    def step(s, e_meas, phi, w_base, valid=True, sigma=0.0):
        T = s.T
        # --- target estimation (alpha-beta) ---
        if valid:
            lam_m = phi + e_meas
            if not s.init: s.lam, s.init = lam_m, True
            lam_p = s.lam + T * s.lam_dot
            r = lam_m - lam_p
            s.lam = lam_p + s.alpha * r
            s.lam_dot += s.beta / T * r
            s.lost_t = 0.0
            e = e_meas + (s.lam_dot - s.w) * s.tau_v   # latency comp
        else:
            s.lost_t += T
            s.lam += T * s.lam_dot
            if s.lost_t > s.coast_t: s.lam_dot *= 0.9  # decay FF
            e = s.lam - phi

        # --- deadband ---
        d = s.dead0 + 2.0 * sigma
        e_t = 0.0 if abs(e) < d else e - math.copysign(d, e)

        # --- PID ---
        P = s.Kp * e_t
        s.I += s.Ki * T * e_t + (T / s.Tt) * (s.lim_prev - s.raw_prev)
        y = -e
        if s.prev_y is None: s.prev_y = y
        Tf = s.Kd / (s.N * s.Kp) if s.Kd > 0 else 0.0
        s.D = (Tf * s.D - s.Kd * (y - s.prev_y)) / (Tf + T) if s.Kd > 0 else 0.0
        s.prev_y = y

        FF = s.Kff * s.lam_dot - w_base
        raw = P + s.I + s.D + FF

        # --- rate / jerk / accel limiting ---
        w_sat = s.clip(raw, -s.w_max, s.w_max)
        a_req = (w_sat - s.w) / T
        s.a += s.clip(a_req - s.a, -s.j_max * T, s.j_max * T)
        s.a = s.clip(s.a, -s.a_max, s.a_max)
        # prevent overshoot of target rate due to jerk-limited accel
        w_new = s.w + s.a * T
        if (w_sat - s.w) * (w_sat - w_new) < 0:
            w_new, s.a = w_sat, 0.0
        s.w = w_new

        s.raw_prev, s.lim_prev = raw, s.w
        return s.w   # rate command to inner motor loop
```

The usage pattern is one instance each for yaw and pitch, run at camera frame rate or higher. If the controller runs faster than detections arrive, upsample using the α-β prediction.

---

## 7. Cycle 2 Validation Checklist

- **Static target:** steady-state error below the deadband, with no limit cycling.
- **Constant-velocity target:** tracking lag below 0.5°, which confirms the feedforward works.
- **Step reacquisition (30°):** smooth S-curve motion with no overshoot beyond 5%.
- **Saturation test:** no windup rebound after a long saturated slew.
- **Base disturbance (handheld or vehicle vibration):** at least 20 dB attenuation below 2 Hz.
- **Target occlusion:** coasts for 0.5 s, then decelerates smoothly within the a_max limit.

**Candidate Cycle 3 extensions:**
- Replace the α-β filter with a constant-acceleration Kalman filter (adds acceleration feedforward).
- MPC-based trajectory shaping.
- Composition-aware offsets, such as rule-of-thirds framing.
- Cross-axis coupling compensation at high pitch angles (sec θ yaw scaling).