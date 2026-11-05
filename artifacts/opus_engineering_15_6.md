# Autonomous Gimbal Tracking Controller: PID with Velocity Feedforward and Acceleration Limiting (Cycle 15)

## 1. System Overview

```
 Camera ──► Detector ──► Pixel→Angle ──► LOS Estimator ──► Controller ──► Rate/Accel ──► Gimbal Rate Loop
 (frames)   (u,v,conf)   (e_ψ, e_θ)      (KF: angle,rate)   PID + FF      Limiter        (motor driver)
                ▲                              ▲                                              │
                └──────── latency τ_v ─────────┴──────── IMU / encoder (gimbal attitude) ◄────┘
```

Each axis (yaw ψ, pitch θ) runs an independent outer **angle loop** that outputs a rate command. The command goes to the gimbal's inner **rate loop**, which is assumed to be fast (≥5× the outer bandwidth).

---

## 2. Measurement Model: Pixel Error to Angular Error

The camera has intrinsics (f_x, f_y, c_x, c_y). The detection centroid (u, v) maps to angular error as:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

The sign convention is that positive pitch points up, while image v increases downward.

**Inertial line-of-sight (LOS) angle.** This is what gets fused in the estimator:

$$
\lambda_\psi[k] = \psi_g(t_k - \tau_v) + e_\psi[k]
$$

- ψ_g is the gimbal angle from the encoder or IMU, **sampled at the image capture timestamp**, not at the processing time.
- This decouples target motion from gimbal motion. It is the key step that makes feedforward valid.

For large pitch angles, apply the full rotation instead of adding angles: form the camera ray r_c, compute r_I = R_IG(t−τ_v) · R_GC · r_c, and extract azimuth and elevation.

---

## 3. Target LOS Estimator (Constant-Velocity Kalman Filter)

The state per axis is x = [λ, λ̇]ᵀ.

$$
x_{k+1} = \begin{bmatrix}1 & T\\0 & 1\end{bmatrix}x_k + w_k,\qquad
Q = q\begin{bmatrix}T^3/3 & T^2/2\\ T^2/2 & T\end{bmatrix}
$$

$$
z_k = \lambda_k + n_k,\qquad R = \sigma_{pix}^2 / f^2 \;(\text{scaled by } 1/\text{conf})
$$

**Latency compensation.** Propagate the filtered state forward by the measured pipeline delay τ_v:

$$
\hat\lambda_{now} = \hat\lambda + \hat{\dot\lambda}\,\tau_v,\qquad \hat\omega_{tgt} = \hat{\dot\lambda}
$$

**Outlier gating.** Reject a measurement if its normalized innovation squared exceeds χ²₁(0.99) ≈ 6.63. Count rejections toward the lost-target logic.

---

## 4. Control Law

The tracking error is predicted to the current time, using inertial LOS minus the current gimbal angle:

$$
e(t) = \hat\lambda_{now} - \psi_g(t)
$$

A soft deadband suppresses jitter near center without introducing a discontinuity:

$$
e_d = \begin{cases} 0 & |e| \le \delta \\ \operatorname{sign}(e)\,(|e|-\delta)\cdot\frac{|e|}{|e|+\delta} & \text{otherwise}\end{cases}
$$

### Rate Command

$$
\boxed{\;\omega_{raw} = K_{ff}\,\hat\omega_{tgt} \;+\; K_p e_d \;+\; K_i\!\int e_d\,dt \;-\; K_d\,\dot\psi_{g,f}\;}
$$

| Term | Role |
|---|---|
| **K_ff · ω̂_tgt** | Velocity feedforward. It carries most of the tracking effort, so steady-state lag on a constant-rate target goes to approximately 0 with K_ff ≈ 1. |
| **K_p e** | Corrects the residual error and sets the outer-loop bandwidth. |
| **K_i ∫e** | Removes bias from FF mismatch, friction, and calibration offsets. Kept small. |
| **−K_d ψ̇_g,f** | Derivative on measurement, not on error. This avoids kicks on target reacquisition. It is low-pass filtered with τ_d ≈ T_frame. |

**Confidence scheduling.** Use c ∈ [0, 1] from the detector and track quality:

$$
K_p' = K_p\cdot(0.5 + 0.5c),\qquad K_{ff}' = K_{ff}\cdot c
$$

---

## 5. Rate and Acceleration (Slew) Limiting

The limits apply in this order:

1. **Rate clamp:**
$$
\omega_{sat} = \operatorname{clip}(\omega_{raw}, -\omega_{max}, \omega_{max})
$$

2. **Acceleration limit:**
$$
\Delta\omega_{max} = a_{max} T,\qquad
\omega_{cmd}[k] = \omega_{cmd}[k-1] + \operatorname{clip}(\omega_{sat} - \omega_{cmd}[k-1],\, -\Delta\omega_{max},\, \Delta\omega_{max})
$$

3. **Optional jerk limit**, for cinematic smoothness: apply the same slew limiter one level up, on the acceleration, with bound j_max · T.

4. **Braking-aware rate cap.** This prevents overshoot when approaching the target under an acceleration limit:
$$
\omega_{max,eff} = \min\!\left(\omega_{max},\; \sqrt{2\,a_{max}\,|e|} + |\hat\omega_{tgt}|\right)
$$

### Anti-Windup (Back-Calculation)

$$
\dot I = K_i e_d + \frac{1}{T_t}(\omega_{cmd} - \omega_{raw}),\qquad T_t \approx \sqrt{T_i T_d}\ \text{(or } 0.5\,T_i)
$$

Also freeze integration when the target is lost or confidence c < 0.3.

---

## 6. Target-Loss State Machine

| State | Condition | Behavior |
|---|---|---|
| **TRACK** | Valid detection within gate | Full control law |
| **COAST** | No detection for less than t_coast (≈0.5 s) | Propagate the KF. Use ω = K_ff ω̂_tgt · e^(−t/τ_c), with P and D on the predicted error and I frozen |
| **SEARCH / HOLD** | t > t_coast | Decelerate to 0 at a_max, hold, or run a search pattern |
| **REACQUIRE** | Detection after loss | Reset KF velocity covariance, bumplessly re-seed I so that ω_cmd is continuous, and ramp K_ff from 0 to 1 over ~0.3 s |

---

## 7. Reference Implementation (Python, Per Axis)

```python
import math

class AxisTracker:
    def __init__(s, T, Kp=4.0, Ki=0.4, Kd=0.15, Kff=1.0,
                 w_max=math.radians(180), a_max=math.radians(720),
                 deadband=math.radians(0.15), q=0.5, r=1e-5, tau_d=0.03):
        s.__dict__.update(locals()); del s.__dict__['s']
        s.x = [0.0, 0.0]; s.P = [[1, 0], [0, 1]]
        s.I = 0.0; s.w_cmd = 0.0; s.dpsi_f = 0.0; s.prev_psi = None
        s.lost_t = 0.0; s.ff_ramp = 1.0

    # --- Kalman filter -------------------------------------------------
    def kf_predict(s, dt):
        x, P, q = s.x, s.P, s.q
        s.x = [x[0] + dt * x[1], x[1]]
        p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + q * dt**3 / 3
        p01 = P[0][1] + dt * P[1][1] + q * dt * dt / 2
        p11 = P[1][1] + q * dt
        s.P = [[p00, p01], [p01, p11]]

    def kf_update(s, z, conf):
        R = s.r / max(conf, 0.05)
        y = z - s.x[0]; S = s.P[0][0] + R
        if y * y / S > 6.63:
            return False                              # gate reject
        k0, k1 = s.P[0][0] / S, s.P[1][0] / S
        s.x = [s.x[0] + k0 * y, s.x[1] + k1 * y]
        P = s.P
        s.P = [[(1 - k0) * P[0][0], (1 - k0) * P[0][1]],
               [P[1][0] - k1 * P[0][0], P[1][1] - k1 * P[0][1]]]
        return True

    # --- Control step at rate 1/T ---------------------------------------
    def step(s, psi_g, meas=None, psi_at_capture=None, tau_v=0.0, conf=1.0):
        T = s.T
        s.kf_predict(T)
        ok = False
        if meas is not None:
            ok = s.kf_update(psi_at_capture + meas, conf)   # inertial LOS
        s.lost_t = 0.0 if ok else s.lost_t + T
        if ok and s.ff_ramp < 1.0:
            s.ff_ramp = min(1.0, s.ff_ramp + T / 0.3)

        lam_now = s.x[0] + s.x[1] * tau_v
        w_tgt = s.x[1]
        e = math.remainder(lam_now - psi_g, 2 * math.pi)

        # soft deadband
        a, d = abs(e), s.deadband
        ed = 0.0 if a <= d else math.copysign((a - d) * a / (a + d), e)

        # derivative on measurement (filtered)
        if s.prev_psi is not None:
            raw = (psi_g - s.prev_psi) / T
            s.dpsi_f += (T / (s.tau_d + T)) * (raw - s.dpsi_f)
        s.prev_psi = psi_g

        if s.lost_t > 0.5:                            # HOLD: brake to zero
            target = 0.0; s.ff_ramp = 0.0
        else:
            c = conf if ok else math.exp(-s.lost_t / 0.2)
            ff = s.Kff * c * s.ff_ramp * w_tgt
            target = ff + s.Kp * (0.5 + 0.5 * c) * ed + s.I - s.Kd * s.dpsi_f

        # braking-aware rate cap, then rate clamp
        w_lim = min(s.w_max, math.sqrt(2 * s.a_max * abs(e)) + abs(w_tgt))
        w_sat = max(-w_lim, min(w_lim, target))

        # acceleration slew limit
        dw = max(-s.a_max * T, min(s.a_max * T, w_sat - s.w_cmd))
        s.w_cmd += dw

        # integrator with back-calculation anti-windup
        if ok and conf > 0.3:
            Tt = 0.5 * (s.Kp / max(s.Ki, 1e-6))
            s.I += T * (s.Ki * ed + (s.w_cmd - target) / Tt)
        return s.w_cmd
```

Run one instance per axis at the control rate (e.g., 200 Hz). Vision updates arrive asynchronously at 30–60 Hz. Between frames, `meas=None`, so the filter predicts and the loop keeps running smoothly.

---

## 8. Tuning Procedure

1. **Inner rate loop first.** Verify a step in the rate command settles in under 30 ms with less than 10% overshoot.
2. **Measure the vision latency τ_v** end to end, for example with an LED-blink test. This determines the achievable bandwidth: ω_c ≲ 1/(3 τ_v) rad/s, which is ≈ 4 rad/s for 80 ms.
3. **Set K_p ≈ ω_c.** Set K_ff = 0, K_i = 0, K_d = 0. Track a stationary target and confirm no oscillation.
4. **Enable feedforward.** Pan a target at constant rate and raise K_ff from 0.8 toward 1.0 until the steady lag disappears. If you see overshoot, the latency compensation is underestimated. Fix τ_v rather than lowering K_ff.
5. **Add integral action:** K_i ≈ K_p · ω_c / 10.
6. **Add derivative action only if needed** to damp gimbal or mount resonance: K_d ≈ 0.02–0.1.
7. **Set a_max for the content type.** Use ~300–500 °/s² for cinematic footage and ~1000+ °/s² for aggressive tracking. Raise the KF q if the target maneuvers hard.
8. **Deadband:** set δ ≈ 1–2 pixels expressed in angle.

## 9. Cycle 15 Refinements (vs. a Baseline PID Tracker)

- The feedforward is computed in **inertial LOS space** with capture-time attitude stamping. This removes the self-induced motion feedback loop that causes "hunting."
- A **braking-aware rate cap** eliminates overshoot caused by the acceleration limit.
- **Back-calculation anti-windup** accounts for both the rate and acceleration saturations.
- **Confidence-scheduled** feedforward and proportional gains, plus a **ramped feedforward on reacquire**, provide bumpless transitions.
- The **soft deadband** is continuous, so it introduces no limit cycle at the deadband edge.

## 10. Validation Checklist

- Constant-rate target: steady-state error under 0.2° and no visible lag.
- Step target jump of 20°: no overshoot and an acceleration-limited trapezoid profile.
- Occlusion of 0.4 s: the camera keeps its heading and reacquires without a jerk.
- Detector noise of ±3 px: gimbal jitter under 0.05° RMS.
- 1 Hz sinusoidal target: phase lag under 15° with feedforward enabled.