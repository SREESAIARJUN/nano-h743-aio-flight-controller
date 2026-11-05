# Autonomous Gimbal Visual Tracking Controller: Cycle 10 Formulation

**PID + Velocity Feedforward + Acceleration/Jerk Limiting, with Latency and Ego-Motion Compensation**

I don't have the record of Cycles 1–9, so this is a complete, self-contained formulation. It assumes a typical progression: P → PID → feedforward → limiting → latency/ego-motion handling. Cycle 10 consolidates these into one coherent controller with consistent units, saturation, and a defined state machine.

---

## 1. Problem Setup

**Axes:** Pan (yaw, ψ) and tilt (pitch, θ). The two axes are controlled independently, with a cross-coupling correction (§7). Roll is stabilized separately by the IMU loop.

**Inputs per vision frame k (rate f_v ≈ 30–60 Hz, latency τ_v):**
- Target pixel centroid (u, v), bounding box size, and detection confidence c ∈ [0,1].
- Camera intrinsics: f_x, f_y, c_x, c_y.

**Inputs at control rate (f_c ≈ 200–1000 Hz):**
- Gimbal encoder angles q = [ψ, θ].
- Gimbal IMU angular rates ω_g.
- Base (vehicle) IMU rates ω_b.

**Output:** The angular rate command ω_cmd = [ψ̇_cmd, θ̇_cmd], passed to the inner motor rate loop. The inner loop is assumed to have bandwidth at least 5× the outer loop.

---

## 2. Angular Error from Pixels

Use the exact angular error rather than the small-angle approximation:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

The tilt error is negated because image v increases downward.

**Target bearing in the world/base frame:**

$$
\lambda_k = q(t_k - \tau_v) + e_k
$$

Use the gimbal angle **at the image timestamp**, taken from an encoder ring buffer, not the current angle. This step alone removes most latency-induced oscillation.

---

## 3. Target Motion Estimation (Feedforward Source)

Run a constant-velocity Kalman filter per axis on the bearing λ. An α-β filter is an acceptable lightweight alternative.

**State:** x = [λ, λ̇]ᵀ

**Process model:**

$$
F = \begin{bmatrix}1 & \Delta t\\ 0 & 1\end{bmatrix}, \quad
Q = \sigma_a^2 \begin{bmatrix}\Delta t^4/4 & \Delta t^3/2\\ \Delta t^3/2 & \Delta t^2\end{bmatrix}
$$

**Measurement:** H = [1 0].

**Adaptive measurement noise:**

$$
R = \frac{\sigma_{px}^2 / f^2}{\max(c, c_{min})}
$$

Low-confidence detections are trusted less.

**Latency compensation:** Predict the state forward to the current time:

$$
\hat\lambda(t) = \hat\lambda_k + \hat{\dot\lambda}_k\,(t - t_k + \tau_v)
$$

**Outputs used by the controller:**
- Predicted error: ê = λ̂(t) − q(t)
- Feedforward rate: ω_ff = λ̇̂

**Gating:** Reject a measurement if its Mahalanobis innovation exceeds χ²₁(0.997) ≈ 9. This prevents ID-switch jumps.

---

## 4. Control Law (per axis, at control rate)

$$
\omega_{raw} = \underbrace{K_{ff}\,\hat{\dot\lambda}}_{\text{feedforward}}
\;+\; \underbrace{K_p\,\tilde e}_{P}
\;+\; \underbrace{K_i \!\int\! \tilde e\,dt}_{I}
\;-\; \underbrace{K_d\,\dot{\hat q}_f}_{D\text{ on measurement}}
\;-\; \underbrace{\omega_{b,\perp}}_{\text{ego-motion}}
$$

**Term details:**

- **Shaped error ẽ (smooth deadband).** This avoids hunting on detector jitter without a hard step:
$$
\tilde e = \operatorname{sign}(\hat e)\,\frac{\max(|\hat e|-\delta,\,0)^2}{|\hat e|-\delta+\epsilon}\quad(\text{or simply soft-threshold}),\qquad \delta \approx 1\text{–}2\ \text{px equiv.}
$$

- **Feedforward gain K_ff ∈ [0.8, 1.0].** Slightly below 1 avoids overshoot from velocity-estimate noise.

- **Derivative on measured gimbal rate, not on error.** This prevents derivative kick when a new target is acquired. Apply a first-order filter with cutoff N ≈ 5–10× the loop bandwidth:
$$
\dot{\hat q}_f[n] = \beta\,\dot{\hat q}_f[n-1] + (1-\beta)\,\omega_g[n], \quad \beta = e^{-2\pi f_d T}
$$

- **Ego-motion term.** ω_{b,⊥} is the base angular rate projected onto the gimbal axis. It subtracts vehicle rotation so the controller only fights true target motion. If the inner loop already stabilizes in the inertial frame, drop this term.

- **Gain scheduling by confidence:**
$$
K_p' = K_p\cdot g(c),\qquad g(c)=\text{clip}\!\left(\tfrac{c-c_{min}}{c_{full}-c_{min}},\,0.3,\,1\right)
$$

---

## 5. Rate, Acceleration, and Jerk Limiting

Apply the limits in order, every control tick T.

**1. Rate saturation:**
$$
\omega_{sat} = \text{clip}(\omega_{raw},\,-\omega_{max},\,\omega_{max})
$$

**2. Jerk limit.** This is optional but recommended for cinematic smoothness:
$$
a_{des} = \frac{\omega_{sat} - \omega_{cmd}[n-1]}{T}, \quad
a_{j} = \text{clip}\big(a_{des},\; a[n-1] - j_{max}T,\; a[n-1] + j_{max}T\big)
$$

**3. Acceleration limit:**
$$
a[n] = \text{clip}(a_j,\,-a_{max},\,a_{max}), \qquad
\omega_{cmd}[n] = \omega_{cmd}[n-1] + a[n]\,T
$$

**4. Braking-distance awareness.** Near a mechanical limit q_lim, cap the rate so the gimbal can stop in time:
$$
|\omega_{cmd}| \le \sqrt{2\,a_{max}\,|q_{lim} - q|}
$$

---

## 6. Anti-Windup (Back-Calculation)

Because both rate and acceleration limits create nonlinearity, use back-calculation tracking:

$$
I[n] = I[n-1] + T\left(K_i\,\tilde e + \frac{1}{T_t}\big(\omega_{cmd}[n] - \omega_{raw}[n]\big)\right)
$$

- Set the tracking time constant T_t ≈ √(T_i·T_d), or 0.1–0.5 s.
- Clamp I to ±I_max.
- Freeze integration when tracking state ≠ TRACK.

---

## 7. Pan–Tilt Coupling

At high tilt, pan motion produces reduced image displacement. Compensate as follows:

$$
\omega_{\psi,cmd} \leftarrow \frac{\omega_{\psi,cmd}}{\max(\cos\theta,\,0.2)}
$$

Apply this compensation **before** limiting. The 0.2 floor prevents gimbal-lock blowup near ±90°.

---

## 8. Tracking State Machine

| State | Entry condition | Controller behavior |
|---|---|---|
| **ACQUIRE** | New target locked | Feedforward off; K_p ramps up over ~0.5 s; tighter a_max (soft slew) |
| **TRACK** | Confidence c ≥ c_min and gate passed | Full law (§4) |
| **COAST** | Target lost for < T_coast (~1 s) | Feedforward only, using KF prediction decaying: ω = λ̇̂·e^{−t/τ_c}; integral frozen |
| **REACQUIRE / HOLD** | Lost for > T_coast | Decelerate to zero at a_max; hold angle or run search pattern |

---

## 9. Tuning Procedure

1. **Inner loop.** Identify the inner rate-loop bandwidth ω_i. Then set the outer bandwidth ω_o ≈ ω_i/5 and ensure ω_o < 2π·f_v/10.

2. **Proportional gain.** Set K_p ≈ ω_o. For a rate-commanded integrator plant, this gives crossover at ω_o.

3. **Integral gain.** Set K_i ≈ K_p·ω_o/4. This removes steady-state lag on accelerating targets.

4. **Derivative gain.** Keep K_d small, around 0.05–0.15. It is mainly for damping base disturbances.

5. **Feedforward gain.** Increase K_ff from 0.5 toward 1.0 while tracking a constant-rate target. Stop when lag is approximately 0 without overshoot.

6. **Acceleration and jerk limits.** For cinematic tracking, start with a_max ≈ 2–4·ω_max per second, then set j_max to give a ~0.15 s ramp-up time.

7. **Kalman filter noise.** Tune σ_a to the expected target maneuverability:
   - Pedestrians: ~0.5 rad/s²
   - Vehicles: ~2 rad/s²

**Validation tests:**
- Step response (acquisition)
- Constant-rate target (feedforward lag)
- Sinusoidal target at 0.5–2 Hz (phase lag)
- Occlusion (coast behavior)
- Base vibration (ego-motion rejection)

---

## 10. Reference Implementation (per axis)

```python
import math

class AxisTracker:
    def __init__(s, Kp, Ki, Kd, Kff, w_max, a_max, j_max, Tt, I_max, fd, T):
        s.Kp, s.Ki, s.Kd, s.Kff = Kp, Ki, Kd, Kff
        s.w_max, s.a_max, s.j_max = w_max, a_max, j_max
        s.Tt, s.I_max, s.T = Tt, I_max, T
        s.beta = math.exp(-2 * math.pi * fd * T)
        s.I = s.w_cmd = s.a = s.qd_f = 0.0

    @staticmethod
    def clip(x, lo, hi):
        return max(lo, min(hi, x))

    def soft_deadband(s, e, d):
        m = abs(e) - d
        return 0.0 if m <= 0 else math.copysign(m, e)

    def step(s, e_hat, lam_dot_hat, w_gimbal, w_base_axis,
             conf_gain, state, q, q_lim, coupling=1.0):
        T = s.T

        # D-on-measurement low-pass filter
        s.qd_f = s.beta * s.qd_f + (1 - s.beta) * w_gimbal

        e = s.soft_deadband(e_hat, 0.002)  # rad
        ff = s.Kff * lam_dot_hat if state in ("TRACK", "COAST") else 0.0
        kp = s.Kp * conf_gain if state != "COAST" else 0.0

        w_raw = (ff + kp * e + s.I - s.Kd * s.qd_f - w_base_axis) * coupling

        # Rate limit, including braking distance to the mechanical limit
        w_lim = min(s.w_max, math.sqrt(2 * s.a_max * max(abs(q_lim - q), 0.0)))
        w_sat = s.clip(w_raw, -w_lim, w_lim)

        # Jerk limit, then acceleration limit
        a_des = (w_sat - s.w_cmd) / T
        a_j = s.clip(a_des, s.a - s.j_max * T, s.a + s.j_max * T)
        s.a = s.clip(a_j, -s.a_max, s.a_max)
        s.w_cmd += s.a * T

        # Back-calculation anti-windup (integral frozen outside TRACK)
        if state == "TRACK":
            s.I += T * (s.Ki * e + (s.w_cmd - w_raw) / s.Tt)
            s.I = s.clip(s.I, -s.I_max, s.I_max)

        return s.w_cmd
```

---

## 11. Changes Consolidated in Cycle 10

- **Latency-aligned error:** The bearing uses the encoder angle at the image timestamp, and the KF forward-predicts by τ_v.
- **Feedforward source:** Comes from the target velocity in the world frame, not from the pixel-error derivative. Pixel-error derivatives are noisy and contaminated by gimbal motion.
- **Jerk-limited acceleration profile:** Combined with braking-distance rate limiting near endstops.
- **Unified anti-windup:** Back-calculation accounts for both the rate and acceleration saturations.
- **Confidence-scheduled gains and COAST mode:** Provide graceful degradation under occlusion.

If you share the Cycle 9 parameters or the failure modes you observed (e.g., overshoot on acquisition, lag on fast targets, jitter at rest), I can give specific gain and limit adjustments for this iteration.