# Autonomous Gimbal Tracking Controller: PID + Velocity Feedforward + Acceleration Limiting

*(Cycle 5. I don't have the earlier cycles, so this is a complete, self-contained formulation. Its refinements target the usual weaknesses of earlier iterations: lag on moving targets, jerk, integrator windup, and vision latency.)*

---

## 1. System Definition

**Axes:** pan (yaw, ψ) and tilt (pitch, θ). Each axis is controlled independently, with the same structure on both. The subscript *i* ∈ {ψ, θ} is dropped below.

**Inputs per control tick k (period Tₛ, e.g. 200 Hz inner loop):**

| Symbol | Meaning |
|---|---|
| (uₖ, vₖ) | Target centroid in the image (px), from the detector/tracker at camera rate T_c (e.g. 30 Hz) |
| (c_x, c_y), f_x, f_y | Principal point and focal lengths (px) |
| q_k | Gimbal joint angle (encoder) |
| ω_g | Gimbal joint rate (encoder differentiation or IMU) |
| ω_b | Vehicle/base angular rate (IMU), used for stabilization |
| τ_d | Total vision latency (exposure + inference + transport) |

**Output:** a gimbal rate command ω_cmd, sent to the motor's inner velocity loop.

---

## 2. Measurement Model: Pixel Error to Angular Error

Use the angular error, not raw pixels, so the gains are independent of zoom:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = \arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

**Latency compensation.** The image was captured τ_d ago. Convert the error to the line-of-sight (LOS) angle in the gimbal-base frame at capture time:

$$
\lambda_{meas} = q(t - \tau_d) + e
$$

Here q(t − τ_d) comes from a short ring buffer of encoder samples. This removes the classic oscillation caused by the gimbal "chasing" stale images.

---

## 3. Target State Estimator (for Feedforward)

Use a constant-velocity Kalman filter on the LOS angle per axis. The state is x = [λ, λ̇]ᵀ.

**Predict (every Tₛ):**

$$
x_{k|k-1} = F x_{k-1}, \quad F = \begin{bmatrix}1 & T_s\\ 0 & 1\end{bmatrix}, \quad
P = FPF^\top + Q
$$

$$
Q = \sigma_a^2 \begin{bmatrix} T_s^4/4 & T_s^3/2 \\ T_s^3/2 & T_s^2\end{bmatrix}
$$

**Update (when a detection arrives):** use H = [1 0] and measurement λ_meas, with R = σ_px² / f².
- If λ_meas arrived with delay τ_d, apply the update to the delayed state and re-propagate. Alternatively, propagate the measurement forward: λ_meas + λ̂̇·τ_d.
- Reject outliers with an innovation gate: ν²/S > χ²₁(0.99) ≈ 6.63.

**Outputs:**
- λ̂ is the predicted LOS angle now.
- λ̂̇ is the inertial target angular rate, which drives the feedforward.

**Tuning knob:** σ_a is the target maneuverability.
- Low values (≈0.5 rad/s²) give smooth, cinematic following.
- High values (≈5 rad/s²) give agile tracking.

---

## 4. Control Law

### 4.1 Tracking Error

$$
e_k = \hat\lambda_k - q_k \qquad (\text{wrapped to } (-\pi, \pi] \text{ for pan})
$$

**Deadband (smooth-follow aesthetic).** Use a soft deadband to avoid micro-hunting when the subject is near center:

$$
\tilde e = \begin{cases}
0 & |e| < e_{db} \\
\operatorname{sgn}(e)\,(|e| - e_{db}) & \text{otherwise}
\end{cases}
\qquad e_{db} \approx 0.3^\circ
$$

### 4.2 PID Terms (Discrete, Tustin/Backward Euler)

**Proportional:**

$$
P_k = K_p \tilde e_k
$$

**Integral (conditional integration anti-windup):**

$$
I_k = \begin{cases}
I_{k-1} + K_i T_s \tilde e_k & \text{if not saturated or } \operatorname{sgn}(\tilde e_k) \ne \operatorname{sgn}(\omega_{cmd,k-1}) \\
I_{k-1} & \text{otherwise}
\end{cases}
$$

The integral is clamped to |I| ≤ I_max.

**Derivative on measurement, low-pass filtered.** This avoids derivative kick when the target jumps:

$$
D_k = \frac{\tau_f}{\tau_f + T_s} D_{k-1} - \frac{K_d}{\tau_f + T_s}(q_k - q_{k-1}),
\qquad \tau_f = \frac{K_d}{N K_p},\; N \in [5, 20]
$$

### 4.3 Velocity Feedforward and Base-Motion Rejection

$$
\omega_{ff} = K_{ff}\,\hat{\dot\lambda} \; - \; \omega_{b,\text{axis}}
$$

- K_ff ∈ [0.8, 1.0]. Use 1.0 for zero steady-state lag on constant-rate targets. Slightly less than 1 is softer and more robust to estimator noise.
- The −ω_b term cancels vehicle rotation projected onto the axis. This is skip-able if the gimbal already runs an IMU stabilization loop.

**Confidence scaling.** Fade the feedforward when tracking quality drops:

$$
\omega_{ff} \leftarrow \alpha_k\, \omega_{ff}, \qquad \alpha_k = \operatorname{clip}\!\left(1 - \frac{\operatorname{tr}(P_k) - P_{lo}}{P_{hi} - P_{lo}}, 0, 1\right)
$$

### 4.4 Raw Command

$$
\omega_{raw,k} = \omega_{ff,k} + P_k + I_k + D_k
$$

---

## 5. Rate and Acceleration Limiting (Jerk-Aware Shaping)

### 5.1 Velocity Saturation

$$
\omega_{sat} = \operatorname{clip}(\omega_{raw}, -\omega_{max}, \omega_{max})
$$

### 5.2 Acceleration Limit (Slew-Rate Limiter)

$$
\Delta\omega = \operatorname{clip}\big(\omega_{sat} - \omega_{cmd,k-1},\; -a_{max} T_s,\; a_{max} T_s\big)
$$

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + \Delta\omega
$$

### 5.3 Optional Jerk Limit (Cinematic Mode)

Limit the change in acceleration as well:

$$
a_k = \operatorname{clip}\!\left(\frac{\omega_{sat}-\omega_{cmd,k-1}}{T_s},\; a_{k-1} - j_{max}T_s,\; a_{k-1} + j_{max}T_s\right), \quad |a_k|\le a_{max}
$$

### 5.4 Braking-Distance Awareness (Prevents Overshoot Under Accel Limits)

Cap the closing speed so the gimbal can stop at the target:

$$
|\omega_{sat} - \omega_{ff}| \le \sqrt{2\, a_{max}\, |e|}
$$

This is the "time-optimal" envelope. It is the key Cycle-5 fix for the overshoot that pure slew limiting causes on large re-acquisitions.

### 5.5 Anti-Windup Back-Calculation

Because the limiter alters the command, feed the difference back into the integrator:

$$
I_k \leftarrow I_k + K_{aw}\,(\omega_{cmd,k} - \omega_{raw,k})\,T_s, \qquad K_{aw} \approx \frac{1}{\sqrt{T_i T_d}} \text{ or } \frac{K_i}{K_p}
$$

---

## 6. Mechanical Limits and Mode Logic

**Joint limits (tilt):** apply a soft barrier near the limits:

$$
\omega_{max}^{+} = \min\!\big(\omega_{max},\ \sqrt{2a_{max}(q_{lim}^{+} - q)}\big)
$$

The negative side is handled symmetrically.

**Modes:**

| Mode | Condition | Behavior |
|---|---|---|
| TRACK | Valid detection, gate passed | Full law above |
| COAST | Detection lost for less than t_coast (≈1 s) | Feedforward only from the predicted λ̂; I frozen; K_p halved |
| SEARCH / HOLD | Lost for longer than t_coast | Decelerate at a_max to zero, or run a search pattern; reset I |
| REACQUIRE | Detection returns after loss | Re-initialize the KF at the measurement; braking envelope prevents a snap |

---

## 7. Stability and Tuning Guidance

**Plant model:** with a fast inner velocity loop, each axis behaves like G(s) ≈ e^(−sτ_d) / (s(τ_m s + 1)).

**Crossover rule of thumb:** limit the bandwidth relative to vision latency:

$$
\omega_c \lesssim \frac{1}{3\,\tau_d} \;\Rightarrow\; K_p \approx \omega_c
$$

For example, τ_d = 80 ms gives ω_c ≈ 4 rad/s, so K_p ≈ 4 s⁻¹.

**Starting gains:**
- K_i ≈ K_p · ω_c / 5
- K_d ≈ 0–0.1·K_p. Often small, because the feedforward plus the KF already provide lead.
- Aim for a phase margin of at least 45°. Verify with the Padé-approximated delay.

**Typical limits:**

| Parameter | Agile | Cinematic |
|---|---|---|
| ω_max | 180°/s | 60°/s |
| a_max | 720°/s² | 120°/s² |
| j_max | off | 600°/s³ |
| e_db | 0.1° | 0.5° |
| σ_a | 5 rad/s² | 0.5 rad/s² |

**Tuning order:**
1. Inner rate loop.
2. K_p with K_ff = 0.
3. Add K_ff and verify near-zero lag on a constant-rate target.
4. Add K_i for bias and friction.
5. Set the limits.
6. Add jerk shaping.

---

## 8. Reference Implementation (per axis, per tick)

```python
def gimbal_axis_step(s, meas, q, omega_base, dt, p):
    # --- Estimator ---
    s.kf.predict(dt)
    if meas is not None:
        lam_meas = s.q_hist.at(meas.t_capture) + atan((meas.px - p.c) / p.f)
        if s.kf.innovation_ok(lam_meas, gate=6.63):
            s.kf.update_delayed(lam_meas, meas.t_capture)
            s.t_last = now()
    lam, lam_dot = s.kf.x
    lost = now() - s.t_last
    mode = "TRACK" if lost < p.t_gap else ("COAST" if lost < p.t_coast else "HOLD")

    # --- Error ---
    e = wrap(lam - q)
    e_t = 0.0 if abs(e) < p.e_db else copysign(abs(e) - p.e_db, e)

    if mode == "HOLD":
        target = 0.0; s.I = 0.0
    else:
        kp = p.Kp * (0.5 if mode == "COAST" else 1.0)
        alpha = clip(1 - (s.kf.trP() - p.Plo) / (p.Phi - p.Plo), 0, 1)
        w_ff = alpha * p.Kff * lam_dot - omega_base

        P = kp * e_t
        if mode == "TRACK":
            sat = abs(s.w_cmd) >= p.w_max * 0.99
            if not sat or sign(e_t) != sign(s.w_cmd):
                s.I = clip(s.I + p.Ki * e_t * dt, -p.I_max, p.I_max)
        tau_f = p.Kd / (p.N * p.Kp) if p.Kd > 0 else 0
        s.D = (tau_f * s.D - p.Kd * (q - s.q_prev)) / (tau_f + dt) if p.Kd > 0 else 0

        w_raw = w_ff + P + s.I + s.D
        # braking envelope
        v_close = sqrt(2 * p.a_max * abs(e))
        w_raw = w_ff + clip(w_raw - w_ff, -v_close, v_close)
        target = w_raw

    # --- Limits: joint, rate, accel, jerk ---
    w_hi = min(p.w_max, sqrt(2 * p.a_max * max(p.q_max - q, 0)))
    w_lo = -min(p.w_max, sqrt(2 * p.a_max * max(q - p.q_min, 0)))
    w_sat = clip(target, w_lo, w_hi)

    a_des = (w_sat - s.w_cmd) / dt
    if p.j_max:
        a_des = clip(a_des, s.a - p.j_max * dt, s.a + p.j_max * dt)
    s.a = clip(a_des, -p.a_max, p.a_max)
    w_new = s.w_cmd + s.a * dt

    # --- Back-calculation anti-windup ---
    if mode != "HOLD":
        s.I += p.Kaw * (w_new - target) * dt

    s.w_cmd, s.q_prev = w_new, q
    s.q_hist.push(now(), q)
    return w_new
```

---

## 9. Compact Summary

$$
\boxed{
\omega_{cmd} = \mathcal{L}_{a,j}\Big\{\operatorname{sat}_{\omega}\Big[\underbrace{\alpha K_{ff}\hat{\dot\lambda} - \omega_b}_{\text{feedforward}} + \operatorname{clip}_{\pm\sqrt{2a_{max}|e|}}\big(K_p\tilde e + K_i\!\int\!\tilde e - K_d \dot q_f\big)\Big]\Big\}
}
$$

where:
- e = λ̂ − q, with λ̂ and its rate from a latency-compensated CV Kalman filter.
- 𝓛_{a,j} is the acceleration/jerk limiter.
- Back-calculation anti-windup couples the limiter output to the integrator.

**Validation metrics for this cycle:**
- RMS centering error on a constant-rate target: below 0.5°.
- Overshoot on 30° re-acquisition: below 5%.
- Peak jerk within j_max.
- No limit cycle with a stationary subject, which the deadband handles.