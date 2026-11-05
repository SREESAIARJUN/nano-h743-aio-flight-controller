# Autonomous Gimbal Visual Tracking Controller: Cycle 3 Formulation

**PID + Velocity Feedforward + Acceleration/Jerk Limiting + Latency Compensation**

I don't have the earlier cycles, so I've assumed this progression:

- **Cycle 1:** a basic PID on pixel error.
- **Cycle 2:** adds velocity feedforward.
- **Cycle 3 (this one):** makes the controller physically consistent and smooth. It adds:
  - angular-domain error,
  - latency-compensated target state estimation,
  - ego-motion compensation,
  - a 2-DOF PID with conditional anti-windup,
  - acceleration and jerk limiting with feedforward-aware saturation.

---

## 1. System Definition

**Axes:** pan (yaw, ψ) and tilt (pitch, θ). Each axis is controlled independently, with a cross-coupling correction (§7).

**Inner loop (assumed):** the gimbal motor driver accepts angular-rate commands ω_cmd and tracks them with bandwidth ω_bw. The rate loop is approximated as:

$$
G_{rate}(s) = \frac{\omega_{bw}}{s + \omega_{bw}}
$$

**Outer loop (this controller):** a visual position loop running at the camera frame rate f_v (e.g., 30–60 Hz). Its output is a rate command to the inner loop, which runs at f_c (e.g., 500 Hz–1 kHz).

### Variables

| Symbol | Meaning |
|---|---|
| (u, v) | Target centroid in pixels |
| (c_x, c_y) | Principal point |
| f_x, f_y | Focal lengths (px) |
| φ_g = [ψ_g, θ_g] | Gimbal joint angles (encoders) |
| ω_g | Gimbal angular rate (encoders/IMU) |
| ω_b | Vehicle body rate (IMU) |
| τ_d | Total vision latency (exposure + inference + transport) |

---

## 2. Measurement: Pixel Error to Angular Error

Pixel error is nonlinear with field of view, so convert it to angles first:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = \arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

This makes the gains invariant to zoom. If the zoom changes, update f_x and f_y only; the gains stay the same.

**Absolute target bearing** at the image timestamp t_k:

$$
\beta(t_k) = \varphi_g(t_k) + e(t_k)
$$

Here φ_g(t_k) must be the encoder angle interpolated to the exposure midpoint, not the processing time. This step is critical. Without it, the gimbal's own motion during latency appears as target motion.

---

## 3. Target State Estimation and Latency Compensation

Run a per-axis constant-velocity Kalman filter (or an α-β-γ filter) on the absolute bearing β.

**State:** x = [β, β̇]ᵀ

**Prediction:**

$$
x_{k|k-1} = \begin{bmatrix}1 & T\\ 0 & 1\end{bmatrix} x_{k-1}, \qquad
Q = q\begin{bmatrix}T^3/3 & T^2/2\\ T^2/2 & T\end{bmatrix}
$$

**Update:** standard KF with measurement noise R = σ_px² / f².

Two gates protect the filter from bad measurements:

- **Innovation gate:** reject the measurement if the squared Mahalanobis distance exceeds χ²₁(0.99) ≈ 6.63. This blocks detector jumps.
- **Lost-target handling:** if no detection arrives for more than N frames, coast on the prediction with decaying velocity, β̇ ← λβ̇ with λ ≈ 0.95. After a timeout, switch to hold or search mode.

**Latency compensation:** propagate the estimate forward to the current control time:

$$
\hat\beta(t) = \hat\beta_k + \hat{\dot\beta}_k\,(t - t_k), \qquad t - t_k \approx \tau_d + \text{elapsed}
$$

**Smooth reference generation:** the reference is evaluated at the fast inner rate f_c between vision frames. This turns a 30 Hz stair-step into a continuous ramp, which is a major source of smoothness.

---

## 4. Control Law (Per Axis)

### 4.1 Tracking error

$$
e(t) = \hat\beta(t) - \varphi_g(t)
$$

### 4.2 Feedforward terms

**Target velocity feedforward:**

$$
\omega_{ff,tgt} = K_{ff}\,\hat{\dot\beta}(t), \qquad K_{ff} \in [0.8, 1.0]
$$

**Ego-motion feedforward:** cancels vehicle rotation projected onto the gimbal axis:

$$
\omega_{ff,ego} = -\,\mathbf{J}(\varphi_g)\,\omega_b
$$

Here **J** maps body rates to joint rates. Skip this term if the stabilization loop already handles it.

### 4.3 2-DOF PID (feedback)

$$
\omega_{fb} = K_p\,e + K_i \int e\,dt + K_d\,\dot e_f
$$

Implementation details:

- **Derivative on measurement:** use ė_f = −φ̇_g,f for the D term. This avoids derivative kick when the target is reacquired.
- **First-order D filter:** use N ≈ 5–10.
- **Velocity is handled by feedforward:** the PID only has to correct residual error. This allows low gains and a smooth response.

**Deadband with smooth blend** (prevents hunting when centered):

$$
e' = e\cdot\left(1 - e^{-(e/e_{db})^2}\right)
$$

### 4.4 Unsaturated command

$$
\omega_{des} = \omega_{ff,tgt} + \omega_{ff,ego} + \omega_{fb}
$$

---

## 5. Acceleration and Jerk Limiting

These limits are applied at the inner rate, with period Δt = 1/f_c.

### Step 1: Rate limit

$$
\omega_1 = \text{clip}(\omega_{des}, -\omega_{max}, \omega_{max})
$$

### Step 2: Acceleration limit (relative to the last issued command)

$$
a_{req} = \frac{\omega_1 - \omega_{cmd,k-1}}{\Delta t}, \qquad
a_1 = \text{clip}(a_{req}, -a_{max}, a_{max})
$$

### Step 3: Jerk limit

This produces S-curve smoothness, which is important for cinematic footage:

$$
a_k = a_{k-1} + \text{clip}(a_1 - a_{k-1}, -j_{max}\Delta t, j_{max}\Delta t)
$$

### Step 4: Braking-aware acceleration

This step prevents overshoot when a deceleration is needed. The command should not exceed the velocity from which the gimbal can still stop at the target:

$$
|\omega| \le \sqrt{2\,a_{max}\,|e|} + |\hat{\dot\beta}|
$$

If |ω₁| exceeds this bound, clamp it before Step 2.

### Step 5: Integrate

$$
\omega_{cmd,k} = \omega_{cmd,k-1} + a_k\,\Delta t
$$

### Feedforward-aware saturation

When limits bind, preserve the feedforward and saturate the feedback first:

$$
\omega_{fb}^{sat} = \text{clip}(\omega_{fb},\; -\omega_{max} - \omega_{ff},\; \omega_{max} - \omega_{ff})
$$

---

## 6. Anti-Windup

Use **conditional integration combined with back-calculation**:

$$
\dot I = K_i\,e + K_{aw}\,(\omega_{cmd} - \omega_{des})
$$

- Freeze I entirely when any of the following hold:
  - the target is lost,
  - |e| > e_large (e.g., 5°),
  - the acceleration limiter is active.
- Choose K_aw ≈ 1/T_i, where T_i = K_p / K_i.

---

## 7. Cross-Axis and Geometric Corrections

- **Pan gain scheduling vs. tilt:** near high elevation, a given yaw rate produces less image motion. Scale the pan loop:

$$
K_{\psi}^{eff} = \frac{K_\psi}{\max(\cos\theta_g,\ c_{min})}
$$

  with c_min ≈ 0.2 to avoid the gimbal-lock singularity.

- **Zenith handling:** near θ = ±90°, cap the pan rate and allow tilt to pass through the zenith rather than spinning the pan axis.

---

## 8. Discrete Implementation (Pseudocode)

```python
# --- Vision thread (f_v) ---
on_detection(u, v, t_img):
    e_meas = [atan((u-cx)/fx), atan((v-cy)/fy)]
    phi_img = encoder_interp(t_img)            # angle at exposure time
    beta = phi_img + e_meas
    for ax in (PAN, TILT):
        kf[ax].predict_to(t_img)
        if kf[ax].mahalanobis(beta[ax]) < GATE:
            kf[ax].update(beta[ax])
    last_seen = t_img

# --- Control thread (f_c) ---
loop every dt:
    t = now()
    for ax in (PAN, TILT):
        beta_hat, beta_dot = kf[ax].extrapolate(t)       # latency comp.
        if t - last_seen > T_lost:
            beta_dot *= decay
            freeze_I = True

        e = wrap(beta_hat - phi[ax])
        e = smooth_deadband(e, e_db)

        ff = Kff*beta_dot + ego_ff(ax, omega_body, phi)
        D  = d_filter(-phi_dot[ax])
        P  = Kp[ax]*sched(ax)*e
        fb = P + I[ax] + Kd*D

        w_des = ff + fb
        w_des = brake_limit(w_des, e, beta_dot, a_max)
        w = clip(w_des, -w_max, w_max)

        a_req = (w - w_prev[ax]) / dt
        a_req = clip(a_req, -a_max, a_max)
        a[ax] += clip(a_req - a[ax], -j_max*dt, j_max*dt)
        w_cmd = w_prev[ax] + a[ax]*dt

        if not freeze_I and abs(e) < e_large:
            I[ax] += (Ki*e + Kaw*(w_cmd - w_des)) * dt

        send_rate(ax, w_cmd)
        w_prev[ax] = w_cmd
```

---

## 9. Tuning Procedure

1. **Identify the plant:** measure ω_bw, τ_d, and the effective inertia limits that set a_max.
2. **Set the limits first:**
   - ω_max: footage-dependent, e.g., 60–120°/s.
   - a_max: e.g., 200–400°/s².
   - j_max: e.g., 2000°/s³, or lower for cinematic smoothness.
3. **Feedforward first, with P = I = D = 0:** raise K_ff until a constant-velocity target lags by a roughly constant small offset.
4. **K_p:** choose an outer bandwidth ω_o ≈ ω_bw / 5. Then K_p ≈ ω_o, and also K_p < 1/(3τ_d) for latency stability margin.
5. **K_i:** set T_i ≈ 4/ω_o. Its only job is removing residual bias, such as calibration offsets.
6. **K_d:** usually small or zero, because feedforward plus the rate inner loop already provide damping. Add it only if overshoot persists.
7. **KF noise q:** trades responsiveness against jitter.
   - High q: snappy but noisy.
   - Low q: smooth but laggy on maneuvers.
   - Consider adaptive q driven by the size of the innovation.

---

## 10. Performance Metrics for Cycle 3 Validation

| Metric | Target |
|---|---|
| Steady-state error, constant-velocity target | < 0.2° (≈ a few px) |
| Ramp-tracking lag | ≈ 0 (feedforward-limited) |
| Overshoot on step reacquisition | < 5% |
| RMS command jerk | Minimize; compare with Cycle 2 |
| Image-space jitter, stationary target | < 1 px RMS |
| Recovery after occlusion (1 s) | No windup spike; smooth reacquire |

---

## Summary

The final per-axis command is:

$$
\omega_{cmd} = \mathcal{L}_{j,a,\omega}\Big[\underbrace{K_{ff}\hat{\dot\beta}(t+\tau_d) - \mathbf{J}\omega_b}_{\text{feedforward}} + \underbrace{K_p e + K_i\!\int\! e + K_d\dot e_f}_{\text{feedback}}\Big]
$$

**Notation:**

- 𝓛 is the cascaded rate → acceleration → jerk limiter with braking awareness.
- e is computed in the angular domain against a latency-compensated, Kalman-predicted target bearing that is upsampled to the control rate.

Feedforward does the tracking, feedback cleans up the residual error, and the limiter guarantees smoothness.