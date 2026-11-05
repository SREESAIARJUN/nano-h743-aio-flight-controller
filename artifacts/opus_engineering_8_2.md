# Autonomous Gimbal Visual-Tracking Controller (Cycle 8)

I don't have the earlier cycles, so this is a complete, self-contained formulation of the full controller. It covers error definition, target-motion estimation, PID with feedforward, anti-windup, acceleration and jerk limiting, latency compensation, and tuning. Each axis (pan/yaw, tilt/pitch) runs an identical, decoupled loop unless noted.

---

## 1. Signal Definitions

| Symbol | Meaning |
|---|---|
| (u, v) | Target centroid in the image (px) |
| (c_x, c_y) | Principal point (px) |
| f_x, f_y | Focal length (px) |
| θ | Gimbal joint angle (rad), from the encoder or IMU |
| ω | Gimbal angular rate (rad/s) |
| T_s | Control period (e.g. 1/200 s) |
| T_v | Vision frame period (e.g. 1/30 s) |
| τ_d | Vision pipeline latency (capture → detection), in s |

### 1.1 Angular error (convert pixels to radians)

For the pan axis, using exact pinhole geometry rather than a small-angle approximation:

```
e_ψ = atan((u − c_x) / f_x)
e_φ = atan((v − c_y) / f_y)
```

Working in angles makes the gains independent of resolution and zoom. If the lens zooms, update f_x and f_y continuously.

### 1.2 Absolute target bearing (world frame)

```
ψ_t[k_v] = θ_ψ(t_capture) + e_ψ
```

Here θ_ψ(t_capture) is the gimbal angle interpolated at the image timestamp, not at the processing time. This single step removes most latency-induced oscillation.

---

## 2. Target Motion Estimation (Feedforward Source)

Run a constant-velocity Kalman filter on the absolute bearing ψ_t. A constant-acceleration model can be used for agile targets.

**State and model:**

```
State x = [ψ_t, ψ̇_t]ᵀ

Predict (every T_s):
  F = [[1, T_s], [0, 1]]
  Q = q · [[T_s³/3, T_s²/2], [T_s²/2, T_s]]
  x⁻ = F x
  P⁻ = F P Fᵀ + Q
```

**Update (each vision frame), with H = [1, 0] and R = σ_px² / f_x²:**

1. Propagate the stored state to t_capture, apply the update, then re-propagate to now by τ_d. Equivalently, keep a short state history buffer.
2. Gate the measurement: reject it if the Mahalanobis distance d² > 9 (outlier or re-ID jump).

**Outputs used by the controller:**

```
ψ̂_t(now)       predicted target bearing
ψ̇̂_t(now)      target angular velocity  → feedforward
ψ̈̂_t           (optional) from CA model → acceleration feedforward
```

**Tuning q:** small q gives smooth but laggy estimates; large q gives responsive but noisy ones. A typical starting range is q ∈ [0.5, 5] rad²/s³.

---

## 3. Reference Shaping (Smooth Follow)

Raw predicted bearings are not fed directly to the PID. Instead, generate a smooth reference ψ_r that chases ψ̂_t under velocity, acceleration, and jerk limits. This is the "smooth visual follow" layer.

### 3.1 Deadband with soft entry (prevents hunting at center)

```
e_raw = ψ̂_t − ψ_r
e_db  = sign(e_raw) · max(0, |e_raw| − δ) · (|e_raw| / (|e_raw| + δ))   # C¹-smooth deadband
```

A typical value is δ ≈ 0.3–0.5° (roughly 1% of the field of view).

### 3.2 Desired reference velocity

The desired velocity is feedforward plus a proportional catch-up term:

```
ω_des = ψ̇̂_t + K_r · e_db
ω_des = clamp(ω_des, −ω_max, ω_max)
```

### 3.3 Acceleration and jerk limiting (S-curve)

```
a_des = (ω_des − ω_r) / T_s
a_des = clamp(a_des, −a_max, a_max)

j     = (a_des − a_r) / T_s
j     = clamp(j, −j_max, j_max)

a_r  ← a_r + j · T_s
ω_r  ← ω_r + a_r · T_s
ψ_r  ← ψ_r + ω_r · T_s
```

### 3.4 Braking-aware limit (prevents overshoot when closing on target)

```
ω_brake = sign(e_db) · √(2 · a_max · |e_db|)
if |ω_des − ψ̇̂_t| > |ω_brake|:
    ω_des = ψ̇̂_t + ω_brake
```

This guarantees the reference can decelerate to the target's velocity within the remaining error at a_max.

**Output of the shaping layer:** a consistent triple (ψ_r, ω_r, a_r).

---

## 4. Inner Tracking Controller: PID + Velocity and Acceleration Feedforward

The controller tracks the shaped reference using fast encoder or IMU feedback, not vision. This decouples loop bandwidth from the camera frame rate.

### 4.1 Control law (rate-command gimbal, the typical case)

```
e   = ψ_r − θ
ω_cmd = ω_r                                   # velocity feedforward
      + K_p · e
      + K_i · ∫ e dt
      + K_d · (ω_r − ω_meas)                  # derivative on velocity error, not on e
```

Taking the derivative on the velocity error (ω_r − ω_meas) avoids derivative kick and reuses the clean reference rate.

### 4.2 Torque-command gimbal (direct-drive BLDC)

```
τ_cmd = J · a_r                               # acceleration feedforward
      + B · ω_r                               # viscous friction feedforward
      + τ_c · tanh(ω_r / ω_ε)                 # smoothed Coulomb friction
      + K_p e + K_i ∫e + K_d (ω_r − ω_meas)
```

### 4.3 Discrete implementation (Tustin integral, filtered derivative)

```
I[k]   = I[k−1] + K_i · T_s · (e[k] + e[k−1]) / 2
D[k]   = α · D[k−1] + (1 − α) · K_d · (ω_r[k] − ω_meas[k]),   α = τ_f / (τ_f + T_s)
u_pid  = K_p · e[k] + I[k] + D[k]
u      = u_ff + u_pid
```

### 4.4 Anti-windup (back-calculation) and output limits

```
u_sat = clamp(u, −u_max, u_max)
I[k] += K_aw · T_s · (u_sat − u)          # K_aw ≈ 1 / T_i  or  √(K_i / K_d)
```

Also apply these rules:
- Freeze the integrator when the target is lost.
- Freeze the integrator when the reference is acceleration-saturated (|a_r| = a_max), since a large error there is expected and not a steady-state bias.

### 4.5 Output-stage acceleration limit (hardware protection)

```
Δu_max = a_hw · T_s
u_out  = u_prev + clamp(u_sat − u_prev, −Δu_max, Δu_max)
```

---

## 5. Supervisory State Machine

| State | Entry condition | Behavior |
|---|---|---|
| **ACQUIRE** | New lock, \|e\| large | Higher ω_max; integrator off; jerk-limited slew |
| **TRACK** | \|e\| < 3δ for N frames | Full PID + feedforward; nominal limits |
| **COAST** | Detection lost, t < T_coast (≈1 s) | Reference continues on KF prediction; ψ̇̂ decays by e^(−t/τ_c); integrator frozen |
| **SEARCH / HOLD** | t > T_coast | Decelerate at a_max to stop; hold or run a search pattern |
| **REACQUIRE** | Detection gated in | Re-initialize KF velocity with blending; no step in ψ_r |

**Bumpless transfer:** on every state change, re-seed ψ_r ← θ, ω_r ← ω_meas, a_r ← 0 (or the measured acceleration). This prevents steps in the command.

---

## 6. Two-Axis Coupling Considerations

- **Pitch-dependent yaw scaling.** At tilt φ, image-space yaw error maps to joint yaw rate with a factor of 1/cos φ. Clamp the factor near ±90° to avoid gimbal-lock singular gains (or switch to a roll-assisted mode).
- **Base-motion rejection.** If the gimbal is on a moving platform, add the base angular rate from the IMU, projected into joint axes, as a further feedforward term: ω_cmd += −R_joint · ω_base.

---

## 7. Stability and Tuning Procedure

1. **Identify the plant.** Run a step or chirp on the rate loop and fit G(s) = K / (s(τ_m s + 1)) e^(−s τ_a).
2. **Tune the inner loop** for a phase margin of at least 45° and a crossover of ω_c ≈ 1/(4 τ_total).
   - K_p ≈ ω_c / K
   - T_i ≈ 4 / ω_c
   - K_d chosen to add roughly 20–30° of phase at ω_c
3. **Set the feedforward gain.** Start with K_ff = 1. Trim it down (0.8–0.95) if overshoot appears on velocity steps, since this indicates latency mismatch.
4. **Shaping-layer gain.** Set K_r to about 1/5 of the inner-loop bandwidth so the outer loop is separated in time scale.
5. **Limits.** a_max and j_max come from motor torque and image-blur constraints. For video smoothness, a typical starting point is a_max ≈ 2–4 rad/s² and j_max ≈ 20–50 rad/s³.
6. **Validate** against these targets:
   - steady ramp-tracking error (should approach 0 thanks to feedforward)
   - RMS pixel error
   - jerk RMS (a smoothness metric)
   - overshoot under 5% on target direction reversal

---

## 8. Reference Implementation (Python-style, one axis)

```python
class GimbalAxisController:
    def __init__(s, Ts, Kp, Ki, Kd, Kr, Kaw, wmax, amax, jmax, umax, deadband, tau_f):
        s.__dict__.update(locals())
        s.I = s.D = 0.0
        s.e_prev = 0.0
        s.psi_r = s.w_r = s.a_r = 0.0
        s.alpha = tau_f / (tau_f + Ts)

    def reset(s, theta, w_meas):                      # bumpless transfer
        s.psi_r, s.w_r, s.a_r = theta, w_meas, 0.0
        s.I = s.D = s.e_prev = 0.0

    def shape(s, psi_hat, w_hat):
        e = psi_hat - s.psi_r
        mag = max(0.0, abs(e) - s.deadband)
        e_db = math.copysign(mag * abs(e) / (abs(e) + s.deadband + 1e-9), e)
        w_brake = math.copysign(math.sqrt(2 * s.amax * abs(e_db)), e_db)
        corr = s.Kr * e_db
        if abs(corr) > abs(w_brake):
            corr = w_brake
        w_des = clamp(w_hat + corr, -s.wmax, s.wmax)
        a_des = clamp((w_des - s.w_r) / s.Ts, -s.amax, s.amax)
        j = clamp((a_des - s.a_r) / s.Ts, -s.jmax, s.jmax)
        s.a_r += j * s.Ts
        s.w_r += s.a_r * s.Ts
        s.psi_r += s.w_r * s.Ts
        return abs(s.a_r) >= s.amax - 1e-6              # saturation flag

    def step(s, psi_hat, w_hat, theta, w_meas, tracking=True):
        a_sat = s.shape(psi_hat, w_hat)
        e = s.psi_r - theta
        s.D = s.alpha * s.D + (1 - s.alpha) * s.Kd * (s.w_r - w_meas)
        u_ff = s.w_r
        u = u_ff + s.Kp * e + s.I + s.D
        u_sat = clamp(u, -s.umax, s.umax)
        if tracking and not a_sat:
            s.I += s.Ki * s.Ts * 0.5 * (e + s.e_prev)
        s.I += s.Kaw * s.Ts * (u_sat - u)               # back-calculation
        s.e_prev = e
        return u_sat
```

**Loop wiring:**
- **At 200 Hz:** run the KF predict, then call `step()` for each axis.
- **At 30 Hz:** on each vision frame, apply the timestamp-compensated KF update (Section 2).

---

## 9. Summary of Key Design Choices

1. Vision sets the target; the encoder or IMU closes the loop. This gives high bandwidth despite the slow camera.
2. Kalman velocity estimate → feedforward gives near-zero lag on constant-rate targets.
3. A jerk-, acceleration-, and braking-limited reference produces cinematically smooth motion with no overshoot on approach.
4. Timestamped latency compensation removes the main cause of oscillation in visual servoing.
5. Back-calculation anti-windup, integrator freezing, and bumpless state transitions keep behavior robust through target loss and reacquisition.

If you share the parameters or issues from previous cycles, such as observed overshoot or latency figures, I can tailor this into a specific delta for Cycle 8.