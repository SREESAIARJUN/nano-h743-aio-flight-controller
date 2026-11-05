# Autonomous Gimbal Tracking Controller: PID with Velocity Feedforward and Acceleration Limiting (Cycle 4)

This formulation assumes Cycle 4 refines earlier iterations. It adds latency-compensated feedforward, base-motion rejection, jerk-bounded rate shaping, and anti-windup that accounts for the limiter.

---

## 1. System Definition

**Axes:** pan (yaw, ψ) and tilt (pitch, θ). Each axis has its own controller. The two axes are coupled only through kinematics (§2.3).

**Control architecture (cascaded):**

```
Vision ──► Angular error e ──► [Outer loop: PID + FF] ──► ω_cmd
                                        │
                              [Accel/Jerk limiter]
                                        │
                                   ω_cmd,lim ──► [Inner rate loop: motor driver / FOC + gyro] ──► gimbal
```

The outer loop runs at the camera rate (30–120 Hz). The inner rate loop runs at 500 Hz–2 kHz on gyro feedback. In this design the outer loop commands **angular rate**, not position. Commanding rate makes feedforward and acceleration limiting natural.

---

## 2. Measurement Model

### 2.1 Pixel to angular error
Let the target centroid be (u, v), the principal point (c_x, c_y), and the focal lengths (f_x, f_y) in pixels:

$$
e_\psi = \arctan\!\left(\frac{u - c_x}{f_x}\right), \qquad
e_\theta = -\arctan\!\left(\frac{v - c_y}{f_y}\right)
$$

Use the arctangent rather than the small-angle approximation. This keeps gain consistent across a wide FOV and through zoom changes. Re-read f_x and f_y from the lens's zoom state every frame.

### 2.2 Latency compensation
A frame captured at time t_k − τ arrives at t_k, where τ is exposure, ISP, inference, and transport delay (typically 30–120 ms). During that interval the gimbal moved by:

$$
\Delta\phi_{gim} = \int_{t_k-\tau}^{t_k} \omega_{gyro}\,dt
$$

The corrected error, expressed in the current gimbal pose, is:

$$
e_k^{c} = e_k^{raw} - \Delta\phi_{gim}
$$

Implement this with a timestamped ring buffer of gyro samples. This single step usually removes most tracking oscillation.

### 2.3 Tilt–pan coupling
At high tilt, a yaw rate produces reduced image-plane motion. To compensate:

$$
\omega_{\psi,cmd} \leftarrow \frac{\omega_{\psi,cmd}}{\max(\cos\theta,\ \cos\theta_{min})}
$$

Clamp θ_min near 80° to avoid the singularity at gimbal lock.

---

## 3. Target State Estimation (Feedforward Source)

Raw differentiated error is too noisy for feedforward. Instead, estimate the target's **inertial** line-of-sight (LOS) angle and rate.

**Absolute LOS measurement:**
$$
\lambda_k = \phi_{gim,k} + e_k^{c}
$$
Here φ_gim is the gimbal's inertial pointing angle, taken from the AHRS combined with joint encoders.

**Constant-velocity Kalman filter per axis.** State x = [λ, λ̇]ᵀ:

$$
F=\begin{bmatrix}1 & T\\0 & 1\end{bmatrix},\quad
Q = q\begin{bmatrix}T^3/3 & T^2/2\\ T^2/2 & T\end{bmatrix},\quad
H=[1\ \ 0],\quad R=\sigma_{pix}^2/f^2
$$

- Tune q to the expected target maneuverability.
- Scale R with detector confidence. Inflate R or skip the update when confidence is low.
- **Lost-target handling:** if the target is missed, predict only for up to N_coast frames. After that, decay λ̂̇ to zero with time constant τ_decay.

A simpler alternative is an α-β filter with α ≈ 0.5 and β ≈ 0.1. It is acceptable on low-compute hardware.

---

## 4. Control Law

### 4.1 Rate command
For each axis, at outer-loop step k:

$$
\omega_{cmd,k} = \underbrace{K_{ff}\,\hat{\dot\lambda}_k}_{\text{target FF}}
\;+\; \underbrace{K_p\, e_k^{c} + K_i\, I_k + K_d\, D_k}_{\text{PID on residual}}
$$

- K_ff ≈ 1.0 nominally. Use 0.8–0.95 if the estimate is noisy.
- Angular error is already the tracking residual, so no setpoint weighting is needed.
- If the inner loop stabilizes against the inertial gyro, base motion is rejected automatically. Otherwise add a base-rate term $-\omega_{base}$ from the vehicle IMU.

### 4.2 Derivative term (filtered, on measurement)
Take the derivative of the latency-corrected error, filtered with a first-order low-pass (N = 5–15):

$$
D_k = \frac{\tau_d}{\tau_d+T}D_{k-1} + \frac{1}{\tau_d+T}\left(e_k^{c}-e_{k-1}^{c}\right),\quad \tau_d = \frac{K_d}{K_p N}
$$

With feedforward carrying target motion, K_d is often small or zero. Its role is to damp the residual.

### 4.3 Deadband for visual smoothness
Use a smooth (C¹) deadband so the camera doesn't hunt on detection jitter:

$$
e^{db} = \begin{cases}0 & |e| < \delta\\ \operatorname{sign}(e)\,\dfrac{(|e|-\delta)^2}{2\delta'} & \delta \le |e| < \delta+\delta' \\ e - \operatorname{sign}(e)(\delta + \delta'/2) & \text{otherwise}\end{cases}
$$

Set δ to about 1–2× the detector's pixel noise, converted to angle. Apply the deadband to the P and I paths only, never to the feedforward.

---

## 5. Acceleration and Jerk Limiting

Constrain the commanded rate trajectory so the footage looks cinematic and the motors don't saturate.

**Acceleration limit.** Default a_max is 60–200 °/s² (lower for film, higher for sport):
$$
\Delta\omega_{req} = \omega_{cmd,k} - \omega_{lim,k-1},\qquad
\Delta\omega = \operatorname{clamp}\!\left(\Delta\omega_{req},\ -a_{max}T,\ a_{max}T\right)
$$

**Jerk limit (smoothness).** Track the limiter's own acceleration state α:
$$
\alpha_k = \operatorname{clamp}\!\left(\frac{\Delta\omega}{T},\ \alpha_{k-1} - j_{max}T,\ \alpha_{k-1} + j_{max}T\right)
$$
$$
\omega_{lim,k} = \operatorname{clamp}\!\left(\omega_{lim,k-1} + \alpha_k T,\ -\omega_{max},\ \omega_{max}\right)
$$

**Overshoot-free braking.** A naive jerk limiter overshoots, so check whether it is time to start decelerating. Begin ramping α toward zero when:

$$
\frac{\alpha_k^2}{2 j_{max}} \ge |\omega_{cmd,k} - \omega_{lim,k}|
$$

This is the standard S-curve / second-order rate-limiter condition.

**Upsampling.** Run the limiter at the inner-loop rate, holding ω_cmd between vision frames. The motor then receives a smooth ramp instead of staircase steps at frame rate.

---

## 6. Anti-Windup Coupled to the Limiter

Because the limiter can saturate the command, the integrator must know about it. Use back-calculation:

$$
I_{k} = I_{k-1} + T\left[e_k^{db} + \frac{1}{T_t}\cdot\frac{\omega_{lim,k} - \omega_{cmd,k}}{K_i}\right], \qquad T_t \approx \sqrt{T_i T_d}\ \text{(or } T_i\text{ if } K_d=0)
$$

Add conditional integration as well:

- Freeze I while the target is lost.
- Freeze I while |e| exceeds a large-error threshold e_acq. During acquisition, let P + FF + limiter do the slewing.
- Clamp |K_i I| ≤ ω_I,max.

---

## 7. Mode Logic

| Mode | Condition | Behavior |
|---|---|---|
| **Acquire** | \|e\| > e_acq | P + FF only, higher a_max, integrator frozen |
| **Track** | target valid, \|e\| ≤ e_acq | Full PID + FF, cinematic a_max / j_max |
| **Coast** | target lost < N_coast frames | FF from KF prediction, P on predicted error, I frozen |
| **Hold/Recenter** | lost > N_coast | λ̂̇ decays to 0; optionally return slowly to home pose |

Use hysteresis on e_acq (for example, enter at 8°, exit at 5°) so the controller doesn't chatter between modes.

---

## 8. Discrete Implementation (per axis)

```python
def outer_step(meas, gyro_buf, st, p, T):
    # --- measurement ---
    if meas.valid:
        e_raw = atan((meas.u - p.cx) / p.fx)            # axis-specific
        e_c   = e_raw - gyro_buf.integrate(meas.t_capture, now())
        lam   = st.phi_gim + e_c
        st.kf.predict(T); st.kf.update(lam, R=p.R / meas.conf)
        st.lost = 0
    else:
        st.kf.predict(T); st.lost += 1
        e_c = st.kf.lam - st.phi_gim                    # predicted error
        if st.lost > p.N_coast:
            st.kf.lam_dot *= exp(-T / p.tau_decay)

    # --- mode ---
    acquiring = abs(e_c) > (p.e_acq_exit if st.acq else p.e_acq_enter)
    st.acq = acquiring
    e_db = smooth_deadband(e_c, p.delta, p.delta2)

    # --- PID + FF ---
    st.D = (p.tau_d*st.D + (e_c - st.e_prev)) / (p.tau_d + T)
    st.e_prev = e_c
    ff  = p.Kff * st.kf.lam_dot
    pid = p.Kp*e_db + p.Ki*st.I + p.Kd*st.D
    w_cmd = (ff + pid) / max(cos(st.tilt), cos(p.tilt_max)) if axis == PAN else ff + pid
    st.w_cmd = w_cmd                                     # consumed by fast loop

    # --- anti-windup (uses limiter output from fast loop) ---
    if meas.valid and not acquiring:
        st.I += T * (e_db + (st.w_lim - w_cmd) / (p.Tt * p.Ki))
        st.I = clamp(st.I, -p.I_max, p.I_max)

def fast_step(st, p, Ts, a_max, j_max):                  # 500 Hz – 2 kHz
    err = st.w_cmd - st.w_lim
    # S-curve braking condition
    if st.alpha**2 / (2*j_max) >= abs(err) and sign(st.alpha) == sign(err):
        a_target = 0.0
    else:
        a_target = clamp(err / Ts, -a_max, a_max)
    st.alpha = clamp(a_target, st.alpha - j_max*Ts, st.alpha + j_max*Ts)
    st.w_lim = clamp(st.w_lim + st.alpha*Ts, -p.w_max, p.w_max)
    return st.w_lim                                      # → inner gyro rate loop
```

---

## 9. Tuning Procedure

1. **Inner rate loop first.** Aim for a bandwidth of about 30–60 Hz, at least 5× the outer-loop bandwidth. Verify with a step in ω_ref.
2. **Measure latency τ.** Flash an LED in view, or correlate a gyro step against image shift. Configure the gyro buffer to match.
3. **P only, FF off, limiter wide.** Raise K_p until the response is slightly underdamped, then back off 30%. A typical K_p is 2–6 s⁻¹, giving outer bandwidth ≈ K_p / 2π.
4. **Enable FF.** Track a constant-velocity target (a turntable or a walking person). Raise K_ff toward 1 until steady-state lag is near zero. Tune KF q so the rate estimate converges within about 0.3–0.5 s.
5. **Add I.** Use K_i ≈ K_p² / 8 to remove residual bias such as calibration offsets or imperfect FF. Confirm there is no overshoot after reacquisition.
6. **Add D only if needed** to damp the residual. Keep N low because vision noise is high.
7. **Set the limiter by aesthetics.** Lower a_max and j_max until motion looks smooth, then verify the tracking error stays inside the frame-safe region for your worst-case target dynamics. A stability-safe guideline: a_max ≥ K_p · ω_target,max.
8. **Validate edge cases:** occlusion and coast, target crossing the tilt singularity, zoom changes, and vehicle vibration.

---

## 10. Performance Checks (Cycle 4 acceptance targets)

| Metric | Target |
|---|---|
| Steady-state lag at constant target rate | < 0.5° (FF-dominated) |
| Settling after reacquisition | < 1.0 s, overshoot < 10% |
| RMS jitter, stationary target | < 1.5× detector noise (deadband effective) |
| Peak commanded accel / jerk | ≤ a_max / j_max (by construction) |
| Integrator windup on 2 s occlusion | None (I frozen, smooth resume) |

**Cycle 4 deltas vs. a basic PID tracker:** the main gains come from gyro-based latency compensation, KF-derived inertial rate feedforward, a jerk-limited S-curve shaper run at the inner-loop rate, back-calculation anti-windup tied to that shaper, and mode logic with hysteresis.