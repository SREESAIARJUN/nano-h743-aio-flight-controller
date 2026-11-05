# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 27)

I don't have the Cycle 26 data for your project, so this is a general procedure with placeholders where your measured values go. It assumes a motorized gimbal in which the camera cradle rotates about a single pitch axis, driven by a motor that also holds the cradle against gravity.

---

## 1. Frames and Definitions

**Cradle body frame {B}**
- Origin **O** lies on the pitch axis, at the intersection with the roll/yaw axes if they intersect.
- **ŷ** is the pitch rotation axis.
- **x̂** points along the optical axis at θ = 0.
- **ẑ** completes the right-handed set.

**Payload quantities** (camera, lens, and cradle together)
- Total mass **M**.
- Center of mass **r_c = (x_c, y_c, z_c)** in {B}.
- First moments **S = M·r_c**.

**Inertia tensor about O**

$$
\mathbf I_O=\begin{bmatrix} I_{xx} & I_{xy} & I_{xz}\\ I_{xy} & I_{yy} & I_{yz}\\ I_{xz} & I_{yz} & I_{zz}\end{bmatrix},\qquad I_{xy}=-\!\int xy\,dm \;\text{(tensor convention)}
$$

**Parallel-axis relation**

$$
\mathbf I_O=\mathbf I_C+M\left(|\mathbf r_c|^2\mathbf E-\mathbf r_c\mathbf r_c^{T}\right)
$$

---

## 2. Balance Objectives

| Objective | Condition | Why it matters |
|---|---|---|
| **Static balance (pitch)** | x_c = z_c = 0 | Removes the gravity torque on the pitch motor at every angle. |
| **Static balance (outer axes)** | y_c chosen so the combined CoM lies on the roll and yaw axes | Prevents pitch angle from changing the loads on the roll and yaw axes. |
| **Dynamic balance** | I_xy = I_yz = 0, so ŷ is a principal axis | Pitch rotation then produces no reaction torques on roll or yaw. |
| **Inertia tuning** | I_yy lies within [I_min, I_max] | Sets acceleration headroom, control bandwidth, and resonance placement. |

**Why the products of inertia matter.** For pure pitch motion (**ω** = ω ŷ, **α** = α ŷ), Euler's equation **τ** = **I α** + **ω** × (**I ω**) gives

$$
\tau_x = I_{xy}\,\alpha + I_{yz}\,\omega^2,\qquad
\tau_y = I_{yy}\,\alpha,\qquad
\tau_z = I_{yz}\,\alpha - I_{xy}\,\omega^2
$$

Nonzero products of inertia therefore inject disturbances into the roll and yaw loops that grow with pitch acceleration and with ω².

---

## 3. Procedure

### Step 0: Pre-cycle baseline
1. Fix the payload configuration: camera body, lens, focal position, filters, cables, and battery. Record every item in the Cycle 27 manifest.
2. Compute the CAD estimates of **r_c** and **I_O** as the predicted baseline.
3. Lock the outer axes (roll and yaw held by their motors or mechanically pinned).
4. Run the system in pure torque mode, or with very low PID gains, for all measurements.

### Step 1: Coarse static balance (manual)
1. Power off the pitch motor. Rotate the cradle to θ = 0°, then 90°, and release it at each position.
2. At θ = 0 (lens horizontal), slide the camera fore and aft (x) until the cradle shows no rotation tendency.
3. At θ = 90° (lens pointing up), adjust the cradle height or riser (z) until the cradle shows no tendency.
4. Check at ±45°. The cradle should be neutrally stable at all angles.

### Step 2: Fine static balance by current-based identification
1. Sweep θ slowly across the full range, in both directions, at a constant low speed (for example 5°/s).
2. Record the motor current and convert it to torque with **τ = K_t · i**.
3. Fit the torque model by least squares:

$$
\tau(\theta) = a\cos\theta + b\sin\theta + \tau_f\,\mathrm{sgn}(\dot\theta) + c
$$

   with
   - a = M g x_c,
   - b = M g z_c (signs depend on your conventions),
   - τ_f the Coulomb friction term,
   - c the cable spring bias.

   Averaging the forward and reverse sweeps cancels τ_f.
4. Recover the CoM offsets:

$$
x_c = \frac{a}{Mg},\qquad z_c = \frac{b}{Mg}
$$

5. Correct the offsets with slide adjustments or trim masses (see Step 4).
6. Iterate until the residual gravity torque falls below the acceptance criterion.

### Step 3: Inertia and product identification

**I_yy: compound pendulum method**
1. Attach a known test mass m_t at a known distance l from the axis, along −z.
2. Measure the small-angle oscillation period T with the motor unpowered.
3. Compute:

$$
I_{yy}=\frac{m_t g l\,T^2}{4\pi^2}-m_t l^2
$$

   Correct for friction damping using the log decrement if it is significant.

**I_yy: alternative active method**
1. Command a chirp or sinusoidal torque on the pitch axis.
2. Fit the frequency response:

$$
\frac{\theta}{\tau}(s) = \frac{1}{I_{yy}s^2 + b s + k}
$$

**Products I_xy and I_yz**
1. Hold roll and yaw with stiff position loops and log their motor torques.
2. Command aggressive pitch profiles: a trapezoidal velocity profile for an α-dominant regime, plus a constant high-ω spin if mechanically allowed.
3. Regress the measured reaction torques on the model from Section 2:

$$
\begin{bmatrix}\tau_x\\ \tau_z\end{bmatrix}=\begin{bmatrix}\alpha & \omega^2\\ -\omega^2 & \alpha\end{bmatrix}\begin{bmatrix}I_{xy}\\ I_{yz}\end{bmatrix}
$$

4. Subtract the static-gravity components from τ_x and τ_z before fitting.

### Step 4: Trim mass solution (two-plane balancing)

**Why two planes.** A single trim mass can cancel the first moments, but not the products of inertia. The products require trim masses separated along ŷ.

**Setup**
- Place trim planes at y = +d (plane A) and y = −d (plane B).
- Define the trim moments u = m·x and w = m·z in each plane.
- Let D_xy = ∫xy dm = −I_xy and D_yz = ∫yz dm = −I_yz, both measured about O.

**Requirements**
1. First moments cancelled: u_A + u_B = −M x_c
2. Product cancelled: d(u_A − u_B) = −D_xy

**Solution**

$$
u_A=\tfrac12\!\left(-Mx_c-\frac{D_{xy}}{d}\right),\quad
u_B=\tfrac12\!\left(-Mx_c+\frac{D_{xy}}{d}\right)
$$

$$
w_A=\tfrac12\!\left(-Mz_c-\frac{D_{yz}}{d}\right),\quad
w_B=\tfrac12\!\left(-Mz_c+\frac{D_{yz}}{d}\right)
$$

**Converting moments to masses and positions.** In each plane:

$$
m = \frac{\sqrt{u^2+w^2}}{\rho},\qquad \phi = \operatorname{atan2}(w,u)
$$

where ρ is the available mounting radius. Use the largest feasible d and ρ to minimize the added mass.

**With discrete mounting slots,** solve the constrained least-squares problem instead:

$$
\min_{m_k\ge 0}\;\sum_k m_k \quad\text{s.t.}\quad \|\mathbf A\mathbf m+\mathbf b\|\le\epsilon
$$

- The rows of **A** are the contributions of each slot to S_x, S_z, D_xy, and D_yz.
- **b** is the current imbalance vector.

### Step 5: I_yy tuning
1. Compute the effect of the trim masses on pitch inertia:

$$
\Delta I_{yy}=\sum_k m_k\,(x_k^2+z_k^2)
$$

2. Check the result against the upper bound I_max, set by acceleration headroom:

$$
I_{yy} \le \frac{\tau_{\text{cont}}}{\alpha_{\text{req}}\cdot SF}
$$

3. Check it against the lower bound I_min, if one is specified. Some designs add inertia deliberately to lower the rigid-body/structural mode ratio or to improve disturbance rejection.
4. If I_yy falls outside the window, move the trim masses to a different radius:
   - To lower I_yy without changing the balance, use a smaller radius and larger mass. This keeps m·ρ constant while ΔI scales as m·ρ².
   - To raise I_yy, do the reverse.
5. Re-derive the control gains, which scale with inertia:

$$
K_p \propto I_{yy}\,\omega_{bw}^2,\qquad K_d \propto I_{yy}\,\omega_{bw}
$$

### Step 6: Outer-axis CoM alignment
1. Adjust y_c, using the cradle's lateral slide, so the combined CoM lies on the roll axis.
2. Verify with the roll-axis current sweep method from Step 2.
3. Re-check the pitch balance afterward. Lateral moves change D_xy and D_yz if the payload is asymmetric.

### Step 7: Verification
1. Repeat the Step 2 sweep and the Step 3 product test with the final trim installed.
2. Run a full-envelope maneuver test and confirm that the roll and yaw disturbance torques are within spec.

---

## 4. Acceptance Criteria (adjust to your spec)

| Parameter | Target |
|---|---|
| Residual gravity torque, M g √(x_c² + z_c²) | ≤ 2–5 % of pitch motor continuous torque |
| CoM offset \|x_c\|, \|z_c\| | ≤ 0.5 mm (typical for cinema-class payloads) |
| Product-induced reaction at α_max | \|I_xy\|·α_max and \|I_yz\|·α_max ≤ 5 % of the roll/yaw torque budget |
| I_yy | Within [I_min, I_max] |
| Added trim mass | ≤ X g (payload budget) |
| Sweep repeatability (fit residual RMS) | < 10 % of residual torque |

---

## 5. Cycle 27 Record Template

```
CYCLE 27 – PITCH CRADLE BALANCE LOG
Payload config ID: ______   M = ____ kg   K_t = ____ N·m/A
Pre-trim:   x_c=____ mm  z_c=____ mm  y_c=____ mm
            I_yy=____ kg·m²  I_xy=____  I_yz=____
Trim planes: d=____ mm  ρ=____ mm
Solution:   Plane A: m=____ g @ φ=____°   Plane B: m=____ g @ φ=____°
Post-trim:  x_c=____  z_c=____