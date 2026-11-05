# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning (Cycle 11)

I don't have your Cycle 11 design data, such as the CAD mass properties, payload list or motor specs. The procedure below is a complete, parameterized framework. Substitute your values where indicated. Assumptions are stated so you can flag any that don't match your hardware.

---

## 1. Reference Frame and Definitions

**Cradle body frame {C}:**
- Origin O at the intersection of the pitch bearing axis and the cradle mid-plane.
- **y_C** along the pitch rotation axis (bearing centerline).
- **x_C** along the optical axis (lens forward) at zero pitch.
- **z_C** completes the right-handed set (up at zero pitch).

**Rotating assembly (the "pitch body"):** cradle structure, camera body, lens, filters/matte box, cables within the service loop, the motor rotor, and the encoder disk.

**Mass properties of interest:**

$$
m,\quad \mathbf{r}_{cm} = \begin{bmatrix} x_{cm} \\ y_{cm} \\ z_{cm} \end{bmatrix},\quad
\mathbf{I}_O = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \\ -I_{xy} & I_{yy} & -I_{yz} \\ -I_{xz} & -I_{yz} & I_{zz} \end{bmatrix}
$$

---

## 2. Balancing Objectives

| Objective | Condition | Why it matters |
|---|---|---|
| **Static balance (pitch)** | $x_{cm} \to 0,\ z_{cm} \to 0$ | Removes gravity torque $\tau_g = m g (x_{cm}\cos\theta + z_{cm}\sin\theta)$ at all pitch angles θ |
| **Lateral CoM** | $y_{cm} \to 0$ (or as specified by the roll/yaw design) | Prevents the pitch assembly from imbalancing the outer (roll/yaw) axes |
| **Dynamic decoupling** | $I_{xy} \to 0,\ I_{yz} \to 0$ | Pitch rotation then generates no reaction torques about x/z, so roll/yaw are not disturbed |
| **Inertia magnitude** | $I_{yy}$ within the controller design window | Sets the plant gain, the achievable bandwidth and the torque margin |
| **Inertia isotropy (secondary)** | $I_{xx} \approx I_{zz}$ | Reduces gyroscopic/centrifugal coupling during combined-axis motion |

**Key insight:** Static balance must hold about **both** x and z. Balancing only fore-aft (x) leaves a vertical offset (z_cm) that produces a sin θ torque. This torque is invisible when the cradle is level but grows as the cradle tilts. This is the most common balancing error.

---

## 3. Analytical Pre-Balance (Design Phase)

### 3.1 Composite CoM

For N components with masses $m_i$ and CoM positions $\mathbf{r}_i$:

$$
\mathbf{r}_{cm} = \frac{\sum m_i \mathbf{r}_i}{\sum m_i}
$$

### 3.2 Composite Inertia (Parallel-Axis / Steiner)

Each component's inertia $\mathbf{I}_i^{cm}$ must be expressed in {C} axes (rotate first if necessary: $\mathbf{R}\mathbf{I}\mathbf{R}^T$). Then:

$$
\mathbf{I}_O = \sum_i \left[ \mathbf{I}_i^{cm} + m_i\left( (\mathbf{r}_i^T\mathbf{r}_i)\mathbf{E}_3 - \mathbf{r}_i\mathbf{r}_i^T \right) \right]
$$

### 3.3 Required Counterbalance

Static trim masses $m_x$ and $m_z$, placed at adjustable positions, must satisfy:

$$
m_x\, x_{t} = -m\, x_{cm}, \qquad m_z\, z_{t} = -m\, z_{cm}
$$

**Design rule:** Prefer **moving the payload** (camera on a dovetail or sliding plate) over adding counterweight. Adding mass increases $I_{yy}$ and total motor load, while moving the payload only redistributes existing mass.

### 3.4 Product-of-Inertia Cancellation

A point trim mass $m_t$ at $(x_t, y_t, z_t)$ contributes:

$$
\Delta I_{xy} = m_t x_t y_t,\quad \Delta I_{yz} = m_t y_t z_t,\quad \Delta I_{xz} = m_t x_t z_t
$$

To null $I_{xy}$ and $I_{yz}$ **without disturbing static balance**, use **mirror pairs** of equal masses placed at $(\pm a, \pm b, \cdot)$ with opposite signs, as illustrated below. A pair at $(+a,+b,0)$ and $(-a,-b,0)$ adds zero net first moment but contributes $\Delta I_{xy} = 2m_t a b$.

---

## 4. Physical Balancing Procedure

### Prerequisites
- Final payload configuration installed: camera, lens at its working focal length/focus, filters, battery, cables and any accessories.
- Pitch motor **unpowered** or in a free-wheel mode; cogging characterized (see Step 4.6).
- Outer axes locked so the pitch axis is horizontal (verify with a level to ≤0.1°).

### Step 4.1 — Lateral (y) Pre-Centering
Center the camera on the cradle using the CAD-predicted lateral offset. Check by measuring from both bearing faces to reference features on the camera. Fine lateral balance is verified later on the roll axis.

### Step 4.2 — Fore-Aft Balance (x_cm)
1. Set pitch to **θ = 0°** (lens horizontal) and release.
2. If the cradle rotates nose-down, slide the payload aft (or vice versa).
3. Iterate until the cradle stays at 0° with no drift.

### Step 4.3 — Vertical Balance (z_cm)
1. Rotate to **θ = 90°** (lens pointing straight up or down) and release.
2. If the cradle falls back toward horizontal, the CoM is offset in z. Adjust the vertical riser/plate height, or move the z-trim mass.
3. Iterate until it stays at 90°.

### Step 4.4 — Cross-Check (Neutral Test)
Place the cradle at **θ = −45°, 0°, +45°, 90°** in turn. A correctly balanced cradle stays at every angle.

If it consistently returns to one specific angle θ*, residual imbalance remains, with direction:

$$
\theta^* = \text{atan2}(z_{cm},\ x_{cm}) + \pi \quad\text{(stable equilibrium hangs below the axis)}
$$

Use this angle to determine which offset to correct next.

### Step 4.5 — Quantitative Residual Measurement
Use either method:

**(a) Motor current method (preferred):** Power the motor in position hold. Sweep θ slowly over its full range and log the holding torque $\tau(\theta) = K_t\, i_q(\theta)$. Fit the data to:

$$
\tau(\theta) = A\cos\theta + B\sin\theta + \tau_{fric}\,\text{sgn}(\dot\theta) + \tau_{cog}(\theta)
$$

The fit gives $x_{cm} = A/(mg)$ and $z_{cm} = B/(mg)$. Sweep in both directions and average them to cancel friction.

**(b) Torque gauge method:** Measure the torque needed to hold the cradle at θ = 0° and 90° using a calibrated force gauge at a known lever arm.

### Step 4.6 — Cogging and Friction Floor
Map $\tau_{cog}(\theta)$ with the payload removed, or with a balanced dummy load. Static balance below the friction/cogging floor cannot be detected by the release test. Treat this floor as the minimum achievable static balance resolution, and use the current method (4.5a) to go beyond it.

---

## 5. Inertia Tensor Measurement and Tuning

### 5.1 Measuring I_yy (About the Pitch Axis)

**Method A — Compound pendulum with known offset mass:**
1. Attach a known mass $m_p$ at distance $d$ below the axis.
2. Measure the small-angle oscillation period T.
3. Calculate:

$$
I_{yy} = \frac{m_p g d\, T^2}{4\pi^2} - m_p d^2
$$

Correct for bearing friction if decay is significant: use the damped frequency, $\omega_n = \omega_d/\sqrt{1-\zeta^2}$.

**Method B — System identification (in situ):**
1. Inject a chirp or PRBS torque command.
2. Fit the frequency response to $G(s) = \frac{1}{I_{yy}s^2 + b s}$ in the rigid-body region below the first structural mode.
3. This method also captures the rotor inertia and the effective cable stiffness.

### 5.2 Measuring Products of Inertia

Mount the pitch assembly on a **trifilar pendulum**, or a mass-properties instrument if available. Measure the moment of inertia about at least **six** axes through the CoM with known direction cosines $(\ell, m, n)$:

$$
I_{\hat{u}} = I_{xx}\ell^2 + I_{yy}m^2 + I_{zz}n^2 - 2I_{xy}\ell m - 2I_{yz}mn - 2I_{xz}\ell n
$$

Solve the resulting linear system (6 equations, 6 unknowns, or least squares with more). Typical axis choices are x, y, z, and the diagonals in the xy, yz and xz planes.

**In-situ alternative:** Spin or oscillate the pitch axis alone with the outer-axis motors in torque-sensing mode. The reaction torques measured on roll and yaw during pitch acceleration $\ddot\theta$ give:

$$
\tau_x^{react} \approx I_{xy}\,\ddot\theta, \qquad \tau_z^{react} \approx I_{yz}\,\ddot\theta
$$

This directly measures the coupling terms you care about.

### 5.3 Tuning Targets

Define these against your controller design. Placeholder criteria are shown below.

| Parameter | Target (fill in for Cycle 11) | Typical starting criterion |
|---|---|---|
| $\lvert x_{cm}\rvert,\ \lvert z_{cm}\rvert$ | ____ mm | Gravity torque ≤ 5–10% of continuous motor torque, or below the cogging floor |
| $\lvert y_{cm}\rvert$ | ____ mm | Per roll-axis balance budget |
| $I_{yy}$ | ____ kg·m² (±__%) | Within ±15% of the value used in the controller tuning |
| $\lvert I_{xy}\rvert / I_{yy}$, $\lvert I_{yz}\rvert / I_{yy}$ | ≤ ____ | ≤ 2–5% |
| First structural mode | ≥ ____ Hz | ≥ 5–10 × the position loop bandwidth |

### 5.4 Tuning Actions

1. **I_yy too high:** Move dense components (battery, accessories) closer to the pitch axis. Use a shorter lens or relocate the matte box. Replace steel counterweights with tungsten ones placed closer in.
2. **I_yy too low** (rare, e.g., light payload with an oversized motor giving excess loop gain): Retune the controller gains rather than adding mass. Add inertia only if it is needed to push a resonance or improve disturbance rejection.
3. **Product-of-inertia correction:** Apply the mirror-pair trim masses from §3.4. After each change, **re-verify static balance (Steps 4.2–4.4)**. The pairs are designed not to affect it, but placement tolerances will.
4. **Iterate:** Measure → compute residuals → trim → re-measure, until all §5.3 criteria are met.

---

## 6. Verification and Acceptance

1. **Static:** The neutral test passes at all four angles. The fitted gravity-torque amplitude $\sqrt{A^2+B^2}$ meets its target.
2. **Dynamic:** Command a pitch step or sinusoid. Roll/yaw tracking error caused by coupling stays below the spec.
3. **Thermal and power:** The steady-state pitch motor current at hold, across the full pitch range, meets the current budget.
4. **Configuration sensitivity:** Repeat the static check after changing the lens zoom/focus extremes, adding or removing filters, and rerouting cables. Record which configurations need rebalancing.
5. **Documentation:** Record the final $m$, $\mathbf{r}_{cm}$, $\mathbf{I}_O$, the trim mass positions, and the measured transfer function. Feed these values back into the controller model for Cycle 12.

---

## 7. Common Pitfalls

- Balancing only at θ = 0° and missing z_cm.
- Balancing without the final cables or accessories installed.
- Lens zoom or focus elements shifting the CoM during a shot. Balance at the working configuration, or at the midpoint of the range.
- Mistaking cogging or friction for balance during release tests.
- Adding counterweight when payload repositioning was possible, which inflates I_yy.
- Ignoring the cable service loop, which acts as a position-dependent spring and mimics imbalance.

---

If you share the Cycle 11 specifics, I can turn this framework into concrete numbers, trim-mass sizing and pass/fail criteria. The most useful details are the payload mass and CAD mass properties, the motor $K_t$ and continuous torque, the target control bandwidth, and the available trim/adjustment features.