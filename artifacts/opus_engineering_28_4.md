# Pitch-Axis Camera Cradle: Mechanical Balancing and Inertia Tensor Tuning Procedure (Cycle 28)

> **Note:** I don't have your program's Cycle 27 results, drawings or tolerance budget. This is a complete, general procedure. Values in **[brackets]** are placeholders to replace with your project numbers.

---

## 1. Coordinate Frame and Definitions

**Frame:**
- Origin **O** is at the pitch bearing axis, midway between the bearings.
- **Y** is the pitch rotation axis.
- **X** is the optical axis (boresight) at θ = 0.
- **Z** completes a right-handed set ("up" at θ = 0).

**Cradle assembly** (camera, lens, cradle, cabling and connectors in their flight positions):
- Total mass: M
- Centre of gravity (CG) position: **r_c** = (x_c, y_c, z_c)

**Inertia tensor about O:**

$$
\mathbf{I}_O=\begin{bmatrix} I_{xx} & -J_{xy} & -J_{xz}\\ -J_{xy} & I_{yy} & -J_{yz}\\ -J_{xz} & -J_{yz} & I_{zz}\end{bmatrix},\qquad J_{ab}=\int ab\,dm
$$

**Parallel-axis relation** (from the CG tensor to O):

$$
\mathbf{I}_O=\mathbf{I}_{CG}+M\left[(\mathbf r_c\!\cdot\!\mathbf r_c)\mathbf 1-\mathbf r_c\mathbf r_c^{T}\right]
$$

---

## 2. Balance Objectives

| Condition | Requirement | Physical consequence if violated |
|---|---|---|
| **Static (pitch)** | x_c = z_c = 0 | Gravity torque τ = Mg·√(x_c²+z_c²)·cos(θ+φ). This creates a motor holding current and an angle-dependent pointing error. |
| **Static (outer axes)** | y_c = 0, or as required by roll balance | An axial CG offset unbalances the **roll** axis (and yaw when tilted). |
| **Dynamic** | J_xy = J_yz = 0 (Y is a principal axis) | A pitch acceleration α produces cross-axis reaction torques τ_x = −J_xy·α and τ_z = −J_yz·α. These couple into roll and yaw. |
| **Inertia shape** | I_yy at target; I_xx ≈ I_zz where practical | Gyroscopic and base-motion coupling scale with (I_zz − I_xx). I_yy sets torque demand and structural mode frequency. |

---

## 3. Procedure

### Step 0: Configuration Freeze
- Install the flight lens. Set focus and zoom to the **balance reference state**, normally mid-zoom. Record any zoom-dependent CG travel.
- Dress cables exactly as in flight. Cable stiffness and mass matter at this tolerance level.
- Lock the roll and yaw axes level. Set the pitch axis to free, motor unpowered or in torque mode.
- Record the configuration ID, the masses of all parts, and the ambient temperature.

### Step 1: Analytical Prediction (CAD)
1. Export M, **r_c** and **I_CG** from CAD, using measured masses to override the density model.
2. Transform the tensor to O using the parallel-axis relation.
3. Compute the predicted static and dynamic imbalance. Compare against the Cycle 27 measured values to update the CAD-to-hardware bias.

### Step 2: Coarse Static Balance
- **Fore-aft (X):** with θ = 0, slide the camera along the X rail until the cradle holds level.
- **Vertical (Z):** set θ = 90° (lens up). Adjust the Z (riser) position until the cradle holds at this angle.
- **Indifference test:** release the cradle at θ = 0°, ±45° and ±90°. It must stay put at every angle. If it drifts toward one orientation, return to the coarse adjustments.

### Step 3: Fine Static Balance by Motor-Current Measurement
1. Command a slow pitch sweep from −90° to +90° at constant low rate **[≤ 5°/s]**.
2. Record the q-axis current i_q(θ) in both directions.
3. Average the two directions to cancel friction:

$$
\tau(\theta)=K_t\,\tfrac{1}{2}(i_{q,+}+i_{q,-}) = a\cos\theta + b\sin\theta + c
$$

4. Extract the CG offsets by least-squares fit:
   - z_c = a/(Mg)
   - x_c = −b/(Mg) (check the signs against your frame)
   - c is the cable or spring bias. If it is significant, investigate the cable routing.
5. Correct with fine-thread adjusters or trim masses. Repeat until within the tolerances in §4.

### Step 4: Dynamic Balance (Products of Inertia)
1. **Measure.** Excite the pitch axis sinusoidally at **[2–10 Hz]** with known α. Use the roll and yaw motors in position hold, or a base 6-axis load cell or IMU, to measure the reaction torques. Then:
   - J_xy = −τ_x/α
   - J_yz = −τ_z/α
   - Use phase-locked demodulation at the excitation frequency.
2. **Correct with two-plane trim.** Choose trim planes at y = +b and y = −b.
   - Let the residual static vector be **U** = M(x_c, z_c) (after Step 3, ideally ≈ 0).
   - Let the couple vector be **C** = (J_xy, J_yz).
   - The trim mass moment vectors (m·r in the xz-plane) are:

$$
\mathbf u_A=-\tfrac12\left(\mathbf U+\mathbf C/b\right),\qquad \mathbf u_B=-\tfrac12\left(\mathbf U-\mathbf C/b\right)
$$

   - Place mass m_A at radius |u_A|/m_A and angle atan2 of **u_A**. Do the same for B.
   - Use the largest practical b. A larger b means smaller trim masses.
3. Re-run Step 3, since dynamic trim can perturb static balance. Then re-run Step 4.

### Step 5: Inertia Magnitude Tuning
1. **Measure I_yy.** Use either method:
   - **Pendulum:** add a known offset mass m_p at distance d and measure the period T. Then I_yy = m_p·g·d·T²/(4π²) − m_p·d².
   - **Torque-acceleration:** apply a known torque step and measure α. Then I_yy = τ/α.
2. **Check against targets:**
   - Torque margin: τ_req = I_yy·α_max + τ_friction + τ_residual-imbalance. This must be ≤ **[70%]** of continuous motor torque.
   - First structural mode: f_n = (1/2π)·√(k_θ/I_yy) ≥ **[4–5×]** the pitch control bandwidth.
   - Load/rotor inertia ratio is within the controller's tuned range **[e.g. ≤ 10:1 direct drive]**.
3. **Shape tuning:** if gyroscopic coupling from base yaw or roll rates is observed (τ_y ∝ (I_zz − I_xx)·ω_x·ω_z), relocate dense items toward I_xx ≈ I_zz:
   - Batteries and electronics go **on** the pitch axis.
   - Trim masses go symmetric about the CG.
4. **Update the outer axes:** propagate the new cradle tensor (parallel-axis to the roll axis) into the roll and yaw models. Re-verify their balance and bandwidth.

### Step 6: Verification Across the Operating Envelope
- Repeat the Step 3 sweep at zoom min and max, focus near and far, and the temperature extremes **[if required]**.
- Run a closed-loop step response and a base-disturbance rejection test. Compare these against the Cycle 27 results.

---

## 4. Acceptance Criteria (Example Values)

| Parameter | Limit |
|---|---|
| Static imbalance √(x_c²+z_c²)·M | ≤ **[0.5 g·cm]**, or holding current ≤ **[5%]** of rated |
| Axial offset y_c | ≤ **[1 mm]** (per roll budget) |
| Product of inertia \|J_xy\|, \|J_yz\| | ≤ **[1%]** of I_yy |
| Principal axis misalignment | ≤ **[1°]**, from the eigenvectors of **I_O** |
| I_yy vs target | within **[±10%]** |
| Indifference across ±90° | no drift > **[0.5°]** over 30 s, unpowered |
| Zoom-induced CG shift | within the static limit, or compensated by feed-forward |

---

## 5. Cycle 28 Record Template

| Item | Cycle 27 | Cycle 28 pre-trim | Cycle 28 post-trim | Pass/Fail |
|---|---|---|---|---|
| M (g) | | | | |
| x_c, y_c, z_c (mm) | | | | |
| J_xy, J_yz (kg·m²) | | | | |
| I_xx, I_yy, I_zz (kg·m²) | | | | |
| Holding current (A) | | | | |
| Trim masses (g @ position) | | | | |
| f_n pitch (Hz) | | | | |
| CAD-to-measured bias | | | | |

**Close-out:** feed the measured tensor back into the CAD correction factors and the controller plant model, so that Cycle 29 starts from a calibrated prediction.

---

If you share the Cycle 27 data, the camera/lens payload mass, the motor specification and the control bandwidth target, I can fill in the numeric targets and compute the specific trim mass placements.