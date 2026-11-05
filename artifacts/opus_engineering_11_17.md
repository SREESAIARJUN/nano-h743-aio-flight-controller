# Cycle 11: 3‑Axis Rigid‑Body Simulator with Reaction‑Wheel Torque Curves and Gyro Drift

This is a single self‑contained file. It needs only `numpy`; `matplotlib` is optional and used for plots. It models:

- **Rigid‑body dynamics**
  - Full 6‑DOF motion with a non‑diagonal inertia tensor.
  - Euler's equations including gyroscopic coupling with the wheels.
  - Quaternion kinematics.
  - RK4 integration.
- **Reaction‑wheel motors**
  - Brushless DC torque–speed curve from back‑EMF, bus voltage and winding resistance.
  - Current limit and first‑order torque lag.
  - Coulomb and viscous friction.
  - Arbitrary wheel axis geometry: 3 orthogonal wheels or a 4‑wheel pyramid.
- **Gyro error model**
  - Turn‑on bias.
  - Angle random walk (ARW).
  - Bias instability, modelled as a Gauss–Markov process.
  - Rate random walk.
  - Temperature sensitivity.
  - Scale‑factor error and axis misalignment.
  - Quantization and saturation.
- **Attitude estimation and control**
  - Attitude propagated by integrating the gyro.
  - Optional star‑tracker correction with bias estimation.
  - Quaternion PD controller with gyroscopic feed‑forward.
- **Verification**
  - Inertial angular‑momentum conservation check.
  - Energy check.
  - Dzhanibekov (intermediate‑axis) instability demo.

```python
#!/usr/bin/env python3
"""
rigidsim.py  --  Cycle 11
3-axis rigid-body physics simulator with reaction-wheel motor torque curves
and a high-fidelity gyro drift model.

Modes:
    python rigidsim.py slew          # 60-deg slew using reaction wheels + gyro/star-tracker estimator
    python rigidsim.py drift         # gyro-only attitude propagation drift study (1 hour)
    python rigidsim.py dzhanibekov   # torque-free intermediate-axis instability, conservation check
    add --noplot to skip plotting, --seed N for reproducibility

Conventions:
    Quaternion q = [w, x, y, z], rotates body-frame vectors into the inertial frame.
    omega = body angular rate expressed in body frame [rad/s].
    h     = wheel angular momentum RELATIVE to body, along each wheel axis [N m s].
    I     = total spacecraft inertia (including wheels) about CoM, body frame.
"""

from __future__ import annotations
import argparse
import math
from dataclasses import dataclass, field
import numpy as np

# =============================================================================
# Quaternion utilities
# =============================================================================

def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])

def qconj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])

def qnorm(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)

def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2)
    return np.array([math.cos(angle / 2), *(axis * s)])

def q_exp(rotvec: np.ndar