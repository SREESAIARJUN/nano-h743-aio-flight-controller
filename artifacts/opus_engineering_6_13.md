# 3-Axis Rigid Body Simulator with Reaction Wheel Motors and Gyro Drift (Cycle 6)

The listing below is a single, self-contained file. It needs only `numpy`; plotting uses `matplotlib` if it is installed.

**What it models:**
- **Rigid body dynamics:** full Euler equations with the gyroscopic coupling term ω×(Iω + h). Attitude is a scalar-first quaternion, integrated with RK4.
- **Motor torque curves:** brushless DC reaction wheels driven by a voltage command. The model includes back-EMF, current limiting, the voltage-limited torque–speed envelope, and Coulomb plus viscous friction. Optional tabulated torque–speed curves are also supported.
- **Gyro errors:** angle random walk (ARW), rate random walk (RRW), Gauss–Markov bias instability, temperature-dependent bias, scale factor, axis misalignment, quantization and saturation.
- **Estimation and control:** a gyro-propagated attitude estimator with optional star-tracker correction (Mahony-style bias estimation), plus a quaternion PD controller with gyroscopic feed-forward.
- **Verification:** a torque-free conservation self-test that runs an intermediate-axis (Dzhanibekov) tumble and checks energy and momentum drift.

```python
#!/usr/bin/env python3
"""
rigidsim6.py — 3-axis rigid body attitude simulator (Cycle 6)

  * Rigid body with full inertia tensor, quaternion kinematics, RK4 integration
  * 3 orthogonal reaction wheels driven by BLDC motor models
      - back-EMF, current limit, supply-voltage limit, Coulomb + viscous friction
      - optional tabulated torque-speed curve
  * MEMS/FOG-style gyro model
      - ARW, RRW, Gauss-Markov bias instability, temperature bias,
        scale factor, misalignment, quantization, saturation
  * Attitude estimator (gyro propagation + optional star tracker, bias estimation)
  * Quaternion PD controller with gyroscopic feed-forward
  * Self-test: torque-free conservation of energy and angular momentum

Usage:
  python rigidsim6.py                    # closed-loop slew, star tracker ON
  python rigidsim6.py --no-startracker   # watch gyro drift corrupt the estimate
  python rigidsim6.py --selftest         # conservation check
  python rigidsim6.py --tf 600 --seed 3 --no-plot
"""
from __future__ import annotations
import argparse
import math
from dataclasses import dataclass, field
import numpy as np

# =============================================================================
# Quaternion utilities (scalar-first [w, x, y, z], body -> inertial)
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

def qexp(rotvec: np.ndarray) -> np.ndarray:
    """Quaternion for a rotation vector (axis * angle, rad)."""
    ang = np.linalg.norm(rotvec)
    if ang < 1e-12:
        return qnorm(np.array([1.0, *(0.5 * rotvec)]))
    ax = rotvec / ang
    return np.array([math.cos(ang / 2), *(math.sin(ang / 2) * ax)])

def qfrom_axis_angle(axis, angle_rad) -> np.ndarray:
    axis = np.asarray(axis, float)
    return qexp(axis / np.linalg.norm(axis) * angle_rad)

def qerr_vec(q_ref: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Small-angle rotation vector from q_ref to q (shortest path), rad."""
    dq = qmul(qconj(q_ref), q)
    if dq[0] < 0:
        dq = -dq
    return 2.0 * dq[1:]

def qangle(q_a: np.ndarray, q_b: np.ndarray) -> float:
    dq = qmul(qconj(q_a), q_b)
    return 2.0 * math.acos(min(1.0, abs(dq[0])))

def qto_dcm(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])

def qto_euler_zyx(q: np.ndarray) -> np.ndarray:
    """Returns roll, pitch, yaw (rad)."""
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])

# =============================================================================
# Motor models
# =============================================================================

@dataclass
class BLDCMotor:
    """Voltage-driven BLDC motor with back-EMF and current limit.

    Electrical time constant is neglected (L/R << control period).
    Torque-speed envelope: |tau| <= Kt * min(I_max, (V_max - Ke*|w|)/R)
    """
    Kt: float = 0.030        # N*m/A
    Ke: float = 0.030        # V/(rad/s)  (SI: Ke == Kt)
    R: float = 1.5           # ohm
    V_max: float = 24.0      # V supply
    I_max: float = 3.0       # A current limit
    tau_coulomb: float = 2e-4    # N*m
    b_viscous: float = 1.5e-6    # N*m/(rad/s)
    w_smooth: float = 0.5        # rad/s, tanh smoothing for Coulomb friction

    def current(self, V: float, w: float) -> float:
        V = max(-self.V_max, min(self.V_max, V))
        i = (V - self.Ke * w) / self.R
        return max(-self.I_max, min(self.I_max, i))

    def friction(self, w: float) -> float:
        return self.tau_coulomb * math.tanh(w / self.w_smooth) + self.b_viscous * w

    def shaft_torque(self, V: float, w: float) -> float:
        """Net torque on the rotor (electromagnetic minus friction)."""
        return self.Kt * self.current(V, w) - self.friction(w)

    def voltage_for_torque(self, tau_cmd: float, w: float) -> float:
        """Inverse model: voltage to obtain tau_cmd at speed w (with friction comp)."""
        i = (tau_cmd + self.friction(w)) / self.Kt
        i = max(-self.I_max, min(self.I_max, i))
        V = i * self.R + self.Ke * w
        return max(-self.V_max, min(self.V_max, V))

    def max_torque_at(self, w: float, direction: float = 1.0) -> float:
        V = self.V_max * math.copysign(