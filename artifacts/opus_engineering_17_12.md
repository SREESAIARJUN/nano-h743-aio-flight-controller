# 3-Axis Rigid Body Attitude Simulator (Cycle 17)

I don't have the code from your previous cycles, so this is a **self-contained, complete implementation**. Tell me what earlier cycles defined (interfaces, parameters, conventions) and I can adapt it to match.

**What it models**
- **Rigid body:** full Euler equations with a non-diagonal inertia tensor, gyroscopic coupling, quaternion kinematics, and RK4 integration.
- **Actuators:** three reaction wheels driven by DC/BLDC motor models. Each includes back-EMF torque-speed limits, a current limit, electrical lag, Coulomb and viscous friction, and power draw.
- **Gyro errors:** angle random walk, bias instability (Gauss-Markov), rate random walk, temperature-dependent bias, scale factor, misalignment, quantization, and range saturation.
- **Estimation and control:** gyro-integrated attitude estimation with an optional star-tracker bias correction, plus a PD slew-and-hold controller.
- **Environment:** gravity gradient torque and residual disturbances.

Dependencies: `numpy` (required) and `matplotlib` (optional, for plots).

```python
#!/usr/bin/env python3
"""
attitude_sim.py  --  3-Axis Rigid Body Physics Simulator  (Cycle 17)

Models:
  * Rigid-body rotational dynamics (Euler's equations, full inertia tensor)
  * Quaternion kinematics, RK4 integration
  * Reaction wheels driven by DC motors with realistic torque-speed curves
  * MEMS/FOG-style gyro error model with drift
  * Gyro-integrated attitude estimator with optional star tracker corrections
  * PD attitude controller (slew + hold)

Usage:
  python attitude_sim.py                    # default run with star tracker
  python attitude_sim.py --no-star          # pure gyro propagation (see drift)
  python attitude_sim.py --t-end 3000 --csv out.csv --no-plot

Conventions:
  Quaternions are scalar-first [w, x, y, z]. q maps body -> inertial.
  Body rates are in rad/s and expressed in the body frame.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field

import numpy as np

DEG = math.pi / 180.0
HOUR = 3600.0


# ----------------------------------------------------------------------------
# Quaternion utilities
# ----------------------------------------------------------------------------
def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a (x) b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def qconj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qnormalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    q = q / n
    # Keep the scalar part non-negative (canonical hemisphere).
    return q if q[0] >= 0 else -q


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2.0)
    return np.array([math.cos(angle / 2.0), *(axis * s)])


def q_from_rotvec(rv: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(rv)
    if ang < 1e-12:
        return qnormalize(np.array([1.0, *(0.5 * rv)]))
    return q_from_axis_angle(rv / ang, ang)


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """DCM that rotates body-frame vectors into the inertial frame."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_angle(q: np.ndarray) -> float:
    """Rotation angle (rad) represented by a quaternion."""
    return 2.0 * math.atan2(np.linalg.norm(q[1:]), abs(q[0]))


def q_error_vec(q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
    """Small-angle rotation vector (body frame) rotating q_from onto q_to."""
    dq = qnormalize(qmul(qconj(q_from), q_to))
    return 2.0 * dq[1:]


# ----------------------------------------------------------------------------
# DC motor with torque-speed curve
# ----------------------------------------------------------------------------
@dataclass
class MotorParams:
    kt: float = 0.0255             # torque constant         [N*m/A]
    ke: float = 0.0255             # back-EMF constant       [V/(rad/s)]
    resistance: float = 1.2        # winding resistance      [ohm]
    inductance: float = 0.6e-3     # winding inductance      [H]
    v_bus: float = 24.0            # supply voltage          [V]
    i_max: float = 3.0             # driver current limit    [A]
    coulomb_friction: float = 2e-4  # [N*m]
    viscous_friction: float = 5e-7  # [N*m/(rad/s)]


class DCMotor:
    """Current-controlled DC/BLDC motor with voltage-headroom saturation.

    Available current at speed omega is limited both by the driver limit and
    by the voltage headroom (V_bus - ke*omega)/R. This produces the classic
    flat region (current limited) followed by a linear roll-off to zero at
    the no-load speed (voltage limited).
    """

    def __init__(self, p: MotorParams):
        self.p = p
        self.current = 0.0

    @property
    def no_load_speed(self) -> float:
        return self.p.v_bus / self.p.ke

    def current_limits(self, omega: float) -> tuple[float, float]:
        p = self.p
        i_hi = min(p.i_max, (p.v_bus - p.ke * omega) / p.resistance)
        i_lo = max(-p.i_max, (-p.v_bus - p.ke * omega) / p.resistance)
        return i_lo, i_hi

    def torque_limits(self, omega: float) -> tuple[float, float]:
        i_lo, i_hi = self.current_limits(omega)
        return self.p.kt * i_lo, self.p.kt * i_hi

    def friction(self, omega: float) -> float:
        p = self.p
        # Smooth sign function avoids chatter at zero speed.
        return p.coulomb_friction * math.tanh(omega / 0.5) + p.viscous_friction * omega

    def step(self, tau_cmd: float, omega: float, dt: float) -> dict:
        """Advance the motor by dt.

        Returns the electromagnetic torque, friction torque, net torque on the
        rotor, current, and electrical power drawn from the bus.
        """
        p = self.p
        i_cmd = tau_cmd / p.kt
        i_lo, i_hi = self.current_limits(omega)
        i_cmd = min(max(i_cmd, i_lo), i_hi)

        # First-order electrical lag: tau_e = L/R (exact discretization).
        tau_e = p.inductance / p.resistance
        alpha = 1.0 - math.exp(-dt / tau_e) if tau_e > 0 else 1.0
        self.current += alpha * (i_cmd - self.current)

        tau_em = p.kt * self.current
        tau_f = self.friction(omega)
        voltage = self.current * p.resistance + p.ke * omega
        return {
            "tau_em": tau_em,
            "tau_friction": tau_f,
            "tau_net": tau_em - tau_f,
            "current": self.current,
            