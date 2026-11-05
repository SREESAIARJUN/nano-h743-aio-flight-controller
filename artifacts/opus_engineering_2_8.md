# 3-Axis Rigid Body Physics Simulator (Cycle 2)

This is a self-contained Python simulator for a spacecraft-style rigid body controlled by three reaction wheels. It needs only NumPy. Matplotlib is optional and used for plots.

**What it models**

| Subsystem | Model |
|---|---|
| Rigid body | 6-DOF state: position, velocity, quaternion attitude, body rates. Full inertia tensor, Euler's equations with the gyroscopic term `ω × (Iω + h_w)`, and RK4 integration. |
| Reaction wheels | One motor per body axis. Four-quadrant torque-speed envelope (current limit plus back-EMF limit), regenerative braking, first-order torque lag, Coulomb and viscous friction, and a wheel speed limit. |
| Gyro | Angle random walk (ARW), rate random walk (RRW), Gauss-Markov bias instability, turn-on bias, scale-factor error, axis misalignment, temperature drift with warm-up, quantization, saturation, and a separate sample rate. |
| GNC | Attitude estimated by integrating the gyro only (dead reckoning), so drift is directly visible. A PD controller acts on the estimate, and the estimate is compared with the true attitude. |
| Disturbances | Gravity-gradient torque, constant bias torque, and random torque noise. |

---

## `rigid_sim.py`

```python
#!/usr/bin/env python3
"""
rigid_sim.py  --  3-Axis Rigid Body Simulator (Cycle 2)
=======================================================
Models:
  * 6-DOF rigid body dynamics (quaternion attitude, full inertia tensor)
  * 3 orthogonal reaction wheels with DC/BLDC motor torque-speed curves
  * MEMS/FOG-style gyro error model (ARW, RRW, bias instability, scale
    factor, misalignment, temperature drift, quantization, saturation)
  * Gyro-only attitude dead reckoning + PD attitude controller

Conventions:
  * Quaternions are scalar-first [w, x, y, z] and rotate body -> inertial.
  * Angular rates are in body frame, rad/s.
  * Wheel spin axes are aligned with body X, Y, Z.

Usage:
  python rigid_sim.py                   # default run, plots if matplotlib present
  python rigid_sim.py --t 600 --seed 3 --csv out.csv --no-plot
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass, field

import numpy as np

# =============================================================================
# Quaternion utilities
# =============================================================================

def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a (x) b."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_norm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle / 2)
    return np.array([math.cos(angle / 2), *(axis * s)])


def q_from_euler(roll, pitch, yaw) -> np.ndarray:
    """ZYX (yaw-pitch-roll) Euler angles -> quaternion."""
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


def q_to_euler(q: np.ndarray) -> np.ndarray:
    """Quaternion -> ZYX Euler [roll, pitch, yaw] (rad)."""
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sp = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Body -> inertial rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def q_exp_rate(omega: np.ndarray, dt: float) -> np.ndarray:
    """Exact quaternion increment for constant body rate over dt."""
    theta = np.linalg.norm(omega) * dt
    if theta < 1e-12:
        return q_norm(np.array([1.0, *(0.5 * omega * dt)]))
    axis = omega / np.linalg.norm(omega)
    return np.array([math.cos(theta/2), *(axis * math.sin(theta/2))])


def q_angle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(q_norm(a), q_norm(b))))
    return 2.0 * math.acos(min(1.0, d))


# =============================================================================
# Motor / reaction wheel model
# =============================================================================

@dataclass
class MotorParams:
    Kt: float = 0.025         # torque constant  [N*m/A]
    Ke: float = 0.025         # back-EMF const   [V*s/rad] (=Kt in SI)
    R: float = 1.2            # winding resistance [ohm]
    V_bus: float = 28.0       # supply voltage [V]
    I_max: float = 4.0        # driver current limit [A]
    tau_e: float = 0.02       # torque loop time constant [s]
    J_wheel: float = 2.0e-3   # wheel rotor inertia [kg*m^2]
    coulomb: float = 2.0e-4   # Coulomb friction [N*m]
    viscous: float = 1.0e-6   # viscous friction [N*m*s/rad]
    omega_max: float = 6000 * 2*math.pi/60   # speed limit [rad/s]
    regen_efficiency: float = 1.0            # braking current headroom factor

    @property
    def T_peak(self) -> float:
        return self.Kt * self.I_max

    @property
    def omega_no_load(self) -> float:
        return self.V_bus / self.Ke


class ReactionWheelMotor:
    """
    Four-quadrant torque-speed envelope.

    Motoring (torque in the direction of spin):
        T_max(w) = min(Kt*I_max, Kt*(V - Ke|w|)/R)    -> linear droop to no-load speed
    Braking / plugging (torque opposite to spin):
        T_max(w) = min(Kt*I_max, Kt*(V + Ke|w|)/R * eff)
    At |w| >= omega_max, further acceleration is inhibited.
    """

    def __init__(self, p: MotorParams):
        self.p = p

    def torque_limit(self, T_cmd: float, w: float) -> float:
        p = self.p
        aw = abs(w)
        motoring = (T_cmd * w) >= 0.0
        if motoring:
            t_emf = p.Kt * max(0.0, p.V_bus - p.Ke * aw) / p.R
            t_lim = min(p.T_peak, t_emf)
            if aw >= p.omega_max:
                t_lim = 0.0
        else:
            t_emf = p.Kt * (p.V_bus + p.Ke * aw) / p.R * p.regen_efficiency
            t_lim = min(p.T_peak, t_emf)
        return float(np.clip(T_cmd, -t_lim, t_lim))

    def friction(self, w: float) -> float:
        p = self.p
        return p.coulomb * math.tanh(w / 0.5) + p.viscous * w

    def current(self, T: float) -> float:
        return T / self.p.Kt

    def electrical_power(self, T: float, w: float) -> float:
        i = self.current(T)
        return i * i * self.p.R + T * w   # copper loss + mechanical power

    def curve(self, n: int = 200):
        """Return (speed_rpm, max_motoring_torque) for plotting."""
        w = np.linspace(0, self.p.omega_no_load, n)
        T = [self.torque_limit(1e9, wi) for wi in w]
        return w * 60 / (2*math.pi), np.array(T)


# =============================================================================
# Gyro error model
# =============================================================================

DEG = math.pi / 180.0


@dataclass
class GyroParams:
    rate_hz: float = 200.0
    arw_deg_rthr: float = 0.15        # angle random walk  [deg/sqrt(hr)]
    rrw_deg_hr_rthr: float = 2.0      # rate random walk   [deg/hr/sqrt(hr)]
    bias_instab_deg_hr: float = 3.0   # Gauss-Markov bias instability [deg/hr]
    bias_corr_time_s: float = 100.0   # GM correlation time
    turn_on_bias_deg_hr: float = 10.0 # 1-sigma constant bias per power-up
    scale_factor_ppm: float = 300.0   # 1-sigma scale factor error
    misalign_mrad: float = 1.0        # 1-sigma axis misalignment
    temp_coeff_deg_hr_per_C: float = 0.5
    temp_start_C: float = 20.0
    temp_final_C: float = 45.0
    temp_tau_s: float = 300.0         # warm-up time constant
    temp_ref_C: float = 25.0
    quant_rad_s: float = 1.0e-6       # LSB
    range_rad_s: float = 400 * DEG    # saturation


class Gyro:
    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.dt = 1.0 / p.rate_hz

        # Unit conversions to