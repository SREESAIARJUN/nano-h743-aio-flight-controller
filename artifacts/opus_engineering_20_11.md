# 3-Axis Rigid-Body Attitude Simulator with Reaction-Wheel Motor Curves and Gyro Drift

I don't have context for earlier "cycles," so this is a complete, standalone simulator (v20) you can run directly. It needs only `numpy`. `matplotlib` is optional and is used for plots.

## What it models

| Subsystem | Model |
|---|---|
| **Rigid body** | Full Euler equations with a non-diagonal inertia tensor, gyroscopic coupling, internal wheel momentum, quaternion kinematics, and RK4 integration |
| **Motors / reaction wheels** | BLDC model with torque constant `Kt` and back-EMF `Ke`. Includes the supply-voltage limit, which produces the torque–speed curve and lets available torque collapse near no-load speed. Also includes current limiting, current-loop lag, Coulomb + viscous bearing friction, and cogging/torque ripple |
| **Gyro** | Turn-on bias, angle random walk (ARW), rate random walk (RRW), Gauss-Markov bias instability, temperature coefficient, scale-factor error, axis misalignment, saturation, and quantization |
| **GNC** | Gyro-integrated attitude estimator, an optional star tracker with bias estimation, and a quaternion PD controller with gyroscopic compensation |

## Source: `attitude_sim.py`

```python
#!/usr/bin/env python3
"""
attitude_sim.py  -  3-axis rigid-body attitude simulator (Cycle 20)

Models:
  * Rigid-body rotational dynamics (Euler's equations + reaction-wheel momentum)
  * Brushless reaction-wheel motors with torque-speed curves, current limits,
    current-loop lag, friction, and torque ripple
  * MEMS/FOG-style rate gyro with drift (bias, ARW, RRW, bias instability,
    temperature sensitivity, scale factor, misalignment, quantization)
  * Gyro-propagated attitude estimator with optional star-tracker correction
  * Quaternion PD attitude controller

Quaternion convention: scalar-first [w, x, y, z], q maps body -> inertial.

Usage examples:
  python attitude_sim.py                      # 120 s slew, star tracker on
  python attitude_sim.py --no-startracker     # watch gyro drift corrupt attitude
  python attitude_sim.py --tf 600 --csv out.csv --no-plot
  python attitude_sim.py --torque-curve       # plot motor torque-speed curve only
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

DEG = math.pi / 180.0
ARCSEC = DEG / 3600.0


# =============================================================================
# Quaternion utilities
# =============================================================================
def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_norm(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    h = 0.5 * angle
    return np.concatenate(([math.cos(h)], math.sin(h) * axis))


def q_exp(rotvec: np.ndarray) -> np.ndarray:
    """Rotation vector -> quaternion (exact exponential map)."""
    ang = np.linalg.norm(rotvec)
    if ang < 1e-12:
        return q_norm(np.concatenate(([1.0], 0.5 * rotvec)))
    return q_from_axis_angle(rotvec / ang, ang)


def q_angle(q: np.ndarray) -> float:
    """Principal rotation angle of quaternion [rad]."""
    return 2.0 * math.atan2(np.linalg.norm(q[1:]), abs(q[0]))


def q_to_euler(q: np.ndarray) -> np.ndarray:
    """ZYX Euler angles (roll, pitch, yaw) [rad]."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


# =============================================================================
# Parameters
# =============================================================================
@dataclass
class BodyParams:
    # Inertia tensor [kg m^2] (slightly non-diagonal for realistic coupling)
    J: np.ndarray = field(default_factory=lambda: np.array([
        [0.50, 0.010, -0.005],
        [0.010, 0.60, 0.008],
        [-0.005, 0.008, 0.40]]))


@dataclass
class MotorParams:
    """Brushless reaction-wheel motor (one per body axis)."""
    Kt: float = 0.030         # torque constant [N m / A]
    Ke: float = 0.030         # back-EMF constant [V s / rad] (SI: Ke == Kt)
    R: float = 1.5            # winding resistance [ohm]
    V_bus: float = 12.0       # supply voltage [V]
    I_max: float = 2.0        # driver current limit [A]
    tau_i: float = 0.004      # closed current-loop time constant [s]
    Iw: float = 5.0e-3        # wheel rotor inertia [kg m^2]
    c_coulomb: float = 2.0e-4 # Coulomb friction [N m]
    b_visc: float = 1.0e-6    # viscous friction [N m s / rad]
    ripple: float = 0.02      # torque ripple fraction
    n_ripple: int = 12        # ripple harmonics per revolution
    omega_eps: float = 0.5    # friction smoothing speed [rad/s]

    @property
    def omega_no_load(self) -> float:
        return self.V_bus / self.Ke

    def current_window(self, omega):
        """Voltage- and driver-limited current range at wheel speed omega.
        This window *is* the torque-speed curve."""
        i_hi = np.minimum(self.I_max, (self.V_bus - self.Ke * omega) / self.R)
        i_lo = np.maximum(-self.I_max, (-self.V_bus - self.Ke * omega) / self.R)
        return i_lo, i_hi

    def torque_curve(self, omega):
        """Max positive / negative electromagnetic torque vs. speed."""
        lo, hi = self.current_window(omega)
        return self.Kt * lo, self.Kt * hi

    def friction(self, omega):
        return self.c_coulomb * np.tanh(omega / self.omega_eps) + self.b_visc * omega


@dataclass
class GyroParams:
    """Rate-gyro error model (typical tactical-grade MEMS)."""
    rate_hz: float = 200.0
    arw_deg_rt_hr: float = 0.15        # angle random walk [deg/sqrt(h)]
    rrw_deg_hr_rt_hr: float = 2.0      # rate random walk [deg/h/sqrt(h)]
    bias_instab_deg_hr: float = 3.0    # Gauss-Markov bias instability [deg/h]
    bias_corr_time: float = 300.0      # Gauss-Markov correlation time [s]
    turn_on_bias_deg_hr: float = 20.0  # 1-sigma turn-on bias [deg/h]
    temp_coef_deg_hr_C: float = 0.5    # bias tempco [deg/h/degC]
    scale_factor_ppm: float = 500.0    # 1-sigma scale-factor error
    misalign_mrad: float = 1.0         # 1-sigma axis misalignment
    saturation_dps: float = 400.0      # measurement range [deg/s]
    lsb_dps: float = 0.0005            # quantization step [deg/s]
    T0: float = 25.0                   # reference temperature [degC]
    T_ramp_C_per_s: float = 0.01       # thermal ramp during the run


@dataclass
class StarTrackerParams:
    enabled: bool = True
    rate_hz: float = 2.0
    noise_arcsec: float = 10.0
    k_att: float = 0.3      # attitude correction gain (0..1)
    k_bias: float = 0.02    # bias estimation gain


@dataclass
class ControlParams:
    rate_hz: float = 100.0
    wn: float = 0.8         # closed-loop natural frequency [rad/s]
    zeta: float = 0.9


@dataclass
class SimParams:
    dt: float = 1.0e-3      # physics step [s]
    tf: float = 120.0
    seed: int = 20
    sl