# 3-Axis Rigid-Body Spacecraft / Gimbal Physics Simulator (Cycle 24)

I've treated "Cycle 24" as a revision label for this build. The simulator is a single self-contained file. It requires only `numpy`, and `matplotlib` is optional for plots.

**What it models:**

| Subsystem | Model |
|---|---|
| **Rigid body** | Full 3-axis Euler equations with a non-diagonal inertia tensor, gyroscopic coupling, quaternion kinematics, and RK4 integration |
| **Actuators** | 3 reaction wheels driven by BLDC motors: back-EMF torque-speed curve, current limit, voltage limit, Coulomb and viscous bearing friction, and motor time constant |
| **Gyro** | Angle random walk (ARW), bias instability (Gauss-Markov), rate random walk, temperature-dependent bias, scale-factor error, axis misalignment, quantization, and saturation |
| **Estimation** | Pure gyro dead-reckoning (shows raw drift) plus a complementary filter with bias estimation using a noisy star tracker |
| **Control** | Quaternion PD controller with gyroscopic feed-forward, a slew maneuver, and environmental disturbance torques |
| **Verification** | Built-in angular-momentum conservation self-test |

```python
#!/usr/bin/env python3
"""
rigidsim.py  -  3D Rigid Body Physics Simulator  (Cycle 24)
==========================================================
3-axis rigid body dynamics + reaction-wheel motor torque curves + gyro drift.

Conventions
-----------
* Quaternions are scalar-first  q = [w, x, y, z], representing body -> inertial.
* Angular velocity omega is expressed in the BODY frame (rad/s).
* Reaction wheels are aligned with body X, Y, Z axes.
* SI units throughout unless stated otherwise.

Usage
-----
    python rigidsim.py                 # run default mission + plots
    python rigidsim.py --no-plot       # headless
    python rigidsim.py --selftest      # momentum-conservation test
    python rigidsim.py --tend 300 --seed 7
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field

import numpy as np

DEG = math.pi / 180.0


# =============================================================================
# Quaternion / rotation utilities
# =============================================================================
def q_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a (x) b."""
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


def q_from_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    h = 0.5 * angle
    return np.concatenate(([math.cos(h)], math.sin(h) * axis))


def q_exp(rotvec: np.ndarray) -> np.ndarray:
    """Rotation vector -> quaternion."""
    theta = np.linalg.norm(rotvec)
    if theta < 1e-12:
        return q_normalize(np.concatenate(([1.0], 0.5 * rotvec)))
    return q_from_axis_angle(rotvec / theta, theta)


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """DCM that maps body vectors to inertial vectors."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_error_vec(q_ref: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Small-angle error rotation vector (rad) taking q_ref -> q, shortest path."""
    qe = q_mul(q_conj(q_ref), q)
    if qe[0] < 0:
        qe = -qe
    return 2.0 * qe[1:]


def q_angle_between(a: np.ndarray, b: np.ndarray) -> float:
    d = abs(float(np.dot(a, b)))
    return 2.0 * math.acos(min(1.0, d))


def omega_matrix(w: np.ndarray) -> np.ndarray:
    """q_dot = 0.5 * Omega(w) q  for body-frame w."""
    wx, wy, wz = w
    return np.array([
        [0.0, -wx, -wy, -wz],
        [wx, 0.0, wz, -wy],
        [wy, -wz, 0.0, wx],
        [wz, wy, -wx, 0.0],
    ])


# =============================================================================
# Motor model (BLDC driving a reaction wheel)
# =============================================================================
@dataclass
class MotorParams:
    kt: float = 0.025          # torque constant   [N*m/A]
    ke: float = 0.025          # back-EMF constant [V*s/rad]  (= kt in SI)
    R: float = 1.2             # winding resistance [ohm]
    V_bus: float = 28.0        # supply voltage [V]
    I_max: float = 2.0         # driver current limit [A]
    tau_e: float = 0.005       # current-loop time constant [s]
    coulomb: float = 2.0e-4    # Coulomb bearing friction [N*m]
    viscous: float = 1.5e-6    # viscous friction [N*m*s/rad]
    J_wheel: float = 0.005     # wheel rotor inertia [kg*m^2]
    max_speed: float = 6000 * 2 * math.pi / 60  # mechanical speed limit [rad/s]


class Motor:
    """
    Torque-speed curve:
        T_avail(+) = min(kt*I_max, kt*(V - ke*w)/R)   (motoring, accelerating)
        T_avail(-) = max(-kt*I_max, kt*(-V - ke*w)/R)
    The commanded torque is clipped to this envelope, then passed through a
    first-order electrical lag. Bearing friction opposes wheel rotation.
    """

    def __init__(self, p: MotorParams):
        self.p = p
        self.torque = 0.0  # actual electromagnetic torque (lag state)

    def envelope(self, w: float) -> tuple[float, float]:
        p = self.p
        t_cur = p.kt * p.I_max
        t_hi = p.kt * (p.V_bus - p.ke * w) / p.R
        t_lo = p.kt * (-p.V_bus - p.ke * w) / p.R
        hi = min(t_cur, t_hi)
        lo = max(-t_cur, t_lo)
        # Hard overspeed protection
        if w >= p.max_speed:
            hi = min(hi, 0.0)
        if w <= -p.max_speed:
            lo = max(lo, 0.0)
        return lo, hi

    def friction(self, w: float) -> float:
        p = self.p
        # Smooth sign() via tanh to avoid stiff discontinuity at w=0
        return p.coulomb * math.tanh(w / 0.5) + p.viscous * w

    def step(self, cmd: float, w: float, dt: float) -> float:
        lo, hi = self.envelope(w)
        target = min(max(cmd, lo), hi)
        a = 1.0 - math.exp(-dt / self.p.tau_e)
        self.torque += a * (target - self.torque)
        # Re-clip in case speed moved the envelope
        self.torque = min(max(self.torque, lo), hi)
        return self.torque

    def current(self) -> float:
        return self.torque / self.p.kt

    def electrical_power(self, w: float) -> float:
        i = self.current()
        return i * i * self.p.R + self.torque * w


# =============================================================================
# Gyro model
# =============================================================================
@dataclass
class GyroParams:
    arw_deg_rthr: float = 0.15          # angle random walk [deg/sqrt(hr)]
    bias_inst_deg_hr: float = 3.0       # bias instability [deg/hr]
    bias_tau: float = 300.0             # Gauss-Markov correlation time [s]
    rrw_deg_hr_rthr: float = 1.0        # rate random walk [deg/hr/sqrt(hr)]
    turn_on_bias_deg_hr: float = 20.0   # constant turn-on bias, 1-sigma
    temp_coeff_deg_hr_C: float = 0.5    # bias temperature sensitivity
    scale_factor_ppm: float = 300.0     # scale-factor error, 1-sigma
    misalign_mrad: float = 1.0          # axis misalignment, 1-sigma
    quant_rad_s: float = 1.0e-6         # LSB
    range_rad_s: float = 400 * DEG      # full scale
    T_ref: float = 25.0                 # calibration temperature [C]


class Gyro:
    """
    Measurement:
        w_meas = Q( sat( (I + S + M) w_true + b_on + b_gm + b_rrw
                         + k_T (T - T_ref) + n_arw ) )
    """

    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.sig_arw = p.arw_deg_rthr * DEG / 60.0              # rad/sqrt(s)
        self.sig_bi = p.bias_inst_deg_hr * DEG / 3600.0         # rad/s
        self.sig_rrw = p.rrw_deg_hr_rthr * DEG / 3600.0 / 60.0  # rad/s/sqrt(s)
        self.k_T = p.temp_coeff_deg_hr_C * DEG / 3600.0         # rad/s/C

        self.b_on = rng.normal(0, p.turn_on_bias_deg_hr * DEG / 3600.0, 3)
        self.b_gm = np.zeros(3)
        self.b_rrw = np.zeros(3)

        sf = rng.normal(0, p.scale_factor_ppm * 1e-6, 3)
        ma = rng.normal(0, p.misalign_mrad * 1e-3, (3, 3))
        np.fill_diagonal(ma, 0.0)
        self.C = np.eye(3) + np.diag(sf) + ma

    def true_bias(self, temp_C: float) -> np.ndarray:
        return self.b_on + self.b_gm + self.b_rrw + self.k_T * (temp_C - self.p.T_ref)

    def measure(self, w_true: np.ndarray,