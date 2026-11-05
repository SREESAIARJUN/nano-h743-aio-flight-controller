# 3-Axis Rigid Body Simulator with Motor Torque Curves and Gyro Drift

I don't have context from earlier cycles, so this is a self-contained, single-file implementation. It depends only on NumPy, plus Matplotlib if you want plots. It covers:

- **Rigid body dynamics:** full 3D Euler equations with quaternion kinematics, a non-diagonal inertia tensor, and gyroscopic coupling from wheel momentum. Integration is fixed-step RK4.
- **Actuators:** three reaction wheels driven by brushless DC motors. Each motor model includes:
  - torque–speed curve (back-EMF, supply voltage, current limit)
  - Coulomb and viscous friction
  - wheel saturation
- **Gyro model:**
  - angle random walk
  - bias instability (Gauss-Markov process)
  - rate random walk
  - temperature-dependent bias
  - scale factor error
  - axis misalignment
  - quantization and saturation
- **Estimation and control:**
  - a Mahony-style estimator, with optional star-tracker corrections, that estimates gyro bias
  - a quaternion PD controller with gyroscopic feed-forward
- **Diagnostics:** momentum conservation check, CSV export, and plots.

```python
#!/usr/bin/env python3
"""
attitude_sim.py  --  3-axis rigid-body attitude simulator (Cycle 3)

Models:
  * Rigid-body rotational dynamics (Euler eqs. + quaternion kinematics, RK4)
  * Reaction-wheel BLDC motors with torque-speed curves, current/voltage
    limits, Coulomb + viscous friction
  * MEMS/FOG-style gyro: ARW, bias instability, rate random walk,
    thermal bias drift, scale factor, misalignment, quantization, saturation
  * Gyro-propagated attitude estimator with optional star-tracker fixes
  * Quaternion PD attitude controller

Usage:
  python attitude_sim.py                    # default slew, plots
  python attitude_sim.py --no-star          # pure gyro propagation -> watch drift
  python attitude_sim.py --t 600 --csv out.csv --no-plot
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field

import numpy as np

# ============================================================================
# Quaternion utilities (scalar-first: q = [w, x, y, z], body->inertial)
# ============================================================================

def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def q_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_norm(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])


def q_from_rotvec(v: np.ndarray) -> np.ndarray:
    ang = np.linalg.norm(v)
    if ang < 1e-12:
        return q_norm(np.array([1.0, *(0.5 * v)]))
    ax = v / ang
    return np.array([np.cos(ang / 2), *(np.sin(ang / 2) * ax)])


def q_to_rotvec(q: np.ndarray) -> np.ndarray:
    q = q if q[0] >= 0 else -q            # shortest path
    s = np.linalg.norm(q[1:])
    if s < 1e-12:
        return 2.0 * q[1:]
    ang = 2.0 * np.arctan2(s, q[0])
    return ang * q[1:] / s


def q_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


def q_to_dcm(q: np.ndarray) -> np.ndarray:
    """Body->inertial rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def omega_matrix(w: np.ndarray) -> np.ndarray:
    """q_dot = 0.5 * Omega(w) @ q  (w in body frame)."""
    wx, wy, wz = w
    return np.array([
        [0.0, -wx, -wy, -wz],
        [wx,  0.0,  wz, -wy],
        [wy, -wz,  0.0,  wx],
        [wz,  wy, -wx,  0.0],
    ])


# ============================================================================
# Motor / reaction wheel model
# ============================================================================

@dataclass
class MotorParams:
    Kt: float = 0.025          # torque constant [N*m/A] (== Ke in SI [V*s/rad])
    R: float = 1.2             # winding resistance [ohm]
    V_bus: float = 28.0        # supply voltage [V]
    I_max: float = 2.0         # driver current limit [A]
    J_wheel: float = 2.0e-3    # wheel rotor inertia [kg*m^2]
    tau_coulomb: float = 4e-4  # Coulomb friction [N*m]
    b_viscous: float = 2e-6    # viscous friction [N*m*s/rad]
    omega_max: float = 650.0   # hard overspeed limit [rad/s] (~6200 rpm)


class ReactionWheelMotor:
    """
    BLDC reaction wheel with a quasi-static electrical model (the electrical
    time constant L/R << control period). The available torque at wheel speed
    Omega is bounded by:
        |I| <= I_max                       (driver current limit)
        I in [(-V - Ke*Omega)/R, (V - Ke*Omega)/R]   (back-EMF voltage limit)
    That gives the classic flat-then-linearly-falling torque-speed curve.
    """

    def __init__(self, axis: np.ndarray, p: MotorParams):
        self.axis = np.asarray(axis, float) / np.linalg.norm(axis)
        self.p = p
        self.current = 0.0
        self.tau_motor = 0.0

    # -- curves -----------------------------------------------------------
    def torque_limits(self, omega: float) -> tuple[float, float]:
        p = self.p
        i_hi = min(p.I_max, (p.V_bus - p.Kt * omega) / p.R)
        i_lo = max(-p.I_max, (-p.V_bus - p.Kt * omega) / p.R)
        if i_hi < i_lo:  # beyond no-load speed: only braking current possible
            i_hi = i_lo = (np.sign(-omega) * p.V_bus - p.Kt * omega) / p.R
        return p.Kt * i_lo, p.Kt * i_hi

    def friction(self, omega: float) -> float:
        p = self.p
        # smooth sign to avoid chattering at zero speed
        return p.tau_coulomb * np.tanh(omega / 0.5) + p.b_viscous * omega

    def torque_speed_curve(self, n: int = 400):
        p = self.p
        w_nl = p.V_bus / p.Kt
        om = np.linspace(-1.1 * w_nl, 1.1 * w_nl, n)
        lo, hi = zip(*(self.torque_limits(o) for o in om))
        return om, np.array(lo), np.array(hi)

    # -- command ----------------------------------------------------------
    def command(self, tau_cmd: float, omega: float) -> float:
        """Apply a torque request; returns the actual electromagnetic torque."""
        lo, hi = self.torque_limits(omega)
        tau = float(np.clip(tau_cmd, lo, hi))
        # overspeed protection: forbid further acceleration past the limit
        if abs(omega) >= self.p.omega_max and np.sign(tau) == np.sign(omega):
            tau = 0.0
        self.tau_motor = tau
        self.current = tau / self.p.Kt
        return tau

    def net_wheel_torque(self, omega: float) -> float:
        return self.tau_motor - self.friction(omega)

    def electrical_power(self, omega: float) -> float:
        p = self.p
        return self.current**2 * p.R + p.Kt * omega * self.current


# ============================================================================
# Gyro model
# ============================================================================

@dataclass
class GyroParams:
    rate_hz: float = 100.0
    arw: float = np.deg2rad(0.15) / 60.0              # [rad/sqrt(s)] (0.15 deg/sqrt(h))
    bias_instab: float = np.deg2rad(3.0) / 3600.0     # [rad/s] (3 deg/h)
    bias_corr_time: float = 300.0                     # Gauss-Markov tau [s]
    rrw: float = np.deg2rad(0.5) / 3600.0 / 60.0      # rate random walk [rad/s^1.5]
    bias_init_sigma: float = np.deg2rad(10.0) / 3600  # turn-on bias [rad/s]
    temp_coeff: float = np.deg2rad(0.05) / 3600       # [rad/s per degC]
    temp_ref: float = 25.0
    scale_factor_ppm: float = 300.0
    misalign_rad: float = 1e-3
    quant: float = np.deg2rad(0.01)                   # LSB [rad/s]
    saturation: float = np.deg2rad(300.0)             # [rad/s]


class Gyro:
    """
    meas = (I + S + M) * w_true + b_on + b_gm + b_rrw + k_T*(T-Tref) + ARW noise
    then saturation and quantization.
    """

    def __init__(self, p: GyroParams, rng: np.random.Generator):
        self.p = p
        self.rng = rng
        self.dt = 1.0 / p.rate_hz
        self.b_on = rng.normal(0, p.bias_init_sigma, 3)
        self.b_gm = np.zeros(3)
        self.b_rrw = np.zeros(3)
        S = np.diag(rng.normal(0, p.scale_factor_ppm * 1e-6, 3))
        M = rng.normal(0, p.misalign_rad, (3, 3))
        np.fill_diagonal(M, 0.0)
        self.C = np.eye(3) + S + M

    def total_bias(self, temp_c: float) -> np.ndarray:
        return (self.b_on + self.b_gm + self.b_rrw
                + self.p.temp_coeff * (temp_c - self.p.temp_ref))

    def step_bias(self):
        p, dt = self.p, self.dt
        # first-order Gauss-Markov bias instability
        phi = np.exp(-dt / p.bias_corr_time)
        sig = p.bias_instab * np.sqrt(1 - phi**2)
        self.b_gm = phi * self.b_gm + self.rng.normal(0, sig, 3)
        # rate random walk
        self.b_rrw += self.rng.normal(0, p.rrw * np.sqrt(dt), 3)

    def measure(self, w_true: np.ndarray, temp_c: float) -> np.ndarray:
        self.step_bias()
        p = self.p
        noise = self.r