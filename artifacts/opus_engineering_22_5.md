# 3-Axis Rigid Body Attitude Simulator with Reaction-Wheel Motor Curves and Gyro Drift

I've treated "Cycle 22" as a version label. If it refers to a specific spec, such as a course assignment or an internal requirements document, share it and I'll align the parameters to it.

The program is a single file and needs only `numpy`. `matplotlib` is optional and only used for plots. It models:

- **Rigid body dynamics:**
  - Full inertia tensor and Euler's equations with gyroscopic coupling.
  - Quaternion kinematics, integrated with fixed-step RK4.
- **Reaction-wheel motors:**
  - Brushless DC model with torque constant `Kt`, back-EMF `Ke`, and winding resistance `R`.
  - Current limit and bus-voltage limit, which together produce the classic torque–speed curve.
  - Coulomb and viscous friction, plus an optional user-supplied lookup-table curve.
- **Gyro errors:**
  - Angle random walk (ARW), rate random walk (RRW), and Gauss–Markov bias instability.
  - Temperature-dependent bias, scale factor, axis misalignment, and quantization.
- **Estimation and control:**
  - Gyro-propagated attitude estimate, with an optional star-tracker correction (Mahony-style bias estimation).
  - Quaternion PD controller.
- **Verification:**
  - A momentum and energy conservation self-test (`--selftest`).

```python
#!/usr/bin/env python3
"""
attitude_sim.py  --  3-axis rigid-body attitude simulator (Cycle 22)

Models:
  * Rigid body: full inertia tensor, Euler's equations incl. wheel momentum,
    quaternion kinematics (scalar-first, body->inertial), RK4 integration.
  * Reaction wheels (3, body-aligned): BLDC motor with Kt, Ke, R, current limit,
    bus-voltage limit (=> torque-speed curve), Coulomb+viscous friction,
    optional tabulated torque-speed curve.
  * Gyro: ARW, RRW, first-order Gauss-Markov bias instability, temperature
    bias coefficient, scale factor, misalignment, quantization.
  * Estimator: gyro integration + optional star tracker (Mahony-style
    attitude/bias correction).
  * Controller: quaternion PD + gyroscopic feed-forward.

Usage:
  python attitude_sim.py                  # run default scenario + plots
  python attitude_sim.py --no-startracker # watch gyro drift accumulate
  python attitude_sim.py --selftest       # conservation checks
  python attitude_sim.py --duration 600 --seed 3 --no-plot
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# =============================================================================
# Quaternion utilities (scalar-first [w, x, y, z])
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


def qnormalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def qexp(rotvec: np.ndarray) -> np.ndarray:
    """Quaternion from rotation vector (axis * angle)."""
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return qnormalize(np.array([1.0, *(0.5 * rotvec)]))
    axis = rotvec / angle
    return np.array([np.cos(angle / 2), *(np.sin(angle / 2) * axis)])


def qto_dcm(q: np.ndarray) -> np.ndarray:
    """DCM rotating body vectors into inertial frame."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def qangle_between(a: np.ndarray, b: np.ndarray) -> float:
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(a, b)))
    return 2.0 * np.arccos(min(1.0, d))


def qerror_vec(q_ref: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Small-angle error rotation vector of q relative to q_ref (body frame)."""
    qe = qmul(qconj(q_ref), q)
    if qe[0] < 0:
        qe = -qe
    return 2.0 * qe[1:]


def qfrom_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


def qto_euler(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


# =============================================================================
# Motor / reaction wheel model
# =============================================================================

@dataclass
class MotorParams:
    Kt: float = 0.025          # N*m/A torque constant
    Ke: float = 0.025          # V/(rad/s) back-EMF constant (SI: Ke == Kt)
    R: float = 1.2             # ohm winding resistance
    I_max: float = 4.0         # A current limit (driver)
    V_bus: float = 28.0        # V supply
    tau_coulomb: float = 2e-4  # N*m Coulomb friction
    b_viscous: float = 1e-6    # N*m/(rad/s) viscous friction
    omega_max: float = 600.0   # rad/s mechanical overspeed cutoff (~5700 rpm)
    # Optional tabulated torque-speed envelope: (speed rad/s, max torque N*m).
    # If given, it overrides the electrical envelope.
    curve_speed: Optional[np.ndarray] = None
    curve_torque: Optional[np.ndarray] = None


class ReactionWheelMotor:
    """BLDC reaction wheel driven by a current-mode controller.

    Command: desired motor torque. The driver converts it to a current
    command, saturated by:
        |I| <= I_max                                  (current limit)
        R*I + Ke*Omega in [-V_bus, V_bus]             (voltage headroom)
    The voltage limit yields the linear torque-speed droop:
        T_max(Omega) = Kt * (V_bus - Ke*|Omega|) / R  (accelerating)
    """

    def __init__(self, p: MotorParams, J_wheel: float):
        self.p = p
        self.J = J_wheel
        self.i_cmd = 0.0  # held current command (ZOH between control steps)

    def set_torque_command(self, tau_cmd: float) -> None:
        self.i_cmd = tau_cmd / self.p.Kt

    def electrical_torque(self, omega: float) -> float:
        """Torque produced at wheel speed omega for the held current command."""
        p = self.p
        i = float(np.clip(self.i_cmd, -p.I_max, p.I_max))

        if p.curve_speed is not None and p.curve_torque is not None:
            t_lim = float(np.interp(abs(omega), p.curve_speed, p.curve_torque))
            tau = float(np.clip(p.Kt * i, -t_lim, t_lim))
        else:
            # Voltage-feasible current range: -V_bus <= R*i + Ke*omega <= V_bus
            i_hi = (p.V_bus - p.Ke * omega) / p.R
            i_lo = (-p.V_bus - p.Ke * omega) / p.R
            i = float(np.clip(i, i_lo, i_hi))
            tau = p.Kt * i

        # Overspeed protection: refuse to accelerate further past omega_max
        if abs(omega) >= p.omega_max and np.sign(tau) == np.sign(omega):
            tau = 0.0
        return tau

    def friction_torque(self, omega: float) -> float:
        p = self.p
        # tanh smooths the Coulomb sign() so RK4 does not chatter at Omega=0
        return p.tau_coulomb * np.tanh(omega / 0.5) + p.b_viscous * omega

    def net_torque(self, omega: float) -> float:
        """Torque accelerating the wheel (= reaction removed from body)."""
        return self.electrical_torque(omega) - self.friction_torque(omega)

    def electrical_power(self, omega: float) -> float:
        tau = self.electrical_torque(omega)
        i = tau / self.p.Kt
        return i * i * self.p.R + tau * omega

    def torque_speed_curve(self, n: int = 200):
        """Max accelerating torque envelope vs speed (for plotting)."""
        speeds = np.linspace(0, self.p.omega_max, n)
        saved = self.i_cmd
        self.i_cmd = 1e9  # saturate command
        torques = np.array([self.electrical_torque(w) for w in speeds])
        self.i_cmd = saved
        return speeds, torques


# =============================================================================
# Gyro model
# =============================================================================

@dataclass
class GyroParams:
    rate_hz: float = 100.0
    arw: float = np.deg2rad(0.15) / 60.0          # rad/sqrt(s)    (0.15 deg/sqrt(h))
    rrw: float = np.deg2rad(0.5) / 3600.0 / 60.0  # rad/s/sqrt(s)  (rate random walk)
    bias_instability: float = np.deg2rad(1.0) / 3600.0  # rad/s (1 deg/h)
    bias_corr_time: float = 300.0                 # s Gauss-Markov correlation time
    initial_bias_sigma: float = np.deg2rad(5.0) / 3600.0  # rad/s turn-on bias
    temp_coeff: float = np.deg2rad(0.02) / 3600.0 # rad/s per degC
    temp_ref: float = 25.0
    scale_factor_ppm: float = 300.0
    misalignment_