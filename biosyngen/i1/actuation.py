"""Excitation -> activation -> joint torque hypothesis (latent, not measured EMG).

Activation (Thelen 2003 form; values ASSUMED, see params.PROVENANCE):

    da/dt = (u - a) / tau(u, a)
    tau = tau_act (0.5 + 1.5 a)      if u > a
    tau = tau_deact / (0.5 + 1.5 a)  otherwise

With 0 < u_min <= u <= 1 the activation stays inside [min(a0, u_min), 1].

Joint torque generator (extensor: v = omega / omega_max; flexor: v = -omega / omega_max):

    tau = a * T0 * f_L(theta) * f_V(v)
    f_L = exp(-((theta - theta_opt) / w)^2)
    f_V = (1 - v) / (1 + v / k)              for 0 <= v < 1 (Hill hyperbola)
        = 0                                  for v >= 1
        = E - (E - 1) / (1 - c v),  c = (1 + 1/k) / (E - 1)   for v < 0

The eccentric branch matches the concentric value and slope at v = 0 and
tends to E (eccentric_max_ratio) for fast lengthening.
"""

from __future__ import annotations

import math

from .params import ActuatorParameters, JointLimits, NeuromuscularParameters


def activation_derivative(
    excitation: float, activation: float, tau_act: float, tau_deact: float
) -> float:
    if excitation > activation:
        tau = tau_act * (0.5 + 1.5 * activation)
    else:
        tau = tau_deact / (0.5 + 1.5 * activation)
    return (excitation - activation) / tau


def force_length(params: ActuatorParameters, angle_rad: float) -> float:
    d = (angle_rad - params.optimal_angle_rad) / params.angle_width_rad
    return math.exp(-d * d)


def force_velocity(params: ActuatorParameters, normalized_shortening: float) -> float:
    v = normalized_shortening
    k = params.hill_curvature
    if v >= 1.0:
        return 0.0
    if v >= 0.0:
        return (1.0 - v) / (1.0 + v / k)
    e = params.eccentric_max_ratio
    slope = 1.0 + 1.0 / k  # |df/dv| of the concentric branch at v = 0
    c = slope / (e - 1.0)
    return e - (e - 1.0) / (1.0 - c * v)


def actuator_torque(
    params: ActuatorParameters,
    activation: float,
    angle_rad: float,
    extension_velocity_rad_s: float,
    extensor: bool,
) -> float:
    """Signed, extension-positive joint torque of one actuator."""
    if extensor:
        v = extension_velocity_rad_s / params.max_shortening_velocity_rad_s
        sign = 1.0
    else:
        v = -extension_velocity_rad_s / params.max_shortening_velocity_rad_s
        sign = -1.0
    return sign * activation * params.max_isometric_torque_n_m * force_length(
        params, angle_rad
    ) * force_velocity(params, v)


def passive_limit_torque(limits: JointLimits, angle_rad: float, velocity_rad_s: float) -> float:
    """Quadratic end-range spring with damping active only beyond a limit."""
    if angle_rad > limits.max_angle_rad:
        d = angle_rad - limits.max_angle_rad
        damping = limits.damping_n_m_s_per_rad * velocity_rad_s if velocity_rad_s > 0.0 else 0.0
        return -limits.stiffness_n_m_per_rad2 * d * d - damping
    if angle_rad < limits.min_angle_rad:
        d = limits.min_angle_rad - angle_rad
        damping = limits.damping_n_m_s_per_rad * velocity_rad_s if velocity_rad_s < 0.0 else 0.0
        return limits.stiffness_n_m_per_rad2 * d * d - damping
    return 0.0


def joint_torques(
    nm: NeuromuscularParameters,
    activations: list[float],
    angles: tuple[float, float, float],
    velocities: tuple[float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, ...], tuple[float, float, float]]:
    """Return (net joint torques, six actuator torques, three passive torques)."""
    acts = nm.actuators()
    limits = (nm.ankle_limits, nm.knee_limits, nm.hip_limits)
    actuator = []
    net = []
    passive = []
    for j in range(3):
        te = actuator_torque(acts[2 * j], activations[2 * j], angles[j], velocities[j], True)
        tf = actuator_torque(acts[2 * j + 1], activations[2 * j + 1], angles[j], velocities[j], False)
        tp = passive_limit_torque(limits[j], angles[j], velocities[j])
        actuator += [te, tf]
        passive.append(tp)
        net.append(te + tf + tp)
    return (net[0], net[1], net[2]), tuple(actuator), (passive[0], passive[1], passive[2])


def activations_for_torques(
    nm: NeuromuscularParameters,
    required: tuple[float, float, float],
    angles: tuple[float, float, float],
    antagonist_activation: float,
) -> list[float]:
    """Static (zero-velocity) activations producing the required net torques.

    The antagonist of each joint is held at ``antagonist_activation``; the agonist
    is solved. Raises ValueError if the agonist would exceed full activation.
    """
    acts = nm.actuators()
    out = [0.0] * 6
    for j in range(3):
        ext, flex = acts[2 * j], acts[2 * j + 1]
        ext_cap = ext.max_isometric_torque_n_m * force_length(ext, angles[j])
        flex_cap = flex.max_isometric_torque_n_m * force_length(flex, angles[j])
        if required[j] >= 0.0:
            a_f = antagonist_activation
            a_e = (required[j] + a_f * flex_cap) / ext_cap
        else:
            a_e = antagonist_activation
            a_f = (-required[j] + a_e * ext_cap) / flex_cap
        if not (0.0 < a_e <= 1.0 and 0.0 < a_f <= 1.0):
            raise ValueError(
                f"static posture needs activation outside (0, 1] at joint {j}: "
                f"extensor {a_e:.3f}, flexor {a_f:.3f}"
            )
        out[2 * j], out[2 * j + 1] = a_e, a_f
    return out
