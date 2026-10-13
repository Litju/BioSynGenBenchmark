"""Planar floating-base 4-segment body (foot, shank, thigh, HAT) and optional bar.

Generalized coordinates (inertial lab frame, +x anterior, +z up, radians CCW
from +x):

    q = [x_ankle, z_ankle, phi_foot, phi_shank, phi_thigh, phi_hat, (x_bar, z_bar)]

Every body point is ``(x_ankle, z_ankle) + sum_k c_k * (cos(phi_k + a_k), sin(phi_k + a_k))``
so positions, Jacobians and the convective term J'(q) q' follow from one
expression. The equations of motion are

    M(q) q'' + h(q, q') = G(q) + Q_actuators + Q_coupling + J_c(q)^T lambda

with M = sum_b m_b J_b^T J_b + I_b e_b e_b^T, h = sum_b m_b J_b^T (J_b' q'),
G = sum_b J_b^T (0, -m_b g). Contact forces lambda are the ground forces on
the heel/toe points and are solved together with q'' (see simulate.py).
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .params import BarParameters, BodyParameters


X, Z, PF, PS, PT, PH, XB, ZB = range(8)
ANGLE_INDICES = (PF, PS, PT, PH)

Term = tuple[int, float, float]  # (angle coordinate index, length, angle offset)


def _local_term(index: int, local_x: float, local_z: float) -> Term:
    return (index, math.hypot(local_x, local_z), math.atan2(local_z, local_x))


@dataclass(frozen=True, slots=True)
class Segment:
    name: str
    mass_kg: float
    inertia_kg_m2: float
    angle_index: int
    com_terms: tuple[Term, ...]


class PlanarBody:
    """Kinematics and mass-matrix dynamics of the declared planar body."""

    def __init__(self, params: BodyParameters, bar: BarParameters | None = None) -> None:
        self.params = params
        self.bar = bar
        self.n = 8 if bar is not None else 6
        p = params
        ls, lt, lh = p.shank_length_m, p.thigh_length_m, p.hat_length_m
        knee = ((PS, ls, 0.0),)
        hip = knee + ((PT, lt, 0.0),)
        inertias = p.segment_inertias_kg_m2()
        self.segments = (
            Segment("foot", p.foot_mass_kg, inertias[0], PF,
                    (_local_term(PF, p.foot_com_x_m, p.foot_com_z_m),)),
            Segment("shank", p.shank_mass_kg, inertias[1], PS,
                    ((PS, p.shank_com_from_ankle_fraction * ls, 0.0),)),
            Segment("thigh", p.thigh_mass_kg, inertias[2], PT,
                    knee + ((PT, p.thigh_com_from_knee_fraction * lt, 0.0),)),
            Segment("hat", p.hat_mass_kg, inertias[3], PH,
                    hip + ((PH, p.hat_com_from_hip_fraction * lh, 0.0),)),
        )
        self.points: dict[str, tuple[Term, ...]] = {
            "ankle": (),
            "heel": (_local_term(PF, p.heel_x_m, -p.ankle_height_m),),
            "toe": (_local_term(PF, p.toe_x_m, -p.ankle_height_m),),
            "knee": knee,
            "hip": hip,
            "shoulder": hip + (_local_term(PH, p.shoulder_along_hat_m, p.shoulder_posterior_m),),
            # Vertex of the head: hip-to-vertex distance 0.47 H (trochanter height
            # ~0.53 H, Winter, recalled). Used only for ground-penetration checks.
            "head_top": hip + ((PH, 0.47 * p.stature_m, 0.0),),
        }
        self.body_mass_kg = sum(s.mass_kg for s in self.segments)
        self.total_mass_kg = self.body_mass_kg + (bar.mass_kg if bar is not None else 0.0)

    # Point kinematics ------------------------------------------------------
    @staticmethod
    def point(q: list[float], terms: tuple[Term, ...]) -> tuple[float, float]:
        x, z = q[X], q[Z]
        for k, c, a in terms:
            ang = q[k] + a
            x += c * math.cos(ang)
            z += c * math.sin(ang)
        return x, z

    @staticmethod
    def point_velocity(q: list[float], qd: list[float], terms: tuple[Term, ...]) -> tuple[float, float]:
        vx, vz = qd[X], qd[Z]
        for k, c, a in terms:
            ang = q[k] + a
            vx -= c * math.sin(ang) * qd[k]
            vz += c * math.cos(ang) * qd[k]
        return vx, vz

    @staticmethod
    def point_jacobian(
        q: list[float], qd: list[float], terms: tuple[Term, ...]
    ) -> tuple[dict[int, float], dict[int, float], float, float]:
        """Return sparse Jacobian rows (x, z) and the convective term J' q'."""
        jx: dict[int, float] = {X: 1.0}
        jz: dict[int, float] = {Z: 1.0}
        bx = bz = 0.0
        for k, c, a in terms:
            ang = q[k] + a
            ca, sa = math.cos(ang), math.sin(ang)
            jx[k] = jx.get(k, 0.0) - c * sa
            jz[k] = jz.get(k, 0.0) + c * ca
            w2 = qd[k] * qd[k]
            bx -= c * ca * w2
            bz -= c * sa * w2
        return jx, jz, bx, bz

    # Dynamics ----------------------------------------------------------------
    def dynamics_terms(
        self, q: list[float], qd: list[float]
    ) -> tuple[list[list[float]], list[float]]:
        """Return M(q) and f = G(q) - h(q, q') (gravity minus convective terms)."""
        n = self.n
        g = self.params.gravity_m_s2
        m = [[0.0] * n for _ in range(n)]
        f = [0.0] * n
        for seg in self.segments:
            jx, jz, bx, bz = self.point_jacobian(q, qd, seg.com_terms)
            mass = seg.mass_kg
            for i, ji in jx.items():
                row = m[i]
                for j, jj in jx.items():
                    row[j] += mass * ji * jj
                f[i] -= mass * ji * bx
            for i, ji in jz.items():
                row = m[i]
                for j, jj in jz.items():
                    row[j] += mass * ji * jj
                f[i] -= mass * ji * bz + mass * g * ji
            m[seg.angle_index][seg.angle_index] += seg.inertia_kg_m2
        if self.bar is not None:
            m[XB][XB] += self.bar.mass_kg
            m[ZB][ZB] += self.bar.mass_kg
            f[ZB] -= self.bar.mass_kg * g
        return m, f

    def _bar_material_terms(self, q: list[float]) -> tuple[Term, ...]:
        """Terms of the HAT material point that currently coincides with the bar."""
        hx, hz = self.point(q, self.points["hip"])
        dx, dz = q[XB] - hx, q[ZB] - hz
        return self.points["hip"] + ((PH, math.hypot(dx, dz), math.atan2(dz, dx) - q[PH]),)

    def coupling_components(
        self, q: list[float], qd: list[float]
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        """Spring and damper forces on the bar (the body receives the opposite).

        The spring is central between the shoulder point and the bar. The damper
        acts at one point, the HAT material point coinciding with the bar, on
        their relative velocity. Both force pairs therefore have zero net
        moment, so the coupling conserves system angular momentum. The damper
        dissipates C |v_rel|^2.
        """
        if self.bar is None:
            return (0.0, 0.0), (0.0, 0.0)
        sx, sz = self.point(q, self.points["shoulder"])
        pvx, pvz = self.point_velocity(q, qd, self._bar_material_terms(q))
        k, c = self.bar.coupling_stiffness_n_m, self.bar.coupling_damping_n_s_m
        return (
            (k * (sx - q[XB]), k * (sz - q[ZB])),
            (c * (pvx - qd[XB]), c * (pvz - qd[ZB])),
        )

    def coupling_force(self, q: list[float], qd: list[float]) -> tuple[float, float]:
        """Total force on the bar from the gripped coupling."""
        (sx, sz), (dx, dz) = self.coupling_components(q, qd)
        return sx + dx, sz + dz

    def coupling_damping_power(self, q: list[float], qd: list[float]) -> float:
        if self.bar is None:
            return 0.0
        pvx, pvz = self.point_velocity(q, qd, self._bar_material_terms(q))
        rx, rz = pvx - qd[XB], pvz - qd[ZB]
        return -self.bar.coupling_damping_n_s_m * (rx * rx + rz * rz)

    def coupling_generalized_force(self, q: list[float], qd: list[float]) -> list[float]:
        out = [0.0] * self.n
        if self.bar is None:
            return out
        (fsx, fsz), (fdx, fdz) = self.coupling_components(q, qd)
        for terms, fx, fz in (
            (self.points["shoulder"], fsx, fsz),
            (self._bar_material_terms(q), fdx, fdz),
        ):
            jx, jz, _, _ = self.point_jacobian(q, qd, terms)
            for i, v in jx.items():
                out[i] -= v * fx
            for i, v in jz.items():
                out[i] -= v * fz
        out[XB] += fsx + fdx
        out[ZB] += fsz + fdz
        return out

    # Joint coordinates -----------------------------------------------------
    @staticmethod
    def joint_angles(q: list[float]) -> tuple[float, float, float]:
        """Extension-positive ankle, knee, hip angles (rad)."""
        return (
            q[PS] - q[PF],
            math.pi - (q[PT] - q[PS]),
            math.pi - (q[PT] - q[PH]),
        )

    @staticmethod
    def joint_velocities(qd: list[float]) -> tuple[float, float, float]:
        return (qd[PS] - qd[PF], qd[PS] - qd[PT], qd[PH] - qd[PT])

    def joint_generalized_force(self, torques: tuple[float, float, float]) -> list[float]:
        """Map extension-positive joint torques to generalized forces (virtual work)."""
        ta, tk, th = torques
        out = [0.0] * self.n
        out[PS] += ta + tk
        out[PF] -= ta
        out[PT] -= tk + th
        out[PH] += th
        return out

    # Whole-body quantities -------------------------------------------------
    def body_com(self, q: list[float]) -> tuple[float, float]:
        sx = sz = 0.0
        for seg in self.segments:
            x, z = self.point(q, seg.com_terms)
            sx += seg.mass_kg * x
            sz += seg.mass_kg * z
        return sx / self.body_mass_kg, sz / self.body_mass_kg

    def body_com_velocity(self, q: list[float], qd: list[float]) -> tuple[float, float]:
        sx = sz = 0.0
        for seg in self.segments:
            vx, vz = self.point_velocity(q, qd, seg.com_terms)
            sx += seg.mass_kg * vx
            sz += seg.mass_kg * vz
        return sx / self.body_mass_kg, sz / self.body_mass_kg

    def system_com(self, q: list[float]) -> tuple[float, float]:
        bx, bz = self.body_com(q)
        if self.bar is None:
            return bx, bz
        mb, mt = self.body_mass_kg, self.total_mass_kg
        return (mb * bx + self.bar.mass_kg * q[XB]) / mt, (mb * bz + self.bar.mass_kg * q[ZB]) / mt

    def system_com_velocity(self, q: list[float], qd: list[float]) -> tuple[float, float]:
        vx, vz = self.body_com_velocity(q, qd)
        if self.bar is None:
            return vx, vz
        mb, mt = self.body_mass_kg, self.total_mass_kg
        return (mb * vx + self.bar.mass_kg * qd[XB]) / mt, (mb * vz + self.bar.mass_kg * qd[ZB]) / mt

    def kinetic_energy(self, q: list[float], qd: list[float]) -> float:
        total = 0.0
        for seg in self.segments:
            vx, vz = self.point_velocity(q, qd, seg.com_terms)
            w = qd[seg.angle_index]
            total += 0.5 * seg.mass_kg * (vx * vx + vz * vz) + 0.5 * seg.inertia_kg_m2 * w * w
        if self.bar is not None:
            total += 0.5 * self.bar.mass_kg * (qd[XB] ** 2 + qd[ZB] ** 2)
        return total

    def potential_energy(self, q: list[float]) -> float:
        g = self.params.gravity_m_s2
        total = 0.0
        for seg in self.segments:
            total += seg.mass_kg * g * self.point(q, seg.com_terms)[1]
        if self.bar is not None:
            total += self.bar.mass_kg * g * q[ZB]
            sx, sz = self.point(q, self.points["shoulder"])
            dx, dz = sx - q[XB], sz - q[ZB]
            total += 0.5 * self.bar.coupling_stiffness_n_m * (dx * dx + dz * dz)
        return total

    def angular_momentum(self, q: list[float], qd: list[float], about: tuple[float, float]) -> float:
        """System angular momentum (kg m^2/s, +y = CCW in x-z) about a fixed lab point."""
        ox, oz = about
        total = 0.0
        for seg in self.segments:
            x, z = self.point(q, seg.com_terms)
            vx, vz = self.point_velocity(q, qd, seg.com_terms)
            total += seg.mass_kg * ((x - ox) * vz - (z - oz) * vx)
            total += seg.inertia_kg_m2 * qd[seg.angle_index]
        if self.bar is not None:
            total += self.bar.mass_kg * ((q[XB] - ox) * qd[ZB] - (q[ZB] - oz) * qd[XB])
        return total

    def segment_states(self, q: list[float], qd: list[float]) -> list[dict[str, float]]:
        """Public per-segment kinematic export (no solver internals)."""
        out = []
        for seg in self.segments:
            x, z = self.point(q, seg.com_terms)
            vx, vz = self.point_velocity(q, qd, seg.com_terms)
            out.append({
                "name": seg.name,
                "x_m": x, "z_m": z, "vx_m_s": vx, "vz_m_s": vz,
                "angle_rad": q[seg.angle_index], "omega_rad_s": qd[seg.angle_index],
            })
        return out
