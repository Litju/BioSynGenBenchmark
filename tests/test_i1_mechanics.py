"""RES-482 verification of the actuated planar jump mechanics.

Each reference is an independent identity or a closed form, not a copy of the
solver path:

* kinetic energy from segment velocities versus 0.5 q'^T M q';
* Newton/Euler momentum balance by central finite differences of momentum
  functions;
* static equilibrium versus the weight and the COM projection;
* closed-form activation and force-velocity properties.
"""

from __future__ import annotations

from dataclasses import replace
import math
import random
import unittest

from biosyngen.i1.actuation import (
    activation_derivative,
    force_velocity,
)
from biosyngen.i1.body import PF, PH, PS, PT, X, XB, Z, ZB, PlanarBody
from biosyngen.i1.control import ConstantStrategy, HoldStrategy
from biosyngen.i1.linalg import solve
from biosyngen.i1.params import ActuatorParameters, BarParameters, BodyParameters, NeuromuscularParameters
from biosyngen.i1.simulate import (
    ContactMode,
    InadmissibleScenario,
    JumpScenario,
    Posture,
    SolverSettings,
    _Engine,
    ContactState,
    simulate_jump,
    static_equilibrium,
)

D = math.radians
STANCE = Posture(D(86), D(92), D(88))


def random_state(rng: random.Random, body: PlanarBody) -> tuple[list[float], list[float]]:
    q = [rng.uniform(-0.2, 0.2), rng.uniform(0.0, 0.3), rng.uniform(-0.4, 0.4),
         rng.uniform(1.2, 1.9), rng.uniform(1.3, 2.3), rng.uniform(1.0, 1.8)]
    qd = [rng.uniform(-2, 2) for _ in range(6)]
    if body.n == 8:
        sx, sz = body.point(q, body.points["shoulder"])
        q += [sx + rng.uniform(-0.01, 0.01), sz + rng.uniform(-0.01, 0.01)]
        qd += [rng.uniform(-2, 2), rng.uniform(-2, 2)]
    return q, qd


def linear_momentum(body: PlanarBody, q, qd) -> tuple[float, float]:
    px = pz = 0.0
    for seg in body.segments:
        vx, vz = body.point_velocity(q, qd, seg.com_terms)
        px += seg.mass_kg * vx
        pz += seg.mass_kg * vz
    if body.bar is not None:
        px += body.bar.mass_kg * qd[XB]
        pz += body.bar.mass_kg * qd[ZB]
    return px, pz


class MassMatrixTests(unittest.TestCase):
    def test_mass_matrix_is_symmetric_positive_definite_and_matches_kinetic_energy(self) -> None:
        rng = random.Random(7)
        for bar in (None, BarParameters()):
            body = PlanarBody(BodyParameters(), bar)
            for _ in range(20):
                q, qd = random_state(rng, body)
                m, _ = body.dynamics_terms(q, qd)
                n = body.n
                for i in range(n):
                    for j in range(n):
                        self.assertAlmostEqual(m[i][j], m[j][i], places=12)
                quad = 0.5 * sum(qd[i] * m[i][j] * qd[j] for i in range(n) for j in range(n))
                self.assertAlmostEqual(quad, body.kinetic_energy(q, qd), delta=1e-10 * max(1.0, quad))
                self.assertGreater(quad, 0.0)
                # Cholesky succeeds only for a positive-definite matrix.
                lower = [[0.0] * n for _ in range(n)]
                for i in range(n):
                    for j in range(i + 1):
                        s = m[i][j] - sum(lower[i][k] * lower[j][k] for k in range(j))
                        if i == j:
                            self.assertGreater(s, 0.0)
                            lower[i][j] = math.sqrt(s)
                        else:
                            lower[i][j] = s / lower[j][j]

    def test_free_flight_obeys_newton_and_euler_for_any_internal_torques(self) -> None:
        # With only gravity and internal joint/coupling forces, dP/dt = -M g z
        # and dH_O/dt = -M g x_com. Momenta are differentiated numerically along
        # (q', q''), an independent path from the mass-matrix assembly.
        rng = random.Random(11)
        for bar in (None, BarParameters()):
            body = PlanarBody(BodyParameters(), bar)
            g = body.params.gravity_m_s2
            for _ in range(10):
                q, qd = random_state(rng, body)
                torques = (rng.uniform(-300, 300), rng.uniform(-300, 300), rng.uniform(-300, 300))
                m, f = body.dynamics_terms(q, qd)
                qj = body.joint_generalized_force(torques)
                qc = body.coupling_generalized_force(q, qd)
                qdd = solve(m, [f[i] + qj[i] + qc[i] for i in range(body.n)])
                eps = 1e-6
                qp = [q[i] + eps * qd[i] for i in range(body.n)]
                qm = [q[i] - eps * qd[i] for i in range(body.n)]
                vp = [qd[i] + eps * qdd[i] for i in range(body.n)]
                vm = [qd[i] - eps * qdd[i] for i in range(body.n)]
                ppx, ppz = linear_momentum(body, qp, vp)
                pmx, pmz = linear_momentum(body, qm, vm)
                self.assertAlmostEqual((ppx - pmx) / (2 * eps), 0.0, delta=1e-4)
                self.assertAlmostEqual((ppz - pmz) / (2 * eps), -body.total_mass_kg * g, delta=1e-4)
                hp = body.angular_momentum(qp, vp, (0.0, 0.0))
                hm = body.angular_momentum(qm, vm, (0.0, 0.0))
                com_x = body.system_com(q)[0]
                self.assertAlmostEqual(
                    (hp - hm) / (2 * eps), -body.total_mass_kg * g * com_x, delta=1e-4
                )


class StaticAndActuatorTests(unittest.TestCase):
    def test_static_equilibrium_supports_weight_with_cop_under_com(self) -> None:
        for bar in (None, BarParameters(mass_kg=30.0)):
            body = PlanarBody(BodyParameters(), bar)
            st = static_equilibrium(body, NeuromuscularParameters(), STANCE, 0.02)
            weight = body.total_mass_kg * body.params.gravity_m_s2
            self.assertAlmostEqual(st.heel_fz_n + st.toe_fz_n, weight, delta=1e-9 * weight)
            self.assertAlmostEqual(st.heel_fx_n, 0.0, delta=1e-9)
            self.assertAlmostEqual(st.cop_x_m, body.system_com(list(st.q))[0], delta=1e-12)

    def test_quiet_stance_hold_does_not_drift(self) -> None:
        traj = simulate_jump(
            JumpScenario("hold", STANCE, HoldStrategy(), settings=SolverSettings(horizon_s=0.3))
        )
        first, last = traj.samples[0], traj.samples[-1]
        self.assertEqual(traj.termination, "HORIZON_REACHED")
        self.assertLess(abs(last.body_com_m[1] - first.body_com_m[1]), 1e-9)
        self.assertLess(abs(last.grf_z_n - first.grf_z_n), 1e-6)
        self.assertTrue(all(s.mode is ContactMode.FLAT for s in traj.samples))

    def test_posture_with_com_outside_the_foot_is_refused(self) -> None:
        leaning = Posture(D(70), D(70), D(70))  # body tilted 20 deg forward
        with self.assertRaisesRegex(InadmissibleScenario, "tensile"):
            static_equilibrium(PlanarBody(BodyParameters()), NeuromuscularParameters(), leaning, 0.02)

    def test_posture_beyond_actuator_capacity_is_refused(self) -> None:
        weak = NeuromuscularParameters().scaled_strength(0.02)
        with self.assertRaisesRegex(InadmissibleScenario, "activation"):
            static_equilibrium(PlanarBody(BodyParameters()), weak, Posture(D(70), D(125), D(60)), 0.02)

    def test_activation_dynamics_stay_bounded_and_respect_time_constants(self) -> None:
        tau_a, tau_d = 0.015, 0.050
        a, dt = 0.01, 1e-5
        for _ in range(int(0.3 / dt)):
            rate = activation_derivative(1.0, a, tau_a, tau_d)
            self.assertLessEqual(rate, (1.0 - a) / (0.5 * tau_a) + 1e-9)
            a += dt * rate
            self.assertLessEqual(a, 1.0)
        self.assertGreater(a, 0.99)
        for _ in range(int(0.5 / dt)):
            a += dt * activation_derivative(0.01, a, tau_a, tau_d)
            self.assertGreaterEqual(a, 0.01)
        self.assertLess(a, 0.02)

    def test_force_velocity_relation_closed_form_properties(self) -> None:
        p = ActuatorParameters(100.0, 1.0, 1.0, 20.0)
        self.assertEqual(force_velocity(p, 0.0), 1.0)
        self.assertEqual(force_velocity(p, 1.0), 0.0)
        self.assertEqual(force_velocity(p, 2.0), 0.0)
        h = 1e-7
        left = (force_velocity(p, 0.0) - force_velocity(p, -h)) / h
        right = (force_velocity(p, h) - force_velocity(p, 0.0)) / h
        self.assertAlmostEqual(left, right, delta=1e-4 * abs(right))
        self.assertAlmostEqual(right, -(1.0 + 1.0 / p.hill_curvature), delta=1e-4 * abs(right))
        self.assertAlmostEqual(force_velocity(p, -1e6), p.eccentric_max_ratio, delta=1e-4)
        values = [force_velocity(p, v / 100.0) for v in range(-300, 101)]
        self.assertTrue(all(a >= b for a, b in zip(values, values[1:])))


class ContactKinematicsTests(unittest.TestCase):
    def test_flat_contact_accelerations_satisfy_constraints_and_newton(self) -> None:
        scenario = JumpScenario("probe", STANCE, ConstantStrategy((0.4, 0.05, 0.6, 0.05, 0.5, 0.05)))
        eng = _Engine(scenario)
        y = eng.initial_state()
        n = eng.n
        rng = random.Random(3)
        for i in range(PF, n):
            y[n + i] = 0.0
        # Give the posture a small consistent motion: the shank, thigh and HAT
        # rotate; the foot stays flat, so heel/toe velocities stay zero.
        for i in (PS, PT, PH):
            y[n + i] = rng.uniform(-1.0, 1.0)
        knee = eng.body.points["knee"]
        cs = ContactState(ContactMode.FLAT, 0, eng.body.point(y[:n], eng.heel_terms)[0])
        out = eng.evaluate(0.0, y, cs)
        q, qd, qdd = y[:n], y[n : 2 * n], out["qdd"]
        for terms in (eng.heel_terms, eng.toe_terms):
            jx, jz, bx, bz = eng.body.point_jacobian(q, qd, terms)
            ax = sum(v * qdd[k] for k, v in jx.items()) + bx
            az = sum(v * qdd[k] for k, v in jz.items()) + bz
            self.assertAlmostEqual(ax, 0.0, delta=1e-8)
            self.assertAlmostEqual(az, 0.0, delta=1e-8)
        eps = 1e-6
        qp = [q[i] + eps * qd[i] for i in range(n)]
        qm = [q[i] - eps * qd[i] for i in range(n)]
        vp = [qd[i] + eps * qdd[i] for i in range(n)]
        vm = [qd[i] - eps * qdd[i] for i in range(n)]
        ppx, ppz = linear_momentum(eng.body, qp, vp)
        pmx, pmz = linear_momentum(eng.body, qm, vm)
        fx = out["heel_force"][0] + out["toe_force"][0]
        fz = out["heel_force"][1] + out["toe_force"][1]
        weight = eng.body.total_mass_kg * eng.body.params.gravity_m_s2
        self.assertAlmostEqual((ppx - pmx) / (2 * eps), fx, delta=1e-3)
        self.assertAlmostEqual((ppz - pmz) / (2 * eps), fz - weight, delta=1e-3)
        self.assertIsNotNone(knee)


if __name__ == "__main__":
    unittest.main()
