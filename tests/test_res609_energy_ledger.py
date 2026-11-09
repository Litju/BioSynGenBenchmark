"""RES-609: implicit discrete-gradient compliant contact and independent energy ledger.

Analytical oracles: the discrete-gradient identity F_dg (g1 - g0) = U(g0) - U(g1);
a 1 kg block sliding on a compliant floor must lose exactly its kinetic energy
to friction and stop near v0^2 / (2 mu g); a frictionless, undamped bounce must
conserve KE + PE + U. Falsifiers: the historical explicit scheme visibly creates
energy on the stiff friction fixture, and tampered exports are rejected by the
independent evaluator.
"""
from __future__ import annotations

from copy import deepcopy
import math
import unittest

from biosyngen.compliant_contact import (
    EXPLICIT_INTERVAL_START_V1,
    IMPLICIT_DISCRETE_GRADIENT_V2,
    CompliantPlanarStepper,
    HuntCrossleyContact,
)
from biosyngen.multibody import DynamicsError, PlanarMultibodySystem, RigidBodyState
from tests.test_i2c_multibody import body, point_contact

G = 9.81


def block(law, scheme, *, v0=(0.0, 0.0), height=0.0, dt=.0005, duration=.6):
    system = PlanarMultibodySystem((body("mass", 1, 1),), (),
        (point_contact("floor", "mass", (0, 0), (0, 0), side="LEFT", channel="FZ_LEFT"),),
        gravity_m_s2=G, clock_id="manufactured-block")
    stepper = CompliantPlanarStepper(system, {"floor": law}, scheme=scheme)
    rest_depth = (G / law.stiffness) ** (1 / law.exponent) if height == 0.0 else 0.0
    state = system.make_state({"mass": RigidBodyState((0, height - rest_depth), 0, v0)})
    work = dissipated = 0.0
    trace = []
    for _ in range(round(duration / dt)):
        step, responses = stepper.advance(state, dt)
        r = responses["floor"]
        dissipated += r.dissipated_power_w * dt
        if r.contact_work_j is not None:
            work += r.contact_work_j
        state = step.state
        trace.append(state.bodies["mass"])
    return state, system, law, work, dissipated, trace, stepper


def mechanical(state, law, system):
    b = state.bodies["mass"]
    kin = system.contact_kinematics(state, "floor")
    return .5 * (b.velocity_m_s[0] ** 2 + b.velocity_m_s[1] ** 2) + G * b.position_m[1] + law.potential(kin.gap_m)


class DiscreteGradientLaw(unittest.TestCase):
    def test_discrete_gradient_identity(self):
        law = HuntCrossleyContact(damping_s_m=0.0, friction=0.0)
        for g0, g1 in ((-.01, -.012), (.002, -.004), (-.02, .001), (-.003, -.003000001)):
            normal, tangent = law.implicit_force(g0, g1, 0.0, 0.0)
            self.assertEqual(tangent, 0.0)
            if abs(g1 - g0) > 1e-9:
                self.assertAlmostEqual(normal * (g1 - g0), law.potential(g0) - law.potential(g1), delta=1e-12)
            self.assertGreaterEqual(normal, 0.0)

    def test_nonadhesive_and_cone_at_interval_end(self):
        law = HuntCrossleyContact()
        for vn in (-5, 0, 1, 5):
            for vt in (-1, 0, 1):
                n, t = law.implicit_force(-.01, -.011, vn, vt)
                self.assertGreaterEqual(n, 0.0)
                self.assertLessEqual(abs(t), law.friction * n + 1e-12)
                self.assertLessEqual(t * vt, 0.0)

    def test_unknown_scheme_refused(self):
        system = PlanarMultibodySystem((body("mass", 1, 1),), (),
            (point_contact("floor", "mass", (0, 0), (0, 0), side="LEFT", channel="FZ_LEFT"),),
            gravity_m_s2=G, clock_id="x")
        with self.assertRaises(DynamicsError):
            CompliantPlanarStepper(system, {"floor": HuntCrossleyContact()}, scheme="FASTER_GUESS")


class SlidingBlock(unittest.TestCase):
    """mu = 0.8, epsilon = 0.01 m/s: explicit effective viscosity mu m g / eps = 785 N s/m."""

    law = HuntCrossleyContact(stiffness=200_000, exponent=1.0, damping_s_m=0.5, friction=0.8, slip_transition_m_s=.01)

    def test_implicit_friction_dissipates_kinetic_energy_and_stops(self):
        state, system, law, work, dissipated, trace, stepper = block(self.law, IMPLICIT_DISCRETE_GRADIENT_V2, v0=(2.0, 0.0))
        b = state.bodies["mass"]
        e0 = .5 * 4.0 + 0.0
        self.assertLess(abs(b.velocity_m_s[0]), 1e-3)
        # Regularized Coulomb: stopping distance within 2 % of v0^2/(2 mu g).
        expected = 4.0 / (2 * .8 * G)
        self.assertAlmostEqual(b.position_m[0], expected, delta=.02 * expected)
        self.assertGreater(dissipated, 0.0)
        # Exact discrete channel identity and an energy decrease, never creation.
        stored0 = law.potential(-(G / law.stiffness))
        self.assertAlmostEqual(work + (law.potential(system.contact_kinematics(state, "floor").gap_m) - stored0),
                               -dissipated, delta=1e-9)
        self.assertLess(mechanical(state, law, system), e0 + 1e-9)
        self.assertEqual(stepper.counters["max_corrections_used"] <= stepper.max_corrections, True)

    def test_explicit_red_creates_energy_or_chatters(self):
        # Retained RED: at the same step the historical scheme's friction is
        # beyond its explicit stability bound (h c / m = 0.39 < 2 here, so use
        # a larger step where h c / m = 3.1 > 2) and the block does not settle.
        state, *_ = block(self.law, EXPLICIT_INTERVAL_START_V1, v0=(2.0, 0.0), dt=.004)
        implicit, *_ = block(self.law, IMPLICIT_DISCRETE_GRADIENT_V2, v0=(2.0, 0.0), dt=.004)
        self.assertGreater(abs(state.bodies["mass"].velocity_m_s[0]), 1e-2)
        self.assertLess(abs(implicit.bodies["mass"].velocity_m_s[0]), 1e-3)


class ElasticBounce(unittest.TestCase):
    """Undamped, frictionless bounce from 0.1 m: E0 = m g h exactly.

    The symplectic-Euler step itself dissipates D_num = sum 1/2 m |dv|^2.
    V2 makes that the *only* discrepancy: E + D_num = E0 to round-off. It is
    first order and always a loss, so V2 is accountable, not energy-exact.
    V1 keeps elastic energy bounded but its work channel violates the identity.
    """

    def identity(self, scheme, dt):
        law = HuntCrossleyContact(stiffness=10_000, exponent=1.0, damping_s_m=0.0, friction=0.0)
        state, system, law_, _, _, trace, _ = block(law, scheme, height=.1, dt=dt, duration=.6)
        previous, numerical = 0.0, 0.0
        for b in trace:
            numerical += .5 * (b.velocity_m_s[1] - previous) ** 2
            previous = b.velocity_m_s[1]
        change = mechanical(state, law_, system) - G * .1
        return change, numerical

    def test_v2_identity_exact_and_first_order_numerical_dissipation(self):
        losses = []
        for dt in (.001, .0005, .00025):
            change, numerical = self.identity(IMPLICIT_DISCRETE_GRADIENT_V2, dt)
            self.assertAlmostEqual(change + numerical, 0.0, delta=1e-9)
            self.assertLessEqual(change, 1e-12)        # never energy creation
            losses.append(numerical)
        for coarse, fine in zip(losses, losses[1:]):
            self.assertAlmostEqual(coarse / fine, 2.0, delta=.25)

    def test_v1_work_channel_violates_identity(self):
        change, numerical = self.identity(EXPLICIT_INTERVAL_START_V1, .001)
        self.assertGreater(abs(change + numerical), .05)


class DropJumpIndependentLedger(unittest.TestCase):
    """Articulated 0.5 s drop: first impact, rebound and second flight."""

    @classmethod
    def setUpClass(cls):
        from biosyngen.i2.drop_jump import build_drop_jump_candidate
        from biosyngen.i2.export import export_submission
        cls.cases = {}
        for scheme in (EXPLICIT_INTERVAL_START_V1, IMPLICIT_DISCRETE_GRADIENT_V2):
            scenario = build_drop_jump_candidate(duration_s=.5)
            trial = scenario.run(.0004, compliant_scheme=scheme)
            law = scenario.contact_law
            system = scenario.model.system
            cids = [c for ids in scenario.model.contact_ids.values() for c in ids]
            final = trial.frames[-1].state
            cls.cases[scheme] = dict(
                submission=export_submission(trial), trial=trial,
                joint_work=sum(f.joint_work_discrete_j for f in trial.frames),
                declared=sum(sum(f.contact_dissipated_power_w.values()) * (f.t_end_s - f.t_start_s) for f in trial.frames),
                stored=sum(law.potential(system.contact_kinematics(final, c).gap_m) for c in cids),
                projection=sum(f.constraint_projection_work_j for f in trial.frames))

    def ledger(self, scheme, submission=None, declared=None):
        from biosyngen_benchmark.i2_evaluator import discrete_energy_ledger
        c = self.cases[scheme]
        return discrete_energy_ledger(
            submission or c["submission"], groups=("FOOT_LEFT", "FOOT_RIGHT"),
            joint_work_discrete_j=c["joint_work"], initial_stored_energy_j=0.0,
            final_stored_energy_j=c["stored"], numerical_projection_work_j=c["projection"],
            declared_contact_dissipation_j=c["declared"] if declared is None else declared)

    def test_v1_red_energy_creation_and_v2_green(self):
        red = self.ledger(EXPLICIT_INTERVAL_START_V1)
        green = self.ledger(IMPLICIT_DISCRETE_GRADIENT_V2)
        self.assertEqual(red["passive_contact"]["outcome"], "REJECTED")
        self.assertGreater(red["passive_contact"]["contact_work_j"], 30.0)
        self.assertEqual(red["declared_dissipation"]["outcome"], "REJECTED")
        self.assertEqual(green["passive_contact"]["outcome"], "PASS")
        self.assertEqual(green["declared_dissipation"]["outcome"], "PASS")
        self.assertLess(green["passive_contact"]["contact_work_j"], -10.0)
        self.assertLess(abs(green["ledger"]["closure_residual_j"]), .1)
        self.assertEqual(green["ledger"]["outcome"], "NOT_EVALUABLE")

    def test_tampered_force_and_flipped_sign_are_rejected(self):
        base = self.cases[IMPLICIT_DISCRETE_GRADIENT_V2]["submission"]
        scaled = deepcopy(base)
        for frame in scaled["frames"]:
            for c in frame["contacts"]:
                c["fx"] *= 1.5
                c["fz"] *= 1.5
        self.assertEqual(self.ledger(IMPLICIT_DISCRETE_GRADIENT_V2, scaled)["declared_dissipation"]["outcome"], "REJECTED")
        flipped = deepcopy(base)
        for frame in flipped["frames"]:
            for c in frame["contacts"]:
                c["fx"], c["fz"] = -c["fx"], -c["fz"]
        self.assertEqual(self.ledger(IMPLICIT_DISCRETE_GRADIENT_V2, flipped)["passive_contact"]["outcome"], "REJECTED")

    def test_nonfinite_and_discontinuous_evidence_refused(self):
        from biosyngen_benchmark.i2_evaluator import discrete_energy_ledger
        c = self.cases[IMPLICIT_DISCRETE_GRADIENT_V2]
        bad = discrete_energy_ledger(c["submission"], groups=("FOOT_LEFT",), joint_work_discrete_j=math.nan,
                                     initial_stored_energy_j=0.0, final_stored_energy_j=0.0, numerical_projection_work_j=0.0)
        self.assertEqual(bad["outcome"], "INVALID")
        gap = deepcopy(c["submission"])
        gap["frames"].pop(len(gap["frames"]) // 2)
        self.assertIn(self.ledger(IMPLICIT_DISCRETE_GRADIENT_V2, gap).get("outcome"), ("INVALID",))

    def test_completion_is_not_claimed(self):
        # This 0.5 s window ends in second flight; it is not a completed drop jump.
        trial = self.cases[IMPLICIT_DISCRETE_GRADIENT_V2]["trial"]
        self.assertEqual(trial.frames[-1].controller_stage, "FLIGHT")


if __name__ == "__main__":
    unittest.main()
