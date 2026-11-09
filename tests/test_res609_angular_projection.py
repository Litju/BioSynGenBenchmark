"""RES-609: angular-momentum-neutral drift projection, with the original RED retained.

The analytical oracle is a free-flying, spinning three-link chain under
uniform gravity: angular momentum about the moving COM is exactly conserved
by the continuous dynamics, so any change is numerical. The historical V1
projection must visibly fail (retained RED); V2 must close to round-off.
The full CMJ fixture is scored by the unchanged independent evaluator.
"""
from __future__ import annotations

import math
import unittest

from biosyngen.multibody import (
    PROJECTION_ANGULAR_NEUTRAL_V2,
    PROJECTION_MASS_METRIC_V1,
    DynamicsError,
    PlanarJoint,
    PlanarMultibodySystem,
    RigidBodyState,
)
from tests.test_i2c_multibody import body


def chain(policy):
    bodies = (body("a", 4.0, .05), body("b", 3.0, .03), body("c", 2.0, .02))
    joints = (PlanarJoint("ab", "a", "b", (.25, 0.0), (-.2, 0.0)),
              PlanarJoint("bc", "b", "c", (.2, 0.0), (-.15, 0.0)))
    return PlanarMultibodySystem(bodies, joints, (), gravity_m_s2=9.81, clock_id="RES609_TEST",
                                 projection_policy=policy)


def spinning_state(system):
    # Consistent straight chain rotating rigidly at 6 rad/s about its COM.
    xs = {"a": 0.0, "b": .45, "c": .80}
    masses = {"a": 4.0, "b": 3.0, "c": 2.0}
    com = sum(masses[k]*xs[k] for k in xs)/sum(masses.values())
    omega = 6.0
    states = {k: RigidBodyState((xs[k], 1.0), 0.0, (0.0, omega*(xs[k]-com) + 1.5), omega) for k in xs}
    return system.make_state(states)


def com_angular_momentum(system, state):
    return system.angular_momentum(state, system.center_of_mass(state))


def drift(policy, steps=400, dt=.002):
    system = chain(policy)
    state = spinning_state(system)
    start = com_angular_momentum(system, state)
    worst = 0.0
    for _ in range(steps):
        state = system.advance(state, dt).state
        worst = max(worst, abs(com_angular_momentum(system, state) - start))
    return worst, start, state, system


class AngularNeutralProjection(unittest.TestCase):
    def test_default_policy_is_historical(self):
        self.assertEqual(chain(PROJECTION_MASS_METRIC_V1).projection_policy, PROJECTION_MASS_METRIC_V1)
        system = PlanarMultibodySystem((body("z", 1.0, .1),), (), (), gravity_m_s2=9.81, clock_id="X")
        self.assertEqual(system.projection_policy, PROJECTION_MASS_METRIC_V1)

    def test_unknown_policy_is_refused(self):
        with self.assertRaises(DynamicsError):
            chain("SILENTLY_RELAXED")

    def test_v1_red_and_v2_green_free_flight(self):
        v1, start, _, _ = drift(PROJECTION_MASS_METRIC_V1)
        v2, start2, final, system = drift(PROJECTION_ANGULAR_NEUTRAL_V2)
        self.assertAlmostEqual(start, start2, places=12)
        self.assertGreater(abs(start), 1.0)
        # Retained RED: the historical correction changes L by a visible amount.
        self.assertGreater(v1, 1e-4)
        # GREEN: conserved to round-off, five orders below V1.
        self.assertLess(v2, 1e-10)
        # Joint closure is not sacrificed for the momentum row.
        for joint in system.joints:
            pa = system._point_data(joint.body_a_id, joint.local_anchor_a_m, final).position
            pb = system._point_data(joint.body_b_id, joint.local_anchor_b_m, final).position
            self.assertLess(math.dist(pa, pb), 1e-8)

    def test_linear_momentum_unchanged_by_repair(self):
        _, _, s1, sys1 = drift(PROJECTION_MASS_METRIC_V1, steps=50)
        _, _, s2, sys2 = drift(PROJECTION_ANGULAR_NEUTRAL_V2, steps=50)
        for a, b in zip(sys1.linear_momentum(s1), sys2.linear_momentum(s2)):
            self.assertAlmostEqual(a, b, places=9)

    def test_with_projection_policy_copies_model(self):
        system = chain(PROJECTION_MASS_METRIC_V1)
        other = system.with_projection_policy(PROJECTION_ANGULAR_NEUTRAL_V2)
        self.assertEqual(other.projection_policy, PROJECTION_ANGULAR_NEUTRAL_V2)
        self.assertEqual(system.projection_policy, PROJECTION_MASS_METRIC_V1)
        self.assertEqual(other.bodies, system.bodies)
        self.assertEqual(other.joints, system.joints)


class CmjEvaluatorReplay(unittest.TestCase):
    """Original 1 ms CMJ RED versus V2 under the unchanged evaluator budget."""

    def score(self, policy):
        from biosyngen.i2.export import export_submission
        from biosyngen.i2.jumps import build_jump
        from biosyngen_benchmark.i2_evaluator import evaluate_i2
        scenario = build_jump("SJ", duration_s=1.2)
        scenario.model.system = scenario.model.system.with_projection_policy(policy)
        trial = scenario.run(.001)
        submission = export_submission(trial)
        submission["claims"].append("ANGULAR_MOMENTUM_CLOSURE")
        return trial, evaluate_i2(submission)["findings"]

    def test_sj_red_green(self):
        _, red = self.score(PROJECTION_MASS_METRIC_V1)
        trial, green = self.score(PROJECTION_ANGULAR_NEUTRAL_V2)
        self.assertEqual(red["ANGULAR_MOMENTUM_CLOSURE"]["outcome"], "REJECTED")
        self.assertEqual(green["ANGULAR_MOMENTUM_CLOSURE"]["outcome"], "PASS")
        self.assertEqual(green["LINEAR_MOMENTUM_CLOSURE"]["outcome"], "PASS")
        self.assertEqual(trial.termination, "DURATION_REACHED")
        budget = green["ANGULAR_MOMENTUM_CLOSURE"]["tolerance_kg_m2_s"]
        self.assertLess(green["ANGULAR_MOMENTUM_CLOSURE"]["max_prefix_residual_kg_m2_s"], .2*budget)


if __name__ == "__main__":
    unittest.main()
