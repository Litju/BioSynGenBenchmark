"""RES-624 Gate C: isolated contact-only V2 two-hand grasp, positive and negative."""
import unittest

try:
    import mujoco
    import numpy as np
    import scipy  # noqa: F401
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class IsolatedGrasp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from scripts.res624_isolated_grasp import detailed
        from biosyngen.spatial.grasp_fixture import FixtureSpec
        cls.fx, cls.rec, cls.held = detailed(FixtureSpec(tilt_deg=90), 1.0, 1.5)

    def test_fixture_has_no_bar_weld_connect_or_external_load(self):
        m = self.fx.model
        self.assertEqual(m.neq, 0)
        bar = self.fx.body_id("barbell")
        self.assertEqual(m.body_jntnum[bar], 1)
        self.assertEqual(m.jnt_type[m.body_jntadr[bar]], mujoco.mjtJoint.mjJNT_FREE)
        self.assertFalse(np.any(self.fx.data.xfrc_applied))
        self.assertEqual(m.nu, 6)
        self.assertTrue(np.all(m.actuator_forcelimited))

    def test_red_orientation_bar_is_carried_by_hand_contacts(self):
        h = self.held
        self.assertEqual(h["classification"]["outcome"], "GRASP_SUPPORTED")
        self.assertAlmostEqual(h["hand_support_ratio_hold"]["mean"], 1.0, places=5)
        # Load distribution emerges from contact geometry (not prescribed); symmetric fixture.
        self.assertAlmostEqual(h["load_share_left_n"]+h["load_share_right_n"], 196.2, places=3)
        self.assertAlmostEqual(h["load_share_left_n"], 98.1, delta=.5)
        self.assertGreater(h["min_normal_force_n"], 0)
        self.assertLess(h["max_cone_utilization"], 1)
        self.assertEqual(h["oracle_cone_problems"], 0)

    def test_independent_bar_newton_euler_and_mount_reaction(self):
        h = self.held
        # Implicit-in-velocity integrator: the step impulse of exported loads equals dP to rounding.
        self.assertLess(h["oracle_bar_newton_euler"]["max_step_linear_residual_n_s"], 1e-10)
        self.assertLess(h["oracle_bar_newton_euler"]["max_step_angular_residual_n_m_s"], 1e-4)
        r = h["mount_reaction_final"]
        self.assertAlmostEqual(r["sum_vertical_n"], r["analytic_static_n"], delta=1e-3)

    def test_static_certificate_feasible_and_refuses_reduced_capacity(self):
        from scripts.res624_isolated_grasp import certificate
        self.assertTrue(certificate(self.fx, 1.0)["feasible"])
        self.assertEqual(certificate(self.fx, .1)["code"], "ACTUATOR_INFEASIBLE")

    def test_axial_rotation_is_bounded(self):
        self.assertLess(self.held["max_axial_spin_rad_s"], .1)

    def test_undeclared_force_is_caught_by_oracle(self):
        from biosyngen.spatial.grasp_fixture import FixtureSpec, run_trial
        from biosyngen_benchmark.spatial_oracle import dynamic_balance
        _, _, frames, *_ = run_trial(FixtureSpec(tilt_deg=90), 1.0, .2, push_n=(.1, .2, (0, 0, 50.0)))
        for f in frames:
            f["external"] = []  # hide the applied load from the oracle
        r = dynamic_balance(frames, .001, members={"barbell"})
        self.assertAlmostEqual(r["max_step_linear_residual_n_s"], 50*.001, delta=1e-6)


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class GraspFailures(unittest.TestCase):
    def trial_outcome(self, **kw):
        from biosyngen.spatial.grasp_fixture import FixtureSpec, classify_trial, run_trial

        class Null:
            def record(self):
                return None
        grip = kw.pop("grip")
        _, _, _, pos, hc, ts = run_trial(FixtureSpec(**kw), grip, 1.5, record_every=10**9, recorder=Null())
        return classify_trial(pos, hc, ts), pos

    def test_insufficient_grip_is_a_genuine_slip_to_the_floor(self):
        c, pos = self.trial_outcome(tilt_deg=90, grip=.3)
        self.assertEqual(c["outcome"], "GRASP_SLIP")
        self.assertAlmostEqual(pos[-1][2], .025, delta=.002)

    def test_mass_and_friction_change_the_support_envelope(self):
        self.assertEqual(self.trial_outcome(tilt_deg=90, grip=.6)[0]["outcome"], "GRASP_SUPPORTED")
        self.assertEqual(self.trial_outcome(tilt_deg=90, grip=.6, friction=.5)[0]["outcome"], "GRASP_SLIP")
        self.assertEqual(self.trial_outcome(tilt_deg=90, grip=.6, bar_mass_kg=30, friction=.5)[0]["outcome"], "GRASP_SLIP")
        self.assertEqual(self.trial_outcome(tilt_deg=90, grip=1.0, bar_mass_kg=30, friction=.5)[0]["outcome"], "GRASP_SUPPORTED")

    def test_palm_up_rolling_cylinder_needs_grip_when_tilted(self):
        self.assertEqual(self.trial_outcome(tilt_deg=0, grip=0.)[0]["outcome"], "GRASP_SUPPORTED")
        self.assertEqual(self.trial_outcome(tilt_deg=30, grip=0.)[0]["outcome"], "GRASP_SLIP")

    def test_controlled_release_loses_support_and_free_falls(self):
        from biosyngen.spatial.grasp_fixture import FixtureSpec, run_trial
        _, rec, *_ = run_trial(FixtureSpec(tilt_deg=90), 1.0, 1.3, release_at_s=1.0)
        before = [r for r in rec.records if .6 < r["t"] < 1.0]
        self.assertTrue(all(r["bar"]["active_hand_contacts"] > 0 for r in before))
        free = [r for r in rec.records if r["t"] > 1.05 and r["bar"]["active_contacts"] == 0]
        self.assertGreater(len(free), 100)
        v = np.array([r["bar"]["linear_velocity"][2] for r in free])
        t = np.array([r["t"] for r in free])
        self.assertAlmostEqual(np.polyfit(t, v, 1)[0], -9.81, delta=.01)


if __name__ == "__main__":
    unittest.main()
