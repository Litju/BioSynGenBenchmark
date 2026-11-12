"""RES-624 Gate A: grasp-contact identity and load-bearing support falsifiers.

The evaluator must qualify a genuine contact-only overhead hold and refuse
every demonstrated false-support mechanism for the correct physical reason.
The contract is also bound to the compiled Human V2 model here, so generator
drift becomes a test failure instead of a silent acceptance change.
"""
import unittest

try:
    import mujoco
    import numpy as np  # noqa: F401
    import scipy  # noqa: F401
except ImportError:
    mujoco = None


class ContractIsVersionedAndExplicit(unittest.TestCase):
    def test_contract_is_not_a_prefix_rule(self):
        try:
            from biosyngen_benchmark import grasp_contract as c
        except ImportError as exc:  # numpy is only in the spatial jobs
            self.skipTest(str(exc))
        self.assertEqual(c.CONTRACT_VERSION, "RES624_GRASP_CONTACT_CONTRACT_V1")
        self.assertEqual(len(c.GRASP_GEOMS), 8)
        for seg in ("l_forearm", "r_forearm", "l_thigh", "r_thigh", "l_shank", "lumbar", "thorax", "l_upper_arm"):
            self.assertNotIn(seg, c.GRASP_BODIES)
            self.assertFalse(any(g.startswith(seg) for g in c.GRASP_GEOMS))
        for side in "lr":
            self.assertIn(f"{side}_palm_collision", c.GRASP_GEOMS)
            self.assertIn(f"{side}_finger_flex_pad", c.GRASP_GEOMS)
            self.assertIn(f"{side}_finger_distal_pad", c.GRASP_GEOMS)
            self.assertIn(f"{side}_thumb_opposition_pad", c.GRASP_GEOMS)

    def test_labelled_segment_contact_is_not_grasp_support(self):
        try:
            from biosyngen_benchmark.grasp_contract import classify_frame
        except ImportError as exc:
            self.skipTest(str(exc))
        frame = dict(t=0.0, gravity=[0, 0, -9.81],
                     bodies=[dict(name="barbell", mass=20.0, com=[0, 0, 1.5]),
                             dict(name="l_forearm", mass=2.0, com=[0, .2, 1.5])],
                     contacts=[dict(body1="barbell", body2="l_forearm", geom1="bar_shaft", geom2="l_forearm_collision",
                                    point=[0, .2, 1.5], normal=[0, 0, 1], force_on_body2=[0, 0, 196.2],
                                    friction=1.0, dist=0.0)],
                     grips=[], external=[])
        ledger = classify_frame(frame)
        self.assertEqual(ledger["contacts"], [])
        self.assertEqual(len(ledger["nongrasp"]), 1)
        self.assertEqual(ledger["identity_problems"], [])


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class GraspIdentityAndSupport(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from scripts.res624_receiving_campaign import build_nominal, run_case
        _, cls.nominal = build_nominal()
        cls.frames, cls.static, plant = run_case("nominal_0p3s_red_window", None, None, .3, {}, cls.nominal)
        cls.dt = plant.model.opt.timestep
        cls.plant = plant

    def evaluate(self, frames, verdict=None):
        from biosyngen_benchmark.receiving_acceptance import evaluate_receiving
        return evaluate_receiving(frames, self.dt, verdict or self.static["verdict"])

    def test_contract_binds_to_compiled_model(self):
        from biosyngen_benchmark import grasp_contract as c
        m = self.plant.model
        for interface in c.GRASP_INTERFACES:
            gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, interface.geom)
            self.assertGreaterEqual(gid, 0, interface.geom)
            body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[gid]))
            self.assertEqual(body, interface.body, interface.geom)
        shaft = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, c.BAR_GRASP_GEOM)
        self.assertEqual(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[shaft])), c.BAR_BODY)
        # No registered grasp body may carry an additional bar-contact geom that
        # the contract does not know about: that would silently change the grip.
        known = set(c.GRASP_GEOMS)
        body_names = {i.body for i in c.GRASP_INTERFACES}
        governed = [g for g in range(m.ngeom)
                    if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) in body_names]
        for g in governed:
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
            if name is not None:
                self.assertIn(name, known)

    def test_nominal_hold_is_load_bearing(self):
        r = self.evaluate(self.frames)
        self.assertEqual(r["outcome"], "RECEIVING_HOLD_QUALIFIED", r.get("reason"))
        ev = r["evidence"]
        self.assertGreaterEqual(ev["min_grasp_contacts"], 2)
        self.assertGreaterEqual(ev["hand_support_impulse_ratio"], .98)
        self.assertGreater(ev["hand_support_share"]["l"], .02)
        self.assertGreater(ev["hand_support_share"]["r"], .02)
        self.assertLess(ev["hand_support_force_residual_ratio"], 1e-6)
        self.assertLess(ev["hand_support_moment_residual_ratio"], 1e-6)
        self.assertEqual(ev["grasp_identity_problems"], [])

    def test_external_support_with_5pct_hand_load_is_refused(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.redistribute_hand_load(self.frames, {"l": .05, "r": .05}))
        self.assertEqual(r["outcome"], "GRASP_UNSUPPORTED")
        self.assertAlmostEqual(r["evidence"]["hand_support_impulse_ratio"], .05, places=3)

    def test_forearm_and_thigh_and_torso_support_are_refused(self):
        from scripts import res624_adversarial_scenes as scenes
        for segment in ("forearm", "thigh", "torso"):
            r = self.evaluate(scenes.move_load_to_segment(self.frames, segment))
            self.assertEqual(r["outcome"], "GRASP_UNSUPPORTED", (segment, r.get("reason")))
            self.assertLess(r["evidence"]["hand_support_impulse_ratio"], .1)
            self.assertGreater(r["evidence"]["min_grasp_contacts"], 0)  # the fingers still touch

    def test_unilateral_support_is_refused(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.redistribute_hand_load(self.frames, {"l": 2., "r": 0.}))
        self.assertEqual(r["outcome"], "GRASP_NOT_BILATERAL")
        self.assertLess(r["evidence"]["hand_support_share"]["r"], .02)

    def test_asymmetric_bilateral_sharing_is_qualified(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.redistribute_hand_load(self.frames, {"l": .6, "r": 1.4}))
        self.assertEqual(r["outcome"], "RECEIVING_HOLD_QUALIFIED", r.get("reason"))
        self.assertAlmostEqual(r["evidence"]["hand_support_share"]["l"], .3, delta=.01)
        self.assertAlmostEqual(r["evidence"]["hand_support_share"]["r"], .7, delta=.01)

    def test_forged_identity_is_refused_before_support(self):
        from scripts import res624_adversarial_scenes as scenes
        for scene in (scenes.forge_grasp_geom_body(self.frames), scenes.forge_contact_point(self.frames)):
            r = self.evaluate(scene)
            self.assertEqual(r["outcome"], "GRASP_IDENTITY_UNADMISSIBLE", r.get("reason"))
            self.assertTrue(r["evidence"]["grasp_identity_problems"])

    def test_misleading_segment_contacts_without_real_grasp_are_refused(self):
        from scripts import res624_adversarial_scenes as scenes
        r = self.evaluate(scenes.segment_only(self.frames, "thigh"))
        self.assertEqual(r["outcome"], "GRASP_GEOMETRY_INFEASIBLE")
        self.assertEqual(r["evidence"]["min_grasp_contacts"], 0)
        self.assertGreater(r["evidence"]["nongrasp_bar_contacts_max"], 0)

    def test_misassigned_and_reversed_forces_are_refused(self):
        from scripts import res624_adversarial_scenes as scenes
        swapped = self.evaluate(scenes.swap_contact_sites(self.frames))
        self.assertEqual(swapped["outcome"], "NUMERICAL_NOT_QUALIFIED")
        self.assertTrue(any("ANGULAR" in str(f.get("reason")) for f in swapped["failures"]))
        reversed_ = self.evaluate(scenes.reverse_one_hand(self.frames, "l"))
        self.assertNotEqual(reversed_["outcome"], "RECEIVING_HOLD_QUALIFIED")


if __name__ == "__main__":
    unittest.main()
