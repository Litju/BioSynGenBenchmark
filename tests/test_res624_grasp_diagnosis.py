"""RES-624 Gate A: instrumentation, contact-feasibility LP and RED diagnosis checks."""
import math
import unittest

try:
    import mujoco
    import numpy as np
    import scipy  # noqa: F401
except ImportError:
    mujoco = None

BOX_ON_FLOOR = """<mujoco><option timestep="0.001" gravity="0 0 -9.81" cone="elliptic"/>
<worldbody><geom name="floor" type="plane" size="2 2 .1" friction="{mu} .005 .0001"/>
<body name="box" pos="0 0 .0999"><freejoint/><geom name="b" type="box" size=".1 .1 .1" mass="{mass}"/></body>
</worldbody></mujoco>"""

PENDULUM = """<mujoco><option gravity="0 0 -9.81"/>
<worldbody><body name="arm"><joint name="hinge" type="hinge" axis="0 1 0"/>
<geom type="capsule" fromto="0 0 0 .5 0 0" size=".01" mass="0" contype="0" conaffinity="0"/>
<body pos=".5 0 0"><inertial pos="0 0 0" mass="2" diaginertia="1e-4 1e-4 1e-4"/></body></body></worldbody>
<actuator><motor joint="hinge" ctrlrange="-{cap} {cap}" ctrllimited="true" forcerange="-{cap} {cap}" forcelimited="true"/></actuator>
</mujoco>"""


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class ContactFeasibilityLP(unittest.TestCase):
    def test_object_supported_by_two_upward_contacts_splits_weight(self):
        from biosyngen.spatial.grasp_mechanics import object_wrench_feasibility
        r = object_wrench_feasibility([(-.25, 0, 0), (.25, 0, 0)], [(0, 0, 1)]*2, [.5, .5], (0, 0, 0), 20)
        self.assertTrue(r["feasible"])
        np.testing.assert_allclose(r["normal_forces_n"], [98.1, 98.1], rtol=1e-9)

    def test_single_downward_or_lateral_contacts_cannot_support(self):
        from biosyngen.spatial.grasp_mechanics import object_wrench_feasibility
        self.assertFalse(object_wrench_feasibility([(0, 0, .01)], [(0, 0, -1)], [1.], (0, 0, 0), 20)["feasible"])
        self.assertFalse(object_wrench_feasibility([(.01, 0, 0)], [(-1, 0, 0)], [1.], (0, 0, 0), 20)["feasible"])
        self.assertEqual(object_wrench_feasibility([], [], [], (0, 0, 0), 20)["code"], "NO_CONTACTS")

    def test_frictional_squeeze_threshold_is_exact_two_mu_n_equals_w(self):
        from biosyngen.spatial.grasp_mechanics import object_wrench_feasibility
        w, mu = 196.2, .6
        threshold = w/(2*mu)
        for factor, expected in ((1.001, True), (.999, False)):
            r = object_wrench_feasibility([(.014, 0, 0), (-.014, 0, 0)], [(-1, 0, 0), (1, 0, 0)], [mu, mu],
                                          (0, 0, 0), 20, max_normal_n=factor*threshold)
            self.assertEqual(r["feasible"], expected, factor)

    def test_friction_moment_about_cylinder_axis_is_enforced(self):
        """Two contacts whose friction must spin a free cylinder cannot hold it."""
        from biosyngen.spatial.grasp_mechanics import object_wrench_feasibility
        rad = .014
        # Contacts at the side (normal +x into bar) and on top (normal -z): only friction can lift,
        # and the two friction forces then produce an unbalanced axial moment.
        pts = [(-rad, 0, 0), (0, 0, rad)]
        r = object_wrench_feasibility(pts, [(1, 0, 0), (0, 0, -1)], [1., 1.], (0, 0, 0), 20)
        self.assertFalse(r["feasible"])
        self.assertGreater(r["residual_force_n"]+r["residual_moment_n_m"], 1)

    def test_generalized_certificate_box_on_floor(self):
        from biosyngen.spatial.grasp_mechanics import candidate_contacts, generalized_equilibrium
        m = mujoco.MjModel.from_xml_string(BOX_ON_FLOOR.format(mu=.8, mass=5))
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        cert = generalized_equilibrium(m, d, candidate_contacts(m, d, ["b"], .001))
        self.assertEqual(cert["code"], "STATIC_EQUILIBRIUM_FEASIBLE")
        self.assertAlmostEqual(sum(cert["contact_normal_n"]), 5*9.81, places=6)
        d.qpos[2] = .5
        mujoco.mj_forward(m, d)
        self.assertEqual(generalized_equilibrium(m, d, candidate_contacts(m, d, ["b"], .001))["code"],
                         "STATIC_GRASP_INFEASIBLE")

    def test_generalized_certificate_actuator_capacity(self):
        from biosyngen.spatial.grasp_mechanics import generalized_equilibrium
        need = 2*9.81*.5
        for cap, code in ((need*1.01, "STATIC_EQUILIBRIUM_FEASIBLE"), (need*.99, "ACTUATOR_INFEASIBLE")):
            m = mujoco.MjModel.from_xml_string(PENDULUM.format(cap=cap))
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            cert = generalized_equilibrium(m, d, [])
            self.assertEqual(cert["code"], code)
            if code.startswith("STATIC_EQ"):
                self.assertAlmostEqual(abs(cert["actuator_force"][0]), need, places=6)


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class RedDiagnosis(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from scripts.res624_grasp_diagnosis import _fresh_candidate, instrumented_hold
        from biosyngen.spatial.grasp_diagnostics import diagnose_initial_support, support_chronology
        from biosyngen.spatial.receiving_v2 import forward_hold
        plant, _ = _fresh_candidate()
        cls.diag = diagnose_initial_support(plant)
        plant, _ = _fresh_candidate()
        cls.rec, cls.final = instrumented_hold(plant, .3)
        cls.chron = support_chronology(cls.rec.records)
        ref_plant, _ = _fresh_candidate()
        cls.ref = forward_hold(ref_plant, .3)

    def test_recorder_is_passive_bitwise(self):
        self.assertEqual(self.final["final_bar_z_m"], self.ref["final_bar_z_m"])
        self.assertEqual(self.final["final_pelvis_z_m"], self.ref["final_pelvis_z_m"])

    def test_original_red_drop_reproduced(self):
        drop = self.rec.records[0]["bar"]["position"][2]-self.rec.records[-1]["bar"]["position"][2]
        # Windows 0.376053 m and Linux 0.376222 m were both historically observed.
        self.assertGreater(drop, .37)
        self.assertLess(drop, .383)
        self.assertEqual(self.rec.records[-1]["bar"]["active_hand_contacts"], 0)

    def test_root_cause_is_unsupportable_topology_not_friction_or_saturation(self):
        self.assertEqual(self.diag["failure_class"], "INCORRECT_RELATIVE_ORIENTATION_NO_SUPPORTING_CONTACT_TOPOLOGY")
        self.assertTrue(self.diag["any_eligible_pair"])
        self.assertIn("l_palm_collision", self.diag["pads_within_margin"])
        self.assertGreater(self.diag["topology_within_margin"]["residual_force_n"], 100)
        for elevation in self.diag["palm_normal_elevation_deg"].values():
            self.assertLess(elevation, 15)
        first = self.rec.records[0]
        self.assertLess(first["bar"]["support_ratio"], 0)  # hand contacts push the bar down
        self.assertEqual(first["saturation_count"], 0)

    def test_chronology_first_hand_contact_loss(self):
        self.assertEqual(self.chron["first_zero_hand_contact_step"], 7)
        self.assertLess(self.chron["initial_support_ratio"], 0)
        self.assertEqual(self.chron["final_hand_contacts"], 0)
        self.assertNotEqual(self.chron["state"], "SUSTAINED")

    def test_energy_bookkeeping_is_finite_and_bar_gains_downward_momentum(self):
        last = self.rec.records[-1]
        self.assertTrue(all(math.isfinite(last["energy"][k]) for k in last["energy"]))
        self.assertLess(last["bar"]["linear_momentum"][2], -40)


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class BrokenContactControls(unittest.TestCase):
    def test_ineligible_shaft_is_classified(self):
        from scripts.res624_grasp_diagnosis import control_ineligible
        r = control_ineligible()
        self.assertEqual(r["observed"], r["expected"])
        self.assertEqual(r["hand_contacts_any_step"], 0)

    def test_unreachable_bar_is_classified(self):
        from scripts.res624_grasp_diagnosis import control_unreachable
        r = control_unreachable()
        self.assertEqual(r["observed"], r["expected"])

    def test_hidden_external_support_is_not_counted_as_grasp(self):
        from scripts.res624_grasp_diagnosis import control_hidden_support
        r = control_hidden_support()
        self.assertEqual(r["observed"], r["expected"])
        self.assertEqual(r["max_contact_support_ratio"], 0.0)

    def test_positive_floor_support_is_detected(self):
        from scripts.res624_grasp_diagnosis import control_plated_floor
        r = control_plated_floor()
        self.assertEqual(r["observed"], "SUSTAINED")
        self.assertAlmostEqual(r["final_support_ratio_mean"], 1.0, places=3)


if __name__ == "__main__":
    unittest.main()
