"""RES-610 spatial plant: positive controls and falsifiers.

Requires MuJoCo (pinned 3.3.7 in the dedicated CI job); skipped otherwise.
Covers mass/inertia, COM velocity frame, joint ranges, bar orientation,
grip closure, constraint overdetermination (bar spin row), contact normal,
nonadhesion, foot balance, omitted supports, actuator saturation, the
perturbed hold and deterministic replay. The independent oracle never
imports the generator; tampered exports must be rejected by it.
"""
from __future__ import annotations

from copy import deepcopy
import math
import unittest

try:
    import mujoco
    import numpy as np
except ImportError:  # pragma: no cover - exercised on stdlib-only runners
    mujoco = None


@unittest.skipIf(mujoco is None, "MuJoCo not installed")
class SpatialPlantModel(unittest.TestCase):
    def setUp(self):
        from biosyngen.spatial.plant import SpatialPlant
        self.plant = SpatialPlant.build()

    def test_mass_and_bar_inertia_match_declared_values(self):
        from biosyngen.spatial.model import AthleteBarbellSpec, bar_inertia, mass_table
        from biosyngen_benchmark.spatial_oracle import mass_properties
        spec = AthleteBarbellSpec()
        mujoco.mj_forward(self.plant.model, self.plant.data)
        frame = self.plant.export_frame()
        declared = {"barbell": spec.total_load_kg, **mass_table(spec)}
        self.assertAlmostEqual(self.plant.total_mass(), sum(mass_table(spec).values()) + spec.total_load_kg, places=9)
        self.assertAlmostEqual(sum(mass_table(spec).values()), spec.body_mass_kg, delta=.002 * spec.body_mass_kg)
        self.assertEqual(mass_properties(frame, declared, bar_inertia=bar_inertia(spec))["outcome"], "PASS")
        wrong = deepcopy(frame)
        bar = next(b for b in wrong["bodies"] if b["name"] == "barbell")
        bar["inertia_world"] = (np.asarray(bar["inertia_world"]) * 1.3).tolist()
        self.assertEqual(mass_properties(wrong, declared, bar_inertia=bar_inertia(spec))["outcome"], "REJECTED")
        self.assertEqual(mass_properties(frame, {**declared, "barbell": 61.0})["outcome"], "REJECTED")

    def test_body_velocity_is_com_velocity(self):
        m, d = self.plant.model, self.plant.data
        d.qvel[:] = np.random.default_rng(3).normal(size=m.nv)
        mujoco.mj_forward(m, d)
        for name in ("l_shank", "r_forearm", "torso", "barbell"):
            b = self.plant.body_id(name)
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp, jr, d.xipos[b], b)
            v, w = self.plant.body_velocity(b)
            np.testing.assert_allclose(v, jp @ d.qvel, atol=1e-12)
            np.testing.assert_allclose(w, jr @ d.qvel, atol=1e-12)

    def test_declared_ranges_are_compiled_and_violations_detected(self):
        from biosyngen.spatial.model import joint_specs
        from biosyngen.spatial.receiving import joint_range_violations
        m = self.plant.model
        for name, spec in joint_specs():
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.assertEqual(tuple(m.jnt_range[j]), (spec.lower, spec.upper))
        specs = dict(joint_specs())
        self.assertEqual(specs["l_elbow_flex"].upper, 2.60)      # planar limit retained
        self.assertEqual(specs["l_knee_flex"].upper, 2.50)
        self.assertEqual(specs["l_ankle_dorsi"].upper, .75)
        self.assertEqual(joint_range_violations(self.plant, {"l_elbow_flex": 2.7}), {"l_elbow_flex": 2.7})
        self.assertEqual(joint_range_violations(self.plant, {"l_elbow_flex": 2.5}), {})

    def test_spec_refuses_undeclared_or_nonphysical_parameters(self):
        from biosyngen.spatial.model import AthleteBarbellSpec
        for kwargs in ({"grip_model": "MAGNET_V1"}, {"body_mass_kg": -1}, {"grip_half_width_m": .9},
                       {"grip_site_distal_m": .3}, {"plate_mass_kg": math.nan}, {"timestep_s": True}):
            with self.assertRaises(ValueError):
                AthleteBarbellSpec(**kwargs)

    def test_grip_topologies_have_declared_constraint_counts(self):
        from biosyngen.spatial.model import AthleteBarbellSpec
        from biosyngen.spatial.plant import SpatialPlant
        self.assertEqual(self.plant.model.neq, 4)
        single = SpatialPlant.build(AthleteBarbellSpec(grip_model="BILATERAL_SINGLE_SITE_CONNECT_V1"))
        self.assertEqual(single.model.neq, 2)

    def test_deterministic_replay(self):
        from biosyngen.spatial.plant import SpatialPlant
        states = []
        for _ in range(2):
            plant = SpatialPlant.build()
            for _ in range(200):
                mujoco.mj_step(plant.model, plant.data)
            states.append(plant.data.qpos.copy())
        np.testing.assert_array_equal(states[0], states[1])


@unittest.skipIf(mujoco is None, "MuJoCo not installed")
class ReceivingConfigurations(unittest.TestCase):
    """Two solved configurations are shared; each solve takes ~10-30 s."""

    @classmethod
    def setUpClass(cls):
        from biosyngen.spatial.plant import SpatialPlant
        from biosyngen.spatial import receiving as R
        cls.R = R
        cls.solved = {}
        for receiving, depth in (("OVERHEAD", "POWER"), ("FRONT_RACK", "POWER")):
            plant = SpatialPlant.build(R.default_spec(receiving))
            x, closure, _ = R.solve_pose(plant, receiving, depth)
            certificate = R.static_equilibrium(plant, receiving, depth=depth)
            frame = plant.export_frame()
            frame["contacts"], frame["grips"] = certificate.contacts, certificate.grips
            cls.solved[receiving] = (plant, x, closure, certificate, frame)

    def test_feasible_closed_equilibria(self):
        from biosyngen_benchmark import spatial_oracle as O
        for receiving, (plant, x, closure, certificate, frame) in self.solved.items():
            with self.subTest(receiving=receiving):
                self.assertEqual(certificate.reason, "FEASIBLE")
                self.assertLess(closure["max_grip_separation_m"], self.R.POSE_TOLERANCE_M)
                self.assertLessEqual(certificate.max_torque_ratio, 1.0)
                self.assertEqual(self.R.depth_label(self.R.thigh_inclination(plant)), "POWER")
                self.assertLessEqual(self.R.trunk_inclination(plant), self.R.TRUNK_LIMIT_RAD)
                self.assertGreaterEqual(min(self.R.bar_clearances(plant).values()), self.R.BAR_CLEARANCE_M - 1e-6)
                for check in (O.static_balance(frame), O.static_balance(frame, members={"barbell"}),
                              O.contact_admissibility(frame), O.support_polygon(frame, margin_m=.03)):
                    self.assertEqual(check["outcome"], "PASS", check)
                # Bar orientation: axis along world y, no pitch or roll.
                rot = plant.data.xmat[plant.body_id("barbell")].reshape(3, 3)
                np.testing.assert_allclose(rot[:, 1], (0, 1, 0), atol=1e-12)

    def test_front_rack_load_path_uses_shelf_and_bar_spin_row_is_unsupported_overhead(self):
        _, _, closure, certificate, frame = self.solved["FRONT_RACK"]
        shelf = [c for c in certificate.contacts if "deltoid" in c["geom1"] + c["geom2"]]
        self.assertEqual(len(shelf), 2)
        bar_weight = 60.0 * 9.81
        self.assertGreater(sum(c["force_on_body2"][2] * (1 if c["geom2"] == "bar_shaft" else -1) for c in shelf),
                           .3 * bar_weight)
        _, _, _, overhead, _ = self.solved["OVERHEAD"]
        self.assertFalse(any("bar_shaft" in c["geom1"] + c["geom2"] for c in overhead.contacts))

    def test_tampered_exports_are_rejected(self):
        from biosyngen_benchmark import spatial_oracle as O
        frame = self.solved["FRONT_RACK"][4]
        flipped = deepcopy(frame)
        floor = [c for c in flipped["contacts"] if "floor" in (c["geom1"], c["geom2"])]
        floor[0]["normal"] = [0.0, 1.0, 0.0]
        self.assertEqual(O.contact_admissibility(flipped)["outcome"], "REJECTED")
        adhesive = deepcopy(frame)
        c = next(c for c in adhesive["contacts"] if "floor" in (c["geom1"], c["geom2"]))
        c["force_on_body2"] = [-v for v in c["force_on_body2"]]
        self.assertEqual(O.contact_admissibility(adhesive)["outcome"], "REJECTED")
        scaled = deepcopy(frame)
        for c in scaled["contacts"]:
            c["force_on_body2"] = [1.1 * v for v in c["force_on_body2"]]
        self.assertEqual(O.static_balance(scaled)["outcome"], "REJECTED")
        omitted = deepcopy(frame)
        omitted["contacts"] = [c for c in omitted["contacts"] if "deltoid" not in c["geom1"] + c["geom2"]]
        self.assertEqual(O.static_balance(omitted, members={"barbell"})["outcome"], "REJECTED")
        heel_only = deepcopy(frame)
        heel_only["contacts"] = [c for c in heel_only["contacts"] if "toe" not in c["geom1"] + c["geom2"]]
        self.assertEqual(O.static_balance(heel_only)["outcome"], "REJECTED")
        fake_grip = deepcopy(frame)
        fake_grip["grips"][0]["force_on_body1"] = [0.0, 0.0, 0.0]
        self.assertEqual(O.static_balance(fake_grip, members={"barbell"})["outcome"], "REJECTED")

    def test_closed_palm_front_rack_is_refused_not_forced(self):
        from biosyngen.spatial.plant import SpatialPlant
        plant = SpatialPlant.build(self.R.default_spec("FRONT_RACK", "CLOSED_PALM_V1"))
        _, closure, _ = self.R.solve_pose(plant, "FRONT_RACK", "POWER")
        self.assertGreater(closure["max_grip_separation_m"], 1e-3)

    def test_actuator_capacity_refusal_under_excess_load(self):
        from dataclasses import replace
        from biosyngen.spatial.plant import SpatialPlant
        plant = SpatialPlant.build(replace(self.R.default_spec("OVERHEAD"), plate_mass_kg=170.0))
        self.R.apply_pose(plant, self.solved["OVERHEAD"][1])
        certificate = self.R.static_equilibrium(plant, "OVERHEAD")
        self.assertIn(certificate.reason, ("ACTUATOR_CAPACITY_EXCEEDED", "UNBALANCED_FREE_BODY_RESIDUAL"))
        self.assertFalse(certificate.feasible)

    def test_perturbed_hold_is_stable_and_newton_euler_closes(self):
        from biosyngen_benchmark import spatial_oracle as O
        plant, x, _, certificate, _ = self.solved["OVERHEAD"]
        hold = self.R.hold_trial(plant, x, certificate, duration_s=.6, push_window=(.2, .3))
        self.assertEqual(hold["saturated_steps"], 0)
        self.assertLess(hold["final_max_speed"], .5)
        self.assertEqual(self.R.joint_range_violations(plant, hold["joint_angles"]), {})
        dt = plant.model.opt.timestep
        system = O.dynamic_balance(hold["frames"], dt)
        bar = O.dynamic_balance(hold["frames"], dt, members={"barbell"})
        self.assertLess(system["max_step_linear_residual_n_s"], 1e-4)
        self.assertLess(bar["max_step_linear_residual_n_s"], 1e-9)
        # Falsifier: dropping the recorded push leaves an unexplained impulse.
        tampered = deepcopy(hold["frames"])
        for f in tampered:
            f["external"] = []
        self.assertGreater(O.dynamic_balance(tampered, dt, members={"barbell"})["max_step_linear_residual_n_s"], .05)


if __name__ == "__main__":
    unittest.main()
