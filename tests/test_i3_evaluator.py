"""RES-504 externally authored analytical submissions; no generator imports.

Mass, velocities and applied forces are computed from Newton's equation here,
not from a generator export or a stored successful simulation.
"""
import unittest
from biosyngen_benchmark.i2_evaluator import evaluate_i2

G = 9.80665
CLAIMS = ["LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY", "BILATERAL_FORCE", "ANATOMICAL_PHASES"]


def reference(groups=("FOOT_LEFT", "FOOT_RIGHT"), fractions=None, fx=0.0):
    mass = 2.0
    fractions = fractions or [1 / len(groups)] * len(groups)
    frames = []
    for i in range(2):
        t0, t1 = i * 0.5, (i + 1) * 0.5
        contacts = [{"name": group, "group": group, "x": 0.0, "z": 0.0,
                     "fx": fx * share, "fz": mass * G * share}
                    for group, share in zip(groups, fractions)]
        frames.append({"t0": t0, "t1": t1, "body_states": [[fx / mass * t1**2 / 2, 0, 0, fx / mass * t1, 0, 0]],
                       "contacts": contacts})
    return {"schema": "BIOSYNGEN_I2_SUBMISSION_V1", "exercise_id": "INDEPENDENT_ANALYTICAL_FIXTURE",
            "gravity_m_s2": G, "bodies": [{"name": "block", "mass_kg": mass, "inertia_kg_m2": 0.2}],
            "initial_body_states": [[0, 0, 0, 0, 0, 0]], "frames": frames, "impulses": [],
            "external_support_groups": list(groups), "friction_by_group": dict.fromkeys(groups, 0.5),
            "claims": CLAIMS.copy()}


class AnalyticalReferenceTests(unittest.TestCase):
    def test_ten_independent_support_and_free_fall_cases(self):
        cases = [(("FOOT_LEFT",), [1.0]), (("FOOT_RIGHT",), [1.0]),
                 (("FOOT_LEFT", "FOOT_RIGHT"), [0.5, 0.5]),
                 (("FOOT_LEFT", "FOOT_RIGHT"), [0.3, 0.7]),
                 (("FOOT_LEFT", "BAR_FLOOR"), [0.4, 0.6]),
                 (("BENCH", "FOOT_RIGHT"), [0.8, 0.2]),
                 (("BENCH", "RACK", "FOOT_LEFT"), [0.4, 0.4, 0.2]),
                 (("RACK",), [1.0]), (("BAR_FLOOR",), [1.0])]
        for groups, fractions in cases:
            with self.subTest(groups=groups, fractions=fractions):
                result = evaluate_i2(reference(groups, fractions))
                self.assertEqual(result["status"], "EVALUATED")
                self.assertEqual(result["findings"]["LINEAR_MOMENTUM_CLOSURE"]["outcome"], "PASS")
                self.assertEqual(result["findings"]["CONTACT_ADMISSIBILITY"]["outcome"], "PASS")
                self.assertEqual(result["findings"]["BILATERAL_FORCE"]["outcome"], "NOT_EVALUABLE")
        falling = reference()
        falling["external_support_groups"] = []
        falling["friction_by_group"] = {}
        for frame in falling["frames"]:
            t = frame["t1"]
            frame["contacts"] = []
            frame["body_states"][0] = [0, -G * t**2 / 2, 0, 0, -G * t, 0]
        self.assertEqual(evaluate_i2(falling)["findings"]["LINEAR_MOMENTUM_CLOSURE"]["outcome"], "PASS")

    def test_local_corruption_cannot_cancel_at_record_end(self):
        sub = reference()
        sub["frames"][0]["body_states"][0][3] = 100.0
        self.assertEqual(evaluate_i2(sub)["findings"]["LINEAR_MOMENTUM_CLOSURE"]["outcome"], "REJECTED")

    def test_equal_opposite_force_faults_cannot_cancel(self):
        sub = reference()
        for i, frame in enumerate(sub["frames"]):
            frame["contacts"][0]["fz"] += 5.0 if i == 0 else -5.0
        self.assertEqual(evaluate_i2(sub)["findings"]["LINEAR_MOMENTUM_CLOSURE"]["outcome"], "REJECTED")

    def test_tensile_impulse_cannot_pass_contact_admissibility(self):
        sub = reference()
        sub["impulses"] = [{"t": 0.5, "name": "FOOT_LEFT", "group": "FOOT_LEFT", "x": 0, "z": 0,
                            "jx": 0, "jz": -1.0}]
        for frame in sub["frames"]:
            frame["body_states"][0][4] = -0.5
        self.assertEqual(evaluate_i2(sub)["findings"]["CONTACT_ADMISSIBILITY"]["outcome"], "REJECTED")

    def test_impulse_outside_record_is_invalid_even_if_zero(self):
        sub = reference()
        sub["impulses"] = [{"t": 100.0, "name": "FOOT_LEFT", "group": "FOOT_LEFT", "x": 0, "z": 0,
                            "jx": 0, "jz": 0}]
        self.assertEqual(evaluate_i2(sub)["status"], "INVALID")

    def test_impulse_friction_cone_is_checked(self):
        sub = reference()
        sub["impulses"] = [{"t": 0.5, "name": "FOOT_LEFT", "group": "FOOT_LEFT", "x": 0, "z": 0,
                            "jx": 10, "jz": 1}]
        for frame in sub["frames"]:
            frame["body_states"][0][3:5] = [5, 0.5]
        self.assertEqual(evaluate_i2(sub)["findings"]["CONTACT_ADMISSIBILITY"]["outcome"], "REJECTED")

    def test_instantaneous_frame_at_an_interval_boundary_is_not_double_counted(self):
        sub = reference()
        sub["impulses"] = [{"t": 0.5, "name": "FOOT_LEFT", "group": "FOOT_LEFT", "x": 0, "z": 0,
                            "jx": 0, "jz": 1.0}]
        instant = {"t0": 0.5, "t1": 0.5, "body_states": [[0, 0, 0, 0, 0.5, 0]], "contacts": []}
        sub["frames"].insert(1, instant)
        sub["frames"][-1]["body_states"][0][4] = 0.5
        self.assertEqual(evaluate_i2(sub)["findings"]["LINEAR_MOMENTUM_CLOSURE"]["outcome"], "PASS")


class HostileSchemaTests(unittest.TestCase):
    def test_empty_record_does_not_pass_contact_admissibility(self):
        sub = reference()
        sub["frames"] = []
        result = evaluate_i2(sub)
        self.assertEqual(result["status"], "EVALUATED")
        for claim in ("LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY"):
            self.assertEqual(result["findings"][claim]["outcome"], "NOT_EVALUABLE")
            self.assertEqual(result["findings"][claim]["code"],
                             "NO_POSITIVE_DURATION_OBSERVATION")

    def test_instantaneous_only_record_is_not_a_positive_physics_case(self):
        sub = reference()
        sub["frames"] = [{
            "t0": 0.0, "t1": 0.0,
            "body_states": [list(state) for state in sub["initial_body_states"]],
            "contacts": [],
        }]
        sub["external_support_groups"] = []
        sub["friction_by_group"] = {}
        result = evaluate_i2(sub)
        self.assertEqual(result["status"], "EVALUATED")
        for claim in ("LINEAR_MOMENTUM_CLOSURE", "CONTACT_ADMISSIBILITY"):
            self.assertEqual(result["findings"][claim]["outcome"], "NOT_EVALUABLE")
        self.assertEqual(result["findings"]["LINEAR_MOMENTUM_CLOSURE"]["code"],
                         "NO_POSITIVE_DURATION_OBSERVATION")

    def test_arbitrary_nested_json_types_return_invalid_without_throwing(self):
        mutations = [(key, value) for key in ("bodies", "initial_body_states", "frames", "impulses",
                      "external_support_groups", "friction_by_group", "claims") for value in (None, 1, True, "x")]
        for key, value in mutations:
            with self.subTest(key=key, value=value):
                sub = reference()
                sub[key] = value
                self.assertEqual(evaluate_i2(sub)["status"], "INVALID")
        for path in ("body", "state", "frame", "contact", "impulse"):
            with self.subTest(path=path):
                sub = reference()
                if path == "body": sub["bodies"] = [None]
                if path == "state": sub["initial_body_states"] = [None]
                if path == "frame": sub["frames"] = [None]
                if path == "contact": sub["frames"][0]["contacts"] = [None]
                if path == "impulse": sub["impulses"] = [None]
                self.assertEqual(evaluate_i2(sub)["status"], "INVALID")

    def test_massless_empty_world_is_invalid(self):
        sub = reference()
        sub["bodies"] = []
        sub["initial_body_states"] = []
        for frame in sub["frames"]:
            frame["body_states"] = []
            frame["contacts"] = []
        self.assertEqual(evaluate_i2(sub)["status"], "INVALID")

    def test_inertia_must_be_positive(self):
        for value in (0, -1, float("nan"), True):
            sub = reference()
            sub["bodies"][0]["inertia_kg_m2"] = value
            with self.subTest(value=value):
                self.assertEqual(evaluate_i2(sub)["status"], "INVALID")

    def test_duplicate_body_identities_are_invalid(self):
        sub = reference()
        sub["bodies"] *= 2
        sub["initial_body_states"] *= 2
        for frame in sub["frames"]: frame["body_states"] *= 2
        self.assertEqual(evaluate_i2(sub)["status"], "INVALID")

    def test_missing_or_nonfinite_friction_cannot_bypass_a_cone(self):
        for value in ({}, {"FOOT_LEFT": float("nan"), "FOOT_RIGHT": 0.5},
                      {"FOOT_LEFT": -1, "FOOT_RIGHT": 0.5}, {"FOOT_LEFT": True, "FOOT_RIGHT": 0.5}):
            with self.subTest(value=value):
                sub = reference(fx=100.0)
                sub["friction_by_group"] = value
                self.assertEqual(evaluate_i2(sub)["status"], "INVALID")

    def test_unknown_support_or_claim_is_invalid(self):
        sub = reference()
        sub["external_support_groups"].append("SECRET_SUPPORT")
        self.assertEqual(evaluate_i2(sub)["status"], "INVALID")
        sub = reference()
        sub["claims"].append("BIOLOGICAL_VALIDITY")
        self.assertEqual(evaluate_i2(sub)["status"], "INVALID")

    def test_candidate_tolerance_and_source_stage_spoofing_are_invalid(self):
        for key, value in (("tolerance", 1e30), ("source_stage", "REAL_PAIRED_FORCE_LPT"),
                           ("calibration", {"approved": True}), ("clock_sync", 0)):
            sub = reference()
            sub[key] = value
            with self.subTest(key=key):
                self.assertEqual(evaluate_i2(sub)["status"], "INVALID")

    def test_nonfinite_states_and_multiple_corruptions_are_invalid(self):
        sub = reference()
        sub["initial_body_states"][0][0] = float("inf")
        sub["frames"][0]["contacts"][0]["fz"] = float("nan")
        sub["frames"][1]["t0"] = -1
        self.assertEqual(evaluate_i2(sub)["status"], "INVALID")

    def test_impulses_require_observed_frames(self):
        sub = reference()
        sub["frames"] = []
        sub["impulses"] = [{"t": 0, "name": "FOOT_LEFT", "group": "FOOT_LEFT", "x": 0, "z": 0, "jx": 0, "jz": 0}]
        self.assertEqual(evaluate_i2(sub)["status"], "INVALID")


if __name__ == "__main__":
    unittest.main()
