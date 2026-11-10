"""RES-624 independent receiving acceptance: positive verdict and adversarial refusals."""
import unittest

try:
    import mujoco
    import numpy as np  # noqa: F401
    import scipy  # noqa: F401
except ImportError:
    mujoco = None


class EvaluatorIsIndependent(unittest.TestCase):
    def test_evaluator_imports_no_generator_or_engine(self):
        import ast
        from pathlib import Path
        for path in ("biosyngen_benchmark/receiving_acceptance.py", "biosyngen_benchmark/rigid_body_check.py"):
            tree = ast.parse(Path(path).read_text(encoding="utf-8"))
            names = {n.module or "" for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
                {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
            self.assertFalse({n for n in names if n.startswith(("biosyngen.", "mujoco")) or n == "biosyngen"}, path)

    def test_frozen_res618_script_is_unchanged_hardcoded_fail(self):
        from pathlib import Path
        src = Path("scripts/res618_fullbody_qualification.py").read_text(encoding="utf-8")
        self.assertIn('outcome="FAIL", code="RECEIVING_NOT_QUALIFIED_AFTER_TWO_REPAIRS"', src)
        self.assertIn("return 1", src)


@unittest.skipIf(mujoco is None, "MuJoCo/SciPy required by mandatory spatial CI")
class Acceptance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from scripts.res624_receiving_campaign import build_nominal, run_case
        _, cls.nominal = build_nominal()
        cls.frames, cls.static, plant = run_case("nominal_0p3s_red_window", None, None, .3, {}, cls.nominal)
        cls.dt = plant.model.opt.timestep

    def evaluate(self, frames, verdict=None):
        from biosyngen_benchmark.receiving_acceptance import evaluate_receiving
        return evaluate_receiving(frames, self.dt, verdict or self.static["verdict"])

    def test_nominal_red_window_is_qualified_from_evidence(self):
        r = self.evaluate(self.frames)
        self.assertEqual(r["outcome"], "RECEIVING_HOLD_QUALIFIED", r.get("reason"))
        self.assertLess(r["evidence"]["bar_newton_euler"]["max_step_linear_residual_n_s"], 1e-10)

    def test_frozen_red_is_refused(self):
        from scripts.res624_receiving_qualification import red_frames
        frames, verdict, drop = red_frames()
        self.assertEqual(verdict, "STATIC_GRASP_INFEASIBLE")
        self.assertGreater(drop, .3)
        self.assertNotEqual(self.evaluate(frames, verdict)["outcome"], "RECEIVING_HOLD_QUALIFIED")

    def test_adversarial_manipulations_are_refused(self):
        from scripts.res624_receiving_qualification import adversarial
        out = adversarial(self.frames, self.dt, self.static["verdict"], self.nominal)
        for name, r in out.items():
            self.assertTrue(r["refused"], (name, r))
        self.assertEqual(set(out), {"hand_contacts_deleted", "declared_friction_violated", "actuator_capacity_exceeded",
                                    "non_foot_floor_contact", "static_not_certified", "undeclared_bar_support"})

    def test_truncated_or_nonfinite_frames_are_not_qualified(self):
        import copy
        self.assertEqual(self.evaluate(self.frames[:1])["outcome"], "NUMERICAL_NOT_QUALIFIED")
        bad = copy.deepcopy(self.frames)
        bad[5]["bodies"][0]["v"][0] = float("nan")
        self.assertEqual(self.evaluate(bad)["outcome"], "NUMERICAL_NOT_QUALIFIED")


if __name__ == "__main__":
    unittest.main()
