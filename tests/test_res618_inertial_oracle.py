"""Analytical box decomposition and adversarial exports independent of generator."""
from copy import deepcopy
import unittest

try:
    import numpy as np
    from biosyngen_benchmark.human_v2_inertia import aggregate, compare_decomposition
except ImportError:
    np = None


def box(name, mass, center, lengths):
    x, y, z = lengths
    return dict(name=name, mass_kg=mass, com_world_m=center,
                inertia_world_kg_m2=np.diag([mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]).tolist(),
                frame="WORLD_X_ANTERIOR_Y_LEFT_Z_UP")


@unittest.skipIf(np is None, "NumPy required by mandatory spatial CI")
class InertialOracle(unittest.TestCase):
    def setUp(self):
        self.parent = box("parent", 12, [0, 0, 0], [2, 3, 4])
        self.children = [box("left", 6, [-.5, 0, 0], [1, 3, 4]), box("right", 6, [.5, 0, 0], [1, 3, 4])]

    def test_exact_uniform_box_partition(self):
        result = compare_decomposition(self.parent, self.children)
        self.assertEqual(result["outcome"], "PASS")
        self.assertLess(result["inertia_error_kg_m2"], 1e-13)

    def test_wrong_mass_com_tensor_and_frame_controls(self):
        for key, value in (("mass_kg", 6.1), ("com_world_m", [.6, 0, 0]),
                           ("inertia_world_kg_m2", np.eye(3).tolist())):
            children = deepcopy(self.children)
            children[1][key] = value
            self.assertEqual(compare_decomposition(self.parent, children)["outcome"], "FAIL")
        for key, value in (("frame", "LOCAL"), ("name", "left"), ("mass_kg", float("nan")),
                           ("inertia_world_kg_m2", np.diag([1, 1, 3]).tolist())):
            children = deepcopy(self.children)
            children[1][key] = value
            with self.assertRaises(ValueError):
                aggregate(children)

    def test_rotation_and_translation_covariance(self):
        original = aggregate(self.children)
        rotation = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
        translation = np.array([3, -2, 5])
        children = deepcopy(self.children)
        for child in children:
            child["com_world_m"] = (rotation @ child["com_world_m"] + translation).tolist()
            child["inertia_world_kg_m2"] = (rotation @ np.array(child["inertia_world_kg_m2"]) @ rotation.T).tolist()
        transformed = aggregate(children)
        np.testing.assert_allclose(transformed["com_world_m"], rotation @ original["com_world_m"] + translation, atol=1e-14)
        np.testing.assert_allclose(transformed["inertia_world_kg_m2"], rotation @ original["inertia_world_kg_m2"] @ rotation.T, atol=1e-14)

    def test_faulty_budget_refused(self):
        with self.assertRaises(ValueError):
            compare_decomposition(self.parent, self.children, mass_tolerance_kg=float("inf"))
