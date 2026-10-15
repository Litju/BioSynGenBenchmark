"""RES-507 two-link over-reach and nonfinite geometry falsifiers."""
import math
import unittest
from biosyngen.i2.power_clean import PowerCleanRefused, front_rack_feasibility

class ReachDomainTests(unittest.TestCase):
    def test_over_reach_is_not_clipped_into_a_feasible_straight_arm(self):
        result = front_rack_feasibility(rack_offset_m=2.0)
        self.assertFalse(result.feasible)
        self.assertIsNone(result.required_elbow_flexion_rad)

    def test_nonfinite_or_invalid_projection_geometry_is_refused(self):
        for arguments in ({"rack_offset_m": math.nan}, {"rack_offset_m": math.inf}, {"rack_offset_m": True},
                          {"grip_projection": 0}, {"grip_projection": -1}, {"grip_projection": 1.2}):
            with self.subTest(arguments=arguments), self.assertRaises(PowerCleanRefused):
                front_rack_feasibility(**arguments)
