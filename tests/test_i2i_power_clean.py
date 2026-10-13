"""RES-497 power clean: verified pull phases and an explicit, computed catch refusal."""

from __future__ import annotations

import math
import unittest

from biosyngen.i2.power_clean import (
    MISSING_DOFS,
    NOT_IMPLEMENTED_EVENTS,
    PowerCleanRefused,
    analyze_power_clean,
    build_power_clean,
    catch_impulse_from_force_template,
    front_rack_feasibility,
)
from biosyngen.multibody import ContactKind


class FrontRackFeasibilityTests(unittest.TestCase):
    def test_required_elbow_flexion_matches_independent_forward_kinematics(self) -> None:
        for offset in (0.08, 0.12, 0.2, 0.3):
            r = front_rack_feasibility(rack_offset_m=offset)
            # Independent check: folding the two links by the reported elbow angle
            # puts the grip at exactly the requested distance from the shoulder.
            gx = r.upper_arm_m + r.forearm_grip_m * math.cos(r.required_elbow_flexion_rad)
            gz = r.forearm_grip_m * math.sin(r.required_elbow_flexion_rad)
            self.assertAlmostEqual(math.hypot(gx, gz), offset, places=12)

    def test_shoulder_rack_offsets_are_infeasible_and_only_hand_offsets_are_reachable(self) -> None:
        for offset in (0.08, 0.10, 0.12, 0.15):
            r = front_rack_feasibility(rack_offset_m=offset)
            self.assertFalse(r.feasible)
            self.assertGreater(r.required_elbow_flexion_rad, r.elbow_limit_rad)
        self.assertTrue(front_rack_feasibility(rack_offset_m=0.25).feasible)


class PowerCleanScopeTests(unittest.TestCase):
    def test_full_scope_is_refused_with_the_computed_reason_and_missing_dofs(self) -> None:
        with self.assertRaisesRegex(PowerCleanRefused, "CATCH_NOT_IMPLEMENTED") as caught:
            build_power_clean("FULL")
        self.assertIn("elbow flexion", str(caught.exception))
        for dof in MISSING_DOFS:
            self.assertIn(dof, str(caught.exception))
        with self.assertRaises(PowerCleanRefused):
            build_power_clean("TURNOVER_ONLY")

    def test_force_template_catch_is_forbidden(self) -> None:
        with self.assertRaisesRegex(PowerCleanRefused, "FORBIDDEN"):
            catch_impulse_from_force_template([0.0, 1.0], [0.0, 2000.0])

    def test_pull_only_scope_reports_pull_events_and_never_synthesizes_a_catch(self) -> None:
        scenario = build_power_clean()
        self.assertEqual(scenario.metadata["protocol_id"], "POWER_CLEAN_DUAL_V1")
        self.assertFalse(any(spec.kind is ContactKind.CATCH for spec in scenario.model.system.contacts))
        trajectory = scenario.run()
        result = analyze_power_clean(trajectory)
        self.assertEqual(result.pull.refusals, ())
        order = [result.events_s[k] for k in ("REP_START", "LIFT_OFF", "FIRST_PULL_END", "EXTENSION_END")]
        self.assertTrue(all(t is not None for t in order))
        self.assertEqual(order, sorted(order))
        self.assertEqual(set(result.not_implemented), set(NOT_IMPLEMENTED_EVENTS))
        self.assertFalse(set(result.events_s) & set(NOT_IMPLEMENTED_EVENTS))
        self.assertTrue(all(v.startswith("NOT_IMPLEMENTED") for v in result.not_implemented.values()))


if __name__ == "__main__":
    unittest.main()
