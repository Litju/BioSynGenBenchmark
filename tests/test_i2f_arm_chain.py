"""RES-494 opt-in lumped arm chain and gripped-bar topology on the bilateral human."""

from __future__ import annotations

import unittest

from biosyngen.i2.human import Anthropometry, LoadKind, LoadSpec, Posture, build_human
from biosyngen.i2.simulate import static_activations


class ArmChainTests(unittest.TestCase):
    def test_arm_chain_conserves_total_body_mass_and_moves_arm_mass_out_of_the_hat(self) -> None:
        plain = build_human()
        armed = build_human(arms=True)
        self.assertAlmostEqual(plain.total_mass_kg(), armed.total_mass_kg(), places=12)
        a = Anthropometry()
        arm_mass = 2.0 * (a.upper_arm_mass_fraction + a.forearm_hand_mass_fraction) * a.body_mass_kg
        self.assertAlmostEqual(plain.segments["hat"].mass_kg - armed.segments["hat"].mass_kg, arm_mass, places=12)
        self.assertNotIn("shoulder", plain.system._joint_by_id)
        self.assertIn("elbow", armed.system._joint_by_id)

    def test_gripped_bar_requires_arms(self) -> None:
        with self.assertRaisesRegex(ValueError, "arm chain"):
            build_human(load=LoadSpec(LoadKind.GRIP_BAR, 40.0))

    def test_held_bar_is_exactly_static_under_the_feedforward_with_full_load_share(self) -> None:
        model = build_human(load=LoadSpec(LoadKind.GRIP_BAR, 40.0, floor_supported=False), arms=True)
        posture = Posture(0.08, -0.12, 0.15, 0.2, 0.3)
        probe = model.posture_state(posture)
        state = model.posture_state(posture, activations=static_activations(model, probe, load_share=1.0))
        result = model.system.solve_accelerations(state, excitations=dict(state.activations))
        worst = max(abs(v) for acc in result.accelerations.values() for v in (*acc.linear_m_s2, acc.angular_rad_s2))
        self.assertLess(worst, 1e-9)
        feet = sum(w.normal_force_n for w in result.contact_wrenches.values())
        self.assertAlmostEqual(feet, model.total_mass_kg() * model.gravity_m_s2, delta=1e-6)
        # Without the load share the same posture is not static.
        unloaded = model.posture_state(posture, activations=static_activations(model, probe, load_share=0.0))
        result = model.system.solve_accelerations(unloaded, excitations=dict(unloaded.activations))
        worst = max(abs(v) for acc in result.accelerations.values() for v in (*acc.linear_m_s2, acc.angular_rad_s2))
        self.assertGreater(worst, 1e-3)


if __name__ == "__main__":
    unittest.main()
