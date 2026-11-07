"""RES-492 extensions to the RES-491 planar multibody engine.

Each change is checked against an independent formulation: the Schur
complement against the full dense saddle-point matrix.
"""

from __future__ import annotations

import math
import random
import unittest

from biosyngen.i1.linalg import solve
from biosyngen.multibody import (
    HILL_TORQUE_VELOCITY_MODEL_ID,
    ActuatorHypothesis,
    AppliedWrench,
    ConstraintSolveError,
    ContactKind,
    ContactMode,
    ContactSpec,
    ContactState,
    DynamicsError,
    ForcePlateFrame,
    PlanarJoint,
    PlanarMultibodySystem,
    RigidBodyState,
)
from tests.test_i2c_multibody import body, evidence, point_contact


def _full_kkt(system: PlanarMultibodySystem, force, rows, targets):
    """Independent dense KKT assembly (the pre-RES-492 formulation)."""
    n, m = system.dof_count, len(rows)
    matrix = [[0.0] * (n + m) for _ in range(n + m)]
    rhs = [0.0] * (n + m)
    for i, mass in enumerate(system._mass_diagonal()):
        matrix[i][i] = mass
        rhs[i] = force[i]
    for r, row in enumerate(rows):
        for c, value in enumerate(row.force_column):
            matrix[c][n + r] = -value
        for c, value in enumerate(row.jacobian):
            matrix[n + r][c] = value
        rhs[n + r] = targets[r]
    solution = solve(matrix, rhs)
    return solution[:n], solution[n:]


class SchurComplementKktTests(unittest.TestCase):
    def test_schur_solution_matches_dense_saddle_point_system(self) -> None:
        rng = random.Random(492)
        system = PlanarMultibodySystem(
            (body("torso", 5.0, 1.0), body("left_leg", 1.0, 0.1), body("right_leg", 1.2, 0.12)),
            (
                PlanarJoint("left_hip", "torso", "left_leg", (-0.3, 0.0), (0.0, 0.5)),
                PlanarJoint("right_hip", "torso", "right_leg", (0.3, 0.0), (0.0, 0.5)),
            ),
            (
                point_contact("left_foot", "left_leg", (0.0, -0.5), (-0.3, 0.0), friction=0.8,
                              side="LEFT", channel="FZ_LEFT"),
                point_contact("right_foot", "right_leg", (0.0, -0.5), (0.3, 0.0), friction=0.8,
                              side="RIGHT", channel="FZ_RIGHT"),
            ),
            gravity_m_s2=9.81, clock_id="schur-crosscheck",
        )
        for _ in range(20):
            state = system.make_state(
                {
                    "torso": RigidBodyState((0.0, 1.0), rng.uniform(-0.1, 0.1), (rng.uniform(-1, 1), 0.0), rng.uniform(-1, 1)),
                    "left_leg": RigidBodyState((-0.3, 0.5), 0.0, (0.0, 0.0), rng.uniform(-1, 1)),
                    "right_leg": RigidBodyState((0.3, 0.5), 0.0, (0.0, 0.0), rng.uniform(-1, 1)),
                },
                active_contacts={
                    "left_foot": ContactState(ContactMode.STICK),
                    "right_foot": ContactState(ContactMode.SLIDE, slip_direction=rng.choice((-1, 1))),
                },
            )
            rows = system._joint_rows(state) + system._contact_constraint_rows(state, state.contacts)[0]
            force = [rng.uniform(-50, 50) for _ in range(system.dof_count)]
            targets = [rng.uniform(-1, 1) for _ in rows]
            fast_x, fast_lam = system._solve_kkt(force, rows, targets)
            dense_x, dense_lam = _full_kkt(system, force, rows, targets)
            for a, b in zip(fast_x + fast_lam, dense_x + dense_lam):
                self.assertAlmostEqual(a, b, delta=1e-9 * (1.0 + abs(b)))


class FrictionPatchTests(unittest.TestCase):
    """A rigid flat foot: heel and toe share one tangential friction row."""

    MU, G, MASS = 0.6, 9.81, 3.0

    def _foot(self, *, grouped: bool = True, friction_toe: float | None = None,
              toe_surface_z: float = 0.0) -> PlanarMultibodySystem:
        frame = ForcePlateFrame("LEFT_PLATE", "LEFT", "FZ_LEFT", (0.0, 0.0), (1.0, 0.0), (0.0, 1.0))
        group = "left_sole" if grouped else None

        def contact(contact_id, kind, local, surface, mu):
            return ContactSpec(contact_id, "foot", local, (0.0, 1.0), mu, 0.0, kind, surface,
                               side="LEFT", measurement_channel_id="FZ_LEFT", plate_frame=frame,
                               friction_group=group)

        return PlanarMultibodySystem(
            (body("foot", self.MASS, 0.02),), (),
            (
                contact("heel", ContactKind.HEEL, (-0.1, -0.05), (-0.1, 0.0), self.MU),
                contact("toe", ContactKind.TOE, (0.15, -0.05), (0.15, toe_surface_z),
                        self.MU if friction_toe is None else friction_toe),
            ),
            gravity_m_s2=self.G, clock_id="friction-patch",
        )

    def _flat_state(self, system):
        return system.make_state({"foot": RigidBodyState((0.0, 0.05), 0.0)}, active_contacts=("heel", "toe"))

    def test_flat_foot_sticks_below_patch_capacity_with_determinate_tangential_reaction(self) -> None:
        dt = 0.001
        system = self._foot()
        state = self._flat_state(system)
        push = AppliedWrench("foot", "lateral", (0.9 * self.MU * self.MASS * self.G, 0.0))
        for _ in range(100):
            step = system.advance(state, dt, external_wrenches=(push,))
            state = step.state
            impulses = step.contact_impulses_n_s
            total_t = impulses["heel"][1] + impulses["toe"][1]
            self.assertAlmostEqual(total_t, -push.force_n[0] * dt, places=12)
            # Heel/toe normal impulses balance weight and the push moment about the COM.
            total_n = impulses["heel"][0] + impulses["toe"][0]
            self.assertAlmostEqual(total_n, self.MASS * self.G * dt, places=12)
            moment = (-0.1 * impulses["heel"][0] + 0.15 * impulses["toe"][0]) + 0.05 * total_t
            self.assertAlmostEqual(moment, 0.0, places=12)
            for name in ("heel", "toe"):
                self.assertLessEqual(abs(impulses[name][1]), self.MU * impulses[name][0] + 1e-15)
                self.assertIs(state.contacts[name].mode, ContactMode.STICK)
        self.assertEqual(state.bodies["foot"].position_m, (0.0, 0.05))
        wrench = system.solve_accelerations(state, external_wrenches=(push,)).contact_wrenches
        self.assertAlmostEqual(wrench["heel"].tangential_force_n + wrench["toe"].tangential_force_n,
                               -push.force_n[0], places=9)

    def test_flat_foot_slides_above_summed_normal_capacity(self) -> None:
        dt = 0.001
        system = self._foot()
        state = self._flat_state(system)
        capacity = self.MU * self.MASS * self.G
        push = AppliedWrench("foot", "lateral", (1.2 * capacity, 0.0))
        for _ in range(50):
            step = system.advance(state, dt, external_wrenches=(push,))
            state = step.state
            impulses = step.contact_impulses_n_s
            self.assertAlmostEqual(impulses["heel"][1] + impulses["toe"][1], -capacity * dt, places=12)
            self.assertTrue(all(state.contacts[c].mode is ContactMode.SLIDE for c in ("heel", "toe")))
            self.assertTrue(all(state.contacts[c].slip_direction == 1 for c in ("heel", "toe")))
        self.assertAlmostEqual(state.bodies["foot"].velocity_m_s[0], 0.2 * capacity / self.MASS * state.time_s, places=10)

    def test_heel_releases_and_toe_alone_carries_the_patch_when_moment_exceeds_support(self) -> None:
        system = self._foot()
        state = self._flat_state(system)
        # Moment large enough that the heel would need adhesion.
        lift_heel = AppliedWrench("foot", "plantarflexing-moment", moment_n_m=-0.2 * self.MASS * self.G)
        step = system.advance(state, 0.001, external_wrenches=(lift_heel,))
        self.assertIs(step.state.contacts["heel"].mode, ContactMode.OPEN)
        self.assertIsNot(step.state.contacts["toe"].mode, ContactMode.OPEN)
        self.assertIn("heel", next(e for e in step.events if e.event_id == "CONTACT_RELEASE").contact_ids)

    def test_ungrouped_flat_foot_and_noncoplanar_patch_fail_closed(self) -> None:
        with self.assertRaisesRegex(ConstraintSolveError, "singular or redundant"):
            system = self._foot(grouped=False)
            system.advance(self._flat_state(system), 0.001)
        tilt = math.asin(0.05 / 0.25)
        system = self._foot(toe_surface_z=0.05)
        # Rotate the foot so both points touch surfaces at different heights.
        c, s = math.cos(tilt), math.sin(tilt)
        heel_world = (-0.1, 0.0)
        com = (heel_world[0] - (c * -0.1 - s * -0.05), heel_world[1] - (s * -0.1 + c * -0.05))
        state = system.make_state({"foot": RigidBodyState(com, tilt)}, active_contacts=("heel", "toe"))
        with self.assertRaisesRegex(ConstraintSolveError, "not coplanar"):
            system.advance(state, 0.001)

    def test_malformed_friction_groups_are_refused(self) -> None:
        with self.assertRaisesRegex(DynamicsError, "must share"):
            self._foot(friction_toe=0.3)
        with self.assertRaisesRegex(DynamicsError, "at least two"):
            PlanarMultibodySystem(
                (body("foot", 1.0, 0.01),), (),
                (point_contact("only", "foot", (0.0, 0.0), (0.0, 0.0), friction=0.5, kind=ContactKind.RACK)
                 .__class__(**{**_spec_fields(point_contact("only", "foot", (0.0, 0.0), (0.0, 0.0),
                                                            friction=0.5, kind=ContactKind.RACK)),
                               "friction_group": "lonely"}),),
                gravity_m_s2=9.81, clock_id="lonely-group",
            )


class HillTorqueVelocityTests(unittest.TestCase):
    @staticmethod
    def _actuator(**kwargs) -> ActuatorHypothesis:
        defaults = dict(
            actuator_id="motor", joint_id="hinge", capacity_n_m=100.0,
            activation_time_s=0.01, deactivation_time_s=0.04,
            provenance=evidence("motor-capacity", 0.1),
            model_id=HILL_TORQUE_VELOCITY_MODEL_ID, max_joint_velocity_rad_s=10.0,
        )
        defaults.update(kwargs)
        return ActuatorHypothesis(**defaults)

    def test_velocity_factor_matches_closed_form_branches(self) -> None:
        actuator = self._actuator()
        k, e = 0.25, 1.5
        self.assertEqual(actuator.velocity_factor(1.0, 0.0), 1.0)
        self.assertAlmostEqual(actuator.velocity_factor(1.0, 5.0), 0.5 / (1.0 + 0.5 / k), places=15)
        self.assertAlmostEqual(actuator.velocity_factor(-1.0, -5.0), 0.5 / (1.0 + 0.5 / k), places=15)
        self.assertEqual(actuator.velocity_factor(1.0, 10.0), 0.0)
        self.assertEqual(actuator.velocity_factor(1.0, 25.0), 0.0)
        c = (e - 1.0) / (2.0 * (1.0 + 1.0 / k))
        self.assertAlmostEqual(actuator.velocity_factor(1.0, -3.0), 1.0 + (e - 1.0) * 0.3 / (0.3 + c), places=15)
        self.assertLess(actuator.velocity_factor(1.0, -1e6), e)
        self.assertGreater(actuator.velocity_factor(1.0, -1e6), e - 1e-6)
        h = 1e-7
        self.assertAlmostEqual(actuator.velocity_factor(1.0, h), actuator.velocity_factor(1.0, -h), places=5)
        # Monotone decreasing in shortening velocity across the whole range.
        values = [actuator.velocity_factor(1.0, w) for w in [-50 + 0.5 * i for i in range(130)]]
        self.assertTrue(all(a >= b for a, b in zip(values, values[1:])))

    def test_model_identity_must_match_declared_parameters(self) -> None:
        with self.assertRaisesRegex(DynamicsError, "constitutive model"):
            self._actuator(model_id="ACTIVATED_SYMMETRIC_TORQUE_CAPACITY_V1")
        with self.assertRaisesRegex(DynamicsError, "constitutive model"):
            self._actuator(max_joint_velocity_rad_s=None)
        with self.assertRaisesRegex(DynamicsError, "eccentric_ratio"):
            self._actuator(eccentric_ratio=0.8)

    def test_free_hinge_approaches_but_never_exceeds_declared_max_velocity(self) -> None:
        actuator = self._actuator(capacity_n_m=5.0)
        system = PlanarMultibodySystem(
            (body("base", 1000.0, 1000.0), body("link", 1.0, 0.05)),
            (PlanarJoint("hinge", "base", "link", (0.0, 0.0), (0.0, 0.0)),),
            (), gravity_m_s2=9.81, clock_id="hill-hinge", actuators=(actuator,),
        )
        state = system.make_state(
            {"base": RigidBodyState((0.0, 0.0), 0.0), "link": RigidBodyState((0.0, 0.0), 0.0)},
            activations={"motor": 1.0},
        )
        result = system.solve_accelerations(state, excitations={"motor": 1.0})
        self.assertAlmostEqual(result.actuator_torques_n_m["motor"], 5.0, places=12)
        peak = 0.0
        for _ in range(2000):
            state = system.advance(state, 0.001, excitations={"motor": 1.0}).state
            relative = state.bodies["link"].angular_velocity_rad_s - state.bodies["base"].angular_velocity_rad_s
            peak = max(peak, relative)
        self.assertLess(peak, 10.0)
        self.assertGreater(peak, 9.0)
        torque = system.solve_accelerations(state, excitations={"motor": 1.0}).actuator_torques_n_m["motor"]
        self.assertAlmostEqual(torque, 5.0 * actuator.velocity_factor(1.0, relative), places=12)


def _spec_fields(spec: ContactSpec) -> dict:
    return {name: getattr(spec, name) for name in ContactSpec.__dataclass_fields__}



class ProjectionImpulseLedgerTests(unittest.TestCase):
    def test_projected_step_momentum_closes_with_contact_and_projection_impulses(self) -> None:
        from tests.test_i2c_multibody import VelocityLevelCoulombFalsifierTests as fixtures

        dt = 0.001
        system = fixtures._two_leg_rack()
        state = fixtures._two_leg_state(fixtures(), system)
        away = AppliedWrench("torso", "pull-from-rack", (10.0, 0.0))
        masses = {"torso": 5.0, "left_leg": 1.0, "right_leg": 1.0}
        worst = 0.0
        exercised = False
        for _ in range(150):
            step = system.advance(state, dt, external_wrenches=(away,))
            exercised = exercised or any(abs(v) > 0.0 for p in step.projection_impulses_n_s.values() for v in p)
            for axis, external in ((0, 10.0 * dt), (1, -sum(masses.values()) * 9.81 * dt)):
                change = sum(
                    m * (step.state.bodies[b].velocity_m_s[axis] - state.bodies[b].velocity_m_s[axis])
                    for b, m in masses.items()
                )
                contact = 0.0
                for ledger, frame_state in ((step.contact_impulses_n_s, state),
                                            (step.projection_impulses_n_s, step.state)):
                    for contact_id, (normal, tangential) in ledger.items():
                        kinematics = system.contact_kinematics(frame_state, contact_id)
                        contact += normal * kinematics.normal_world[axis] + tangential * kinematics.tangent_world[axis]
                worst = max(worst, abs(change - external - contact))
            state = step.state
        self.assertLess(worst, 1e-11)
        self.assertTrue(exercised)



class UnilateralContactLawFalsifierTests(unittest.TestCase):
    """Falsifiers for the velocity-level Signorini-Coulomb law on a rigid flat foot.

    They target four defects found while building the RES-492 jumper: adhesive
    post-step projections during fast toe roll, penetration after release,
    grazing crossings missed by impact detection, and a greedy active set that
    cycled at heel strike instead of releasing the toe.
    """

    def _drop(self, rng: random.Random, spin_rad_s: float = 4.0):
        system = FrictionPatchTests()._foot()
        angle = rng.uniform(-0.6, 0.6)
        state = system.make_state({"foot": RigidBodyState(
            (rng.uniform(-0.05, 0.05), 0.3 + rng.uniform(0.0, 0.2)), angle,
            # Bounded spin keeps landings sole-down: only the heel/toe points have
            # contact geometry, so a tumbled foot could rest on inverted points.
            (rng.uniform(-1.5, 1.5), rng.uniform(-3.0, 0.5)), rng.uniform(-spin_rad_s, spin_rad_s),
        )})
        return system, state

    def test_random_rigid_foot_drops_respect_signorini_coulomb_and_energy(self) -> None:
        rng = random.Random(4920)
        mu = FrictionPatchTests.MU
        for trial in range(25):
            system, state = self._drop(rng)
            initial_energy = system.kinetic_energy(state) + system.potential_energy(state)
            zero_steps = 0
            for _ in range(800):
                step = system.advance(state, 0.001)
                zero_steps = zero_steps + 1 if step.elapsed_s == 0.0 else 0
                self.assertLess(zero_steps, 5, f"trial {trial}: zero-time event loop")
                for contact_id, (normal, tangential) in step.contact_impulses_n_s.items():
                    self.assertGreaterEqual(normal, 0.0)
                # RES-517: projection is signed unloading/reallocation of the
                # already-applied impulse, not a second physical impulse.
                for contact_id in ("heel", "toe"):
                    base = step.contact_impulses_n_s.get(contact_id, (0.0, 0.0))
                    correction = step.projection_impulses_n_s.get(contact_id, (0.0, 0.0))
                    self.assertGreaterEqual(base[0] + correction[0], -1e-8)
                combined = [sum(ledger.get(c, (0.0, 0.0))[axis]
                                for ledger in (step.contact_impulses_n_s, step.projection_impulses_n_s)
                                for c in ("heel", "toe")) for axis in (0, 1)]
                self.assertLessEqual(abs(combined[1]), mu * combined[0] + 1e-8)
                for event in step.events:
                    if event.event_id == "CONTACT_IMPACT":
                        self.assertLessEqual(event.contact_work_j, 1e-9)
                        self.assertTrue(all(p >= 0.0 for p in event.normal_impulses_n_s.values()))
                        for c in event.contact_ids:
                            self.assertLessEqual(abs(event.tangential_impulses_n_s[c]),
                                                 mu * sum(event.normal_impulses_n_s.values()) + 1e-12)
                state = step.state
                for contact_id in ("heel", "toe"):
                    self.assertGreater(system.contact_kinematics(state, contact_id).gap_m, -1e-8,
                                       f"trial {trial}: {contact_id} penetrates")
                # No net creation: mechanical energy never exceeds its initial value.
                energy = system.kinetic_energy(state) + system.potential_energy(state)
                self.assertLessEqual(energy, initial_energy + 1e-9, f"trial {trial}: energy created")
            # A rigid foot dropped on a flat floor must come to rest on it.
            self.assertTrue(all(state.contacts[c].mode is ContactMode.STICK for c in ("heel", "toe")),
                            f"trial {trial}: {state.contacts}")

    def test_per_step_energy_fluctuation_converges_at_second_order(self) -> None:
        """The first-order scheme may raise energy within one step by O(dt^2)
        (impulse acting at the pre-step normal geometry); it must vanish under
        refinement, and the cumulative energy must never exceed its start."""
        worst = {}
        for dt in (0.001, 0.0005, 0.00025):
            rng = random.Random(4920)
            worst[dt] = 0.0
            for _ in range(10):
                system, state = self._drop(rng, spin_rad_s=15.0)
                energy = system.kinetic_energy(state) + system.potential_energy(state)
                for _ in range(int(round(0.4 / dt))):
                    state = system.advance(state, dt).state
                    new_energy = system.kinetic_energy(state) + system.potential_energy(state)
                    worst[dt] = max(worst[dt], new_energy - energy)
                    energy = new_energy
        self.assertGreater(worst[0.001], 0.0)
        self.assertGreater(worst[0.001] / max(worst[0.0005], 1e-300), 3.0)
        self.assertGreater(worst[0.0005] / max(worst[0.00025], 1e-300), 3.0)

    def test_fast_toe_roll_releases_instead_of_pulling_the_toe_down(self) -> None:
        system = FrictionPatchTests()._foot()
        state = system.make_state({"foot": RigidBodyState((0.0, 0.05), 0.0)}, active_contacts=("heel", "toe"))
        # A strong plantarflexing moment lifts the heel, then spins the foot about the toe.
        spin = AppliedWrench("foot", "roll", moment_n_m=-40.0)
        released = False
        for _ in range(200):
            step = system.advance(state, 0.001, external_wrenches=(spin,))
            for contact_id in ("heel", "toe"):
                normal = (step.contact_impulses_n_s.get(contact_id, (0.0, 0.0))[0]
                          + step.projection_impulses_n_s.get(contact_id, (0.0, 0.0))[0])
                self.assertGreaterEqual(normal, -1e-8)
            state = step.state
            released = released or state.contacts["toe"].mode is ContactMode.OPEN
            self.assertGreater(system.contact_kinematics(state, "toe").gap_m, -1e-8)
            self.assertGreater(system.contact_kinematics(state, "heel").gap_m, -1e-8)
        self.assertTrue(released)

    def test_crossing_with_initially_separating_velocity_is_not_missed(self) -> None:
        system = FrictionPatchTests()._foot()
        # Moving up slowly 1e-5 m above the floor: gravity reverses it within the
        # step, so a start-velocity criterion would miss the crossing.
        state = system.make_state({"foot": RigidBodyState((0.0, 0.05 + 1e-5), 0.0, (0.0, 0.004), 0.0)})
        for _ in range(20):
            state = system.advance(state, 0.002).state
            for contact_id in ("heel", "toe"):
                self.assertGreater(system.contact_kinematics(state, contact_id).gap_m, -1e-8)
        self.assertTrue(all(state.contacts[c].mode is not ContactMode.OPEN for c in ("heel", "toe")))



class FiniteSurfaceExtentTests(unittest.TestCase):
    """RES-495: edge-bounded world surfaces (e.g. bench-press J-hooks)."""

    def _block_on_ledge(self, extent=(-0.2, 0.2), vx=0.5, mu=0.0):
        system = PlanarMultibodySystem(
            (body("bar", 20.0, 0.05),), (),
            (ContactSpec("hook", "bar", (0.0, 0.0), (0.0, 1.0), mu, 0.0, ContactKind.RACK,
                         surface_point_m=(0.0, 1.0), surface_extent_m=extent),),
            gravity_m_s2=9.81, clock_id="ledge",
        )
        state = system.make_state({"bar": RigidBodyState((0.0, 1.0), 0.0, (vx, 0.0))}, active_contacts=("hook",))
        return system, state

    def test_bar_slides_off_the_edge_releases_and_falls_ballistically(self) -> None:
        system, state = self._block_on_ledge()
        released_at = None
        for _ in range(800):
            step = system.advance(state, 0.001)
            state = step.state
            if released_at is None and state.contacts["hook"].mode is ContactMode.OPEN:
                released_at = state.time_s
        self.assertIsNotNone(released_at)
        self.assertAlmostEqual(released_at, 0.2 / 0.5, delta=0.003)
        # After leaving the ledge the bar falls freely below the (absent) surface height.
        self.assertLess(state.bodies["bar"].position_m[1], 0.5)
        self.assertAlmostEqual(state.bodies["bar"].velocity_m_s[0], 0.5, places=9)

    def test_crossing_outside_the_extent_is_not_an_impact(self) -> None:
        system = PlanarMultibodySystem(
            (body("bar", 20.0, 0.05),), (),
            (ContactSpec("hook", "bar", (0.0, 0.0), (0.0, 1.0), 0.4, 0.0, ContactKind.RACK,
                         surface_point_m=(0.0, 1.0), surface_extent_m=(-0.1, 0.1)),),
            gravity_m_s2=9.81, clock_id="ledge",
        )
        outside = system.make_state({"bar": RigidBodyState((0.5, 1.2), 0.0)})
        inside = system.make_state({"bar": RigidBodyState((0.0, 1.2), 0.0)})
        for state, expect_impact in ((outside, False), (inside, True)):
            impacts = 0
            for _ in range(400):
                step = system.advance(state, 0.001)
                impacts += sum(e.event_id == "CONTACT_IMPACT" for e in step.events)
                state = step.state
            self.assertEqual(impacts > 0, expect_impact)
            if expect_impact:
                self.assertAlmostEqual(state.bodies["bar"].position_m[1], 1.0, delta=1e-8)
            else:
                self.assertLess(state.bodies["bar"].position_m[1], 1.0)

    def test_entering_the_extent_from_the_side_below_the_top_is_refused(self) -> None:
        system = PlanarMultibodySystem(
            (body("bar", 20.0, 0.05),), (),
            (ContactSpec("hook", "bar", (0.0, 0.0), (0.0, 1.0), 0.4, 0.0, ContactKind.RACK,
                         surface_point_m=(0.0, 1.0), surface_extent_m=(-0.1, 0.1)),),
            gravity_m_s2=9.81, clock_id="ledge",
        )
        # Below the ledge top, moving sideways into the hook.
        state = system.make_state({"bar": RigidBodyState((-0.3, 0.98), 0.0, (2.0, 0.0))})
        with self.assertRaisesRegex(ConstraintSolveError, "EDGE_ENTRY_NOT_MODELED"):
            for _ in range(200):
                state = system.advance(state, 0.001).state

    def test_extent_is_validated(self) -> None:
        with self.assertRaisesRegex(DynamicsError, "nonempty interval"):
            ContactSpec("hook", "bar", (0.0, 0.0), (0.0, 1.0), 0.4, 0.0, ContactKind.RACK,
                        surface_point_m=(0.0, 1.0), surface_extent_m=(0.1, -0.1))


if __name__ == "__main__":
    unittest.main()
