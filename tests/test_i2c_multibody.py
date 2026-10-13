from __future__ import annotations

import math
import random
import unittest

from biosyngen.multibody import (
    ActuatorHypothesis,
    AppliedWrench,
    ConnectionKind,
    ContactKind,
    ContactMode,
    ContactNormalFrame,
    ContactSpec,
    ContactState,
    ConstraintSolveError,
    DynamicsError,
    ForcePlateFrame,
    JointKind,
    ParameterDomainError,
    ParameterProvenance,
    ParameterStatus,
    PlanarJoint,
    PlanarMultibodySystem,
    ReferenceAngleController,
    RigidBodyDefinition,
    RigidBodyState,
    SupportContract,
    SupportIncompleteError,
)


def evidence(parameter_id: str, uncertainty: float = 0.02) -> ParameterProvenance:
    return ParameterProvenance(
        parameter_id=parameter_id,
        status=ParameterStatus.MANUFACTURED_TEST_FIXTURE,
        source_reference="RES-491 manufactured analytical fixture",
        uncertainty_fraction=uncertainty,
    )


def body(body_id: str, mass: float, inertia: float, role: str = "TEST_RIGID_BODY") -> RigidBodyDefinition:
    return RigidBodyDefinition(
        body_id=body_id,
        mass_kg=mass,
        inertia_kg_m2=inertia,
        mass_provenance=evidence(f"{body_id}:mass"),
        inertia_provenance=evidence(f"{body_id}:inertia"),
        role=role,
    )


def point_contact(
    contact_id: str,
    body_id: str,
    local_point: tuple[float, float],
    surface_point: tuple[float, float],
    *,
    normal: tuple[float, float] = (0.0, 1.0),
    friction: float = 0.0,
    kind: ContactKind = ContactKind.FOOT,
    side: str | None = None,
    channel: str | None = None,
    restitution: float = 0.0,
    plate_origin: tuple[float, float] = (0.0, 0.0),
) -> ContactSpec:
    return ContactSpec(
        contact_id=contact_id,
        body_a_id=body_id,
        local_point_a_m=local_point,
        surface_point_m=surface_point,
        normal_direction=normal,
        normal_frame=ContactNormalFrame.WORLD,
        friction_coefficient=friction,
        restitution=restitution,
        kind=kind,
        side=side,
        measurement_channel_id=channel,
        plate_frame=(
            ForcePlateFrame(
                plate_id=f"{side}_PLATE",
                side=side,
                vertical_channel_id=channel,
                origin_world_m=plate_origin,
                x_axis_world=(1.0, 0.0),
                z_axis_world=(0.0, 1.0),
            )
            if kind in {ContactKind.FOOT, ContactKind.HEEL, ContactKind.TOE}
            else None
        ),
    )


def world_cross(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[1] - a[1] * b[0]


class ValidationAndInertiaTests(unittest.TestCase):
    def test_body_parameters_require_finite_positive_mass_inertia_and_provenance(self) -> None:
        with self.assertRaisesRegex(DynamicsError, "mass must be positive and finite"):
            body("massless", 0.0, 1.0)
        with self.assertRaisesRegex(DynamicsError, "inertia must be positive and finite"):
            body("singular", 1.0, math.inf)
        domain = (
            (0.0, "NONPOSITIVE"),
            (-1.0, "NONPOSITIVE"),
            (math.nan, "NONFINITE"),
            (math.inf, "NONFINITE"),
            (-math.inf, "NONFINITE"),
            ("1.0", "NONNUMERIC"),
            (None, "NONNUMERIC"),
            (True, "NONNUMERIC"),
        )
        for field_name in ("mass", "inertia"):
            for value, reason in domain:
                with self.subTest(field=field_name, value=value):
                    arguments = {"mass": 1.0, "inertia": 1.0}
                    arguments[field_name] = value
                    with self.assertRaises(ParameterDomainError) as caught:
                        body("invalid", arguments["mass"], arguments["inertia"])
                    self.assertIsInstance(caught.exception, DynamicsError)
                    self.assertEqual(caught.exception.field, field_name)
                    self.assertEqual(caught.exception.reason, reason)
        with self.assertRaises(TypeError):
            RigidBodyDefinition(  # type: ignore[call-arg]
                body_id="absent_inertia",
                mass_kg=1.0,
                mass_provenance=evidence("absent:mass"),
                inertia_provenance=evidence("absent:inertia"),
            )
        valid = body("valid", 2, 1e-6)
        self.assertEqual((valid.mass_kg, valid.inertia_kg_m2), (2, 1e-6))
        with self.assertRaisesRegex(DynamicsError, "uncertainty must be finite"):
            ParameterProvenance("bad", ParameterStatus.ASSUMED_UNCALIBRATED, "source", float("nan"))

    def test_com_coordinate_mass_matrix_is_diagonal_and_matches_kinetic_energy(self) -> None:
        system = PlanarMultibodySystem(
            (body("a", 2.0, 3.0), body("b", 0.5, 0.2)),
            (),
            (),
            gravity_m_s2=9.81,
            clock_id="fixture-clock",
        )
        matrix = system.mass_matrix()
        self.assertEqual(len(matrix), 6)
        for i in range(6):
            for j in range(6):
                self.assertEqual(matrix[i][j], matrix[j][i])
                if i != j:
                    self.assertEqual(matrix[i][j], 0.0)
        state = system.make_state(
            {
                "a": RigidBodyState((1.0, 2.0), 0.3, (0.4, -0.2), 1.1),
                "b": RigidBodyState((-1.0, 0.5), -0.4, (-0.7, 0.9), -0.6),
            }
        )
        velocity = system._velocity_vector(state)
        quadratic = 0.5 * sum(
            velocity[i] * matrix[i][j] * velocity[j]
            for i in range(6)
            for j in range(6)
        )
        self.assertAlmostEqual(quadratic, system.kinetic_energy(state), places=12)

    def test_linear_solver_is_an_injected_boundary(self) -> None:
        def diagonal_oracle(matrix: list[list[float]], rhs: list[float]) -> list[float]:
            if any(matrix[i][j] != 0.0 for i in range(len(matrix)) for j in range(len(matrix)) if i != j):
                raise AssertionError("fixture expected an unconstrained diagonal mass matrix")
            return [rhs[i] / matrix[i][i] for i in range(len(rhs))]

        system = PlanarMultibodySystem(
            (body("body", 2.0, 3.0),), (), (), gravity_m_s2=9.81,
            clock_id="solver-seam", linear_solver=diagonal_oracle,
            linear_solver_id="MANUFACTURED_DIAGONAL_ORACLE",
        )
        state = system.make_state({"body": RigidBodyState((0.0, 2.0), 0.0)})
        result = system.solve_accelerations(state)
        self.assertEqual(result.linear_solver_id, "MANUFACTURED_DIAGONAL_ORACLE")
        self.assertAlmostEqual(result.accelerations["body"].linear_m_s2[1], -9.81)
        self.assertEqual(result.parameter_uncertainty_fractions["body:mass"], 0.02)
        self.assertEqual(result.uncertainty_policy, "INPUT_BOUNDS_REPORTED_NOT_PROPAGATED")
        with self.assertRaisesRegex(DynamicsError, "requires a stable identity"):
            PlanarMultibodySystem(
                (body("unlabeled", 1.0, 0.1),), (), (), gravity_m_s2=9.81,
                clock_id="missing-solver-id", linear_solver=diagonal_oracle,
            )

    def test_center_of_mass_and_momentum_use_declared_body_masses(self) -> None:
        system = PlanarMultibodySystem(
            (body("heavy", 2.0, 0.4), body("light", 1.0, 0.2)),
            (), (), gravity_m_s2=9.81, clock_id="mass-weighted-fixture",
        )
        state = system.make_state({
            "heavy": RigidBodyState((2.0, 1.0), 0.0, (3.0, 2.0)),
            "light": RigidBodyState((-1.0, 4.0), 0.0, (-1.0, 4.0)),
        })
        self.assertEqual(system.center_of_mass(state), (1.0, 2.0))
        self.assertAlmostEqual(system.linear_momentum(state)[0], 5.0)
        self.assertAlmostEqual(system.linear_momentum(state)[1], 8.0)


class NewtonEulerAndContactTests(unittest.TestCase):
    def test_articulated_left_right_legs_keep_independent_plate_wrenches(self) -> None:
        g = 9.81
        definitions = (
            body("hat", 4.0, 1.0, "MANUFACTURED_HAT_SEGMENT"),
            body("left_thigh", 1.0, 0.2, "MANUFACTURED_LEFT_THIGH"),
            body("left_shank", 0.7, 0.1, "MANUFACTURED_LEFT_SHANK"),
            body("left_foot", 0.3, 0.03, "MANUFACTURED_LEFT_FOOT"),
            body("right_thigh", 1.0, 0.2, "MANUFACTURED_RIGHT_THIGH"),
            body("right_shank", 0.7, 0.1, "MANUFACTURED_RIGHT_SHANK"),
            body("right_foot", 0.3, 0.03, "MANUFACTURED_RIGHT_FOOT"),
        )
        joints = (
            PlanarJoint("left_hip", "hat", "left_thigh", (-0.15, -0.4), (0.0, 0.0)),
            PlanarJoint("left_knee", "left_thigh", "left_shank", (0.0, -0.4), (0.0, 0.05)),
            PlanarJoint("left_ankle", "left_shank", "left_foot", (0.0, -0.3), (0.0, 0.15)),
            PlanarJoint("right_hip", "hat", "right_thigh", (0.15, -0.4), (0.0, 0.0)),
            PlanarJoint("right_knee", "right_thigh", "right_shank", (0.0, -0.4), (0.0, 0.05)),
            PlanarJoint("right_ankle", "right_shank", "right_foot", (0.0, -0.3), (0.0, 0.15)),
        )
        contacts = (
            point_contact("left_sole", "left_foot", (0.0, -0.1), (0.0, 0.0),
                          kind=ContactKind.FOOT, side="LEFT", channel="FZ_LEFT",
                          plate_origin=(-0.15, 0.0)),
            point_contact("right_sole", "right_foot", (0.0, -0.1), (0.0, 0.0),
                          kind=ContactKind.FOOT, side="RIGHT", channel="FZ_RIGHT",
                          plate_origin=(0.15, 0.0)),
            point_contact("rack", "hat", (0.0, 0.0), (0.0, 1.4),
                          normal=(1.0, 0.0), kind=ContactKind.RACK),
        )
        system = PlanarMultibodySystem(
            definitions, joints, contacts, gravity_m_s2=g, clock_id="bilateral-segment-fixture"
        )
        states = {
            "hat": RigidBodyState((0.0, 1.4), 0.0),
            "left_thigh": RigidBodyState((-0.15, 1.0), 0.0),
            "left_shank": RigidBodyState((-0.15, 0.55), 0.0),
            "left_foot": RigidBodyState((-0.15, 0.1), 0.0),
            "right_thigh": RigidBodyState((0.15, 1.0), 0.0),
            "right_shank": RigidBodyState((0.15, 0.55), 0.0),
            "right_foot": RigidBodyState((0.15, 0.1), 0.0),
        }
        active = {
            contact_id: ContactState(ContactMode.SLIDE, slip_direction=1)
            for contact_id in ("left_sole", "right_sole", "rack")
        }
        state = system.make_state(states, active_contacts=active)
        result = system.solve_accelerations(
            state,
            external_wrenches=(AppliedWrench("hat", force_n=(-10.0, 0.0), source_id="fixture-rack-push"),),
            support_contract=SupportContract(
                "BILATERAL_FEET_AND_RACK",
                ("left_sole", "right_sole", "rack"),
                ("left_sole", "right_sole", "rack"),
            ),
        )
        self.assertTrue(all(
            abs(value) < 1e-8
            for acceleration in result.accelerations.values()
            for value in (*acceleration.linear_m_s2, acceleration.angular_rad_s2)
        ))
        left = result.contact_wrenches["left_sole"]
        right = result.contact_wrenches["right_sole"]
        self.assertEqual(left.measurement_channel_id, "FZ_LEFT")
        self.assertEqual(right.measurement_channel_id, "FZ_RIGHT")
        self.assertEqual(left.plate_id, "LEFT_PLATE")
        self.assertEqual(right.plate_id, "RIGHT_PLATE")
        self.assertAlmostEqual(left.normal_force_n + right.normal_force_n, 8.0 * g, places=8)
        self.assertAlmostEqual(result.contact_wrenches["rack"].normal_force_n, 10.0, places=8)

    def test_free_body_newton_euler_matches_independent_wrench_sum(self) -> None:
        mass, inertia, gravity = 2.0, 3.0, 9.81
        definition = body("load", mass, inertia)
        system = PlanarMultibodySystem((definition,), (), (), gravity_m_s2=gravity, clock_id="analytic")
        initial = RigidBodyState((0.2, 0.4), 0.3, (0.0, 0.0), 0.0)
        state = system.make_state({"load": initial})
        external = AppliedWrench(
            "load",
            force_n=(4.0, 6.0),
            local_point_m=(0.5, -0.2),
            moment_n_m=2.0,
            source_id="independent-known-answer",
        )
        result = system.solve_accelerations(state, external_wrenches=(external,))
        acceleration = result.accelerations["load"]
        c, s = math.cos(initial.angle_rad), math.sin(initial.angle_rad)
        rx = c * 0.5 - s * -0.2
        rz = s * 0.5 + c * -0.2
        expected_alpha = (world_cross((rx, rz), external.force_n) + external.moment_n_m) / inertia
        self.assertAlmostEqual(acceleration.linear_m_s2[0], 4.0 / mass, places=12)
        self.assertAlmostEqual(acceleration.linear_m_s2[1], 6.0 / mass - gravity, places=12)
        self.assertAlmostEqual(acceleration.angular_rad_s2, expected_alpha, places=12)

        linear_rate = (
            mass * acceleration.linear_m_s2[0],
            mass * acceleration.linear_m_s2[1],
        )
        expected_force = (external.force_n[0], external.force_n[1] - mass * gravity)
        self.assertAlmostEqual(linear_rate[0], expected_force[0], places=12)
        self.assertAlmostEqual(linear_rate[1], expected_force[1], places=12)
        angular_rate = mass * world_cross(initial.position_m, acceleration.linear_m_s2) + inertia * acceleration.angular_rad_s2
        gravity_moment = world_cross(initial.position_m, (0.0, -mass * gravity))
        applied_point = (initial.position_m[0] + rx, initial.position_m[1] + rz)
        external_moment = world_cross(applied_point, external.force_n) + external.moment_n_m
        self.assertAlmostEqual(angular_rate, gravity_moment + external_moment, places=12)

    def test_two_independent_feet_and_rack_support_have_distinct_wrenches(self) -> None:
        g = 9.81
        definitions = (body("torso", 5.0, 1.0, "TORSO_TEST_SEGMENT"),
                       body("left_leg", 1.0, 0.2, "LEFT_LEG_TEST_SEGMENT"),
                       body("right_leg", 1.0, 0.2, "RIGHT_LEG_TEST_SEGMENT"))
        joints = (
            PlanarJoint("left_hip", "torso", "left_leg", (-0.35, -0.5), (0.0, 0.0)),
            PlanarJoint("right_hip", "torso", "right_leg", (0.35, -0.5), (0.0, 0.0)),
        )
        contacts = (
            point_contact("left_foot", "left_leg", (0.0, -0.5), (0.0, 0.0),
                          side="LEFT", channel="FZ_LEFT"),
            point_contact("right_foot", "right_leg", (0.0, -0.5), (0.0, 0.0),
                          side="RIGHT", channel="FZ_RIGHT"),
            point_contact("rack", "torso", (0.0, 0.0), (0.0, 1.0),
                          normal=(1.0, 0.0), kind=ContactKind.RACK),
        )
        system = PlanarMultibodySystem(
            definitions, joints, contacts, gravity_m_s2=g, clock_id="manufactured-world-time"
        )
        states = {
            "torso": RigidBodyState((0.0, 1.0), 0.0),
            "left_leg": RigidBodyState((-0.35, 0.5), 0.0),
            "right_leg": RigidBodyState((0.35, 0.5), 0.0),
        }
        active = {name: ContactState(ContactMode.SLIDE, slip_direction=1)
                  for name in ("left_foot", "right_foot", "rack")}
        state = system.make_state(states, active_contacts=active)
        contract = SupportContract(
            "TWO_FEET_PLUS_RACK",
            ("left_foot", "right_foot", "rack"),
            ("left_foot", "right_foot", "rack"),
        )
        external = AppliedWrench("torso", force_n=(-10.0, 0.0), source_id="manufactured-push")
        result = system.solve_accelerations(state, external_wrenches=(external,), support_contract=contract)
        self.assertTrue(all(abs(v) < 1e-9 for a in result.accelerations.values()
                            for v in (*a.linear_m_s2, a.angular_rad_s2)))
        left = result.contact_wrenches["left_foot"]
        right = result.contact_wrenches["right_foot"]
        rack = result.contact_wrenches["rack"]
        self.assertEqual((left.side, left.measurement_channel_id), ("LEFT", "FZ_LEFT"))
        self.assertEqual((right.side, right.measurement_channel_id), ("RIGHT", "FZ_RIGHT"))
        self.assertEqual((left.plate_id, right.plate_id), ("LEFT_PLATE", "RIGHT_PLATE"))
        self.assertEqual(left.force_in_plate_frame_n, (0.0, left.normal_force_n))
        self.assertEqual(right.force_in_plate_frame_n, (0.0, right.normal_force_n))
        self.assertGreater(left.normal_force_n, 0.0)
        self.assertGreater(right.normal_force_n, 0.0)
        self.assertAlmostEqual(left.normal_force_n + right.normal_force_n, 7.0 * g, places=9)
        self.assertAlmostEqual(rack.normal_force_n, 10.0, places=9)
        self.assertTrue(all(value < 1e-9 for value in result.constraint_residuals.values()))

        total_acceleration = (
            sum(definition.mass_kg * result.accelerations[definition.body_id].linear_m_s2[0]
                for definition in definitions),
            sum(definition.mass_kg * result.accelerations[definition.body_id].linear_m_s2[1]
                for definition in definitions),
        )
        external_x = external.force_n[0] + left.force_a_world_n[0] + right.force_a_world_n[0] + rack.force_a_world_n[0]
        external_z = external.force_n[1] + left.force_a_world_n[1] + right.force_a_world_n[1] + rack.force_a_world_n[1] - 7.0 * g
        self.assertAlmostEqual(total_acceleration[0], external_x, places=9)
        self.assertAlmostEqual(total_acceleration[1], external_z, places=9)

        incomplete = system.make_state(states, active_contacts={
            "left_foot": active["left_foot"],
            "right_foot": ContactState(),
            "rack": active["rack"],
        })
        with self.assertRaises(SupportIncompleteError):
            system.validate_support_contract(incomplete, contract)

    def test_multiple_foot_contacts_can_share_one_explicit_plate_channel(self) -> None:
        left_frame = ForcePlateFrame(
            "LEFT_PLATE", "LEFT", "FZ_LEFT", (0.0, 0.0), (1.0, 0.0), (0.0, 1.0)
        )
        contacts = (
            ContactSpec("left_heel", "body", (-0.1, -0.1), (0.0, 1.0), 0.6, 0.0,
                        ContactKind.HEEL, (0.0, 0.0), side="LEFT",
                        measurement_channel_id="FZ_LEFT", plate_frame=left_frame),
            ContactSpec("left_toe", "body", (0.1, -0.1), (0.0, 1.0), 0.6, 0.0,
                        ContactKind.TOE, (0.0, 0.0), side="LEFT",
                        measurement_channel_id="FZ_LEFT", plate_frame=left_frame),
        )
        system = PlanarMultibodySystem(
            (body("body", 1.0, 0.2),), (), contacts, gravity_m_s2=9.81, clock_id="shared-plate"
        )
        self.assertEqual(len(system.contacts), 2)

    def test_support_reaction_transfers_when_a_foot_would_need_adhesion(self) -> None:
        g, mass, inertia = 9.81, 4.0, 10.0
        system = PlanarMultibodySystem(
            (body("platform", mass, inertia),),
            (),
            (
                point_contact("left", "platform", (-0.5, -0.5), (0.0, 0.0),
                              side="LEFT", channel="FZ_LEFT"),
                point_contact("right", "platform", (0.5, -0.5), (0.0, 0.0),
                              side="RIGHT", channel="FZ_RIGHT"),
            ),
            gravity_m_s2=g,
            clock_id="analytic-transfer",
        )
        active = {
            "left": ContactState(ContactMode.SLIDE, slip_direction=1),
            "right": ContactState(ContactMode.SLIDE, slip_direction=1),
        }
        state = system.make_state(
            {"platform": RigidBodyState((0.0, 0.5), 0.0)},
            active_contacts=active,
        )
        baseline = system.solve_accelerations(state)
        self.assertAlmostEqual(baseline.contact_wrenches["left"].normal_force_n, mass * g / 2, places=9)
        self.assertAlmostEqual(baseline.contact_wrenches["right"].normal_force_n, mass * g / 2, places=9)

        tipped = system.solve_accelerations(
            state,
            external_wrenches=(AppliedWrench("platform", moment_n_m=25.0, source_id="manufactured-tip"),),
        )
        self.assertEqual(tipped.contact_states["right"].mode, ContactMode.OPEN)
        self.assertGreater(tipped.contact_wrenches["left"].normal_force_n, 0.0)
        self.assertEqual(tipped.contact_wrenches["right"].normal_force_n, 0.0)
        self.assertGreater(tipped.accelerations["platform"].angular_rad_s2, 0.0)
        normal_sum = sum(w.normal_force_n for w in tipped.contact_wrenches.values())
        vertical_external = normal_sum - mass * g
        self.assertAlmostEqual(
            mass * tipped.accelerations["platform"].linear_m_s2[1],
            vertical_external,
            places=9,
        )
        with self.assertRaises(SupportIncompleteError):
            system.solve_accelerations(
                state,
                external_wrenches=(AppliedWrench("platform", moment_n_m=25.0, source_id="manufactured-tip"),),
                support_contract=SupportContract("BOTH_FEET_REQUIRED", ("left", "right"), ("left", "right")),
            )


class FrictionImpactAndActuationTests(unittest.TestCase):
    def test_coulomb_cone_switches_stick_to_sliding_without_exceeding_capacity(self) -> None:
        system = PlanarMultibodySystem(
            (body("block", 2.0, 1.0),),
            (),
            (point_contact("sole", "block", (0.0, -0.5), (0.0, 0.0),
                           friction=0.5, kind=ContactKind.FOOT, side="LEFT", channel="FZ_LEFT"),),
            gravity_m_s2=9.81,
            clock_id="friction-fixture",
        )
        state = system.make_state(
            {"block": RigidBodyState((0.0, 0.5), 0.0)},
            active_contacts=("sole",),
        )
        result = system.solve_accelerations(
            state,
            external_wrenches=(AppliedWrench("block", force_n=(20.0, 0.0), source_id="manufactured-force"),),
        )
        contact = result.contact_wrenches["sole"]
        self.assertEqual(contact.mode, ContactMode.SLIDE)
        self.assertAlmostEqual(contact.normal_force_n, 2.0 * 9.81, places=9)
        self.assertAlmostEqual(abs(contact.tangential_force_n), 0.5 * contact.normal_force_n, places=9)
        self.assertGreater(result.accelerations["block"].linear_m_s2[0], 0.0)
        self.assertAlmostEqual(result.contact_power_w, 0.0, places=12)
        moving = system.make_state(
            {"block": RigidBodyState((0.0, 0.5), 0.0, (1.0, 0.0))},
            active_contacts=result.contact_states,
        )
        sliding = system.solve_accelerations(
            moving,
            external_wrenches=(AppliedWrench("block", force_n=(20.0, 0.0), source_id="manufactured-force"),),
        )
        self.assertLess(sliding.contact_power_w, 0.0)

    def test_catch_impact_is_explicit_simultaneous_and_dissipative(self) -> None:
        torso = body("torso", 2.0, 0.5)
        bar = body("bar", 1.0, 0.1, "LOAD_TEST_BODY")
        contact = ContactSpec(
            contact_id="bar_to_torso_catch",
            body_a_id="bar",
            local_point_a_m=(0.0, 0.0),
            body_b_id="torso",
            local_point_b_m=(0.05, 0.0),
            normal_direction=(1.0, 0.0),
            normal_frame=ContactNormalFrame.BODY_B,
            friction_coefficient=0.0,
            restitution=0.0,
            kind=ContactKind.CATCH,
        )
        system = PlanarMultibodySystem((torso, bar), (), (contact,), gravity_m_s2=9.81, clock_id="catch-event")
        state = system.make_state(
            {
                "torso": RigidBodyState((0.0, 1.0), math.pi / 2.0, (0.0, 0.0)),
                "bar": RigidBodyState((0.0, 1.05), 0.0, (0.0, -2.0)),
            }
        )
        after, event = system.resolve_impact(state, ("bar_to_torso_catch",))
        self.assertEqual(event.event_id, "CONTACT_IMPACT")
        self.assertEqual(event.contact_ids, ("bar_to_torso_catch",))
        self.assertAlmostEqual(after.bodies["torso"].velocity_m_s[1], -2.0 / 3.0, places=10)
        self.assertAlmostEqual(after.bodies["bar"].velocity_m_s[1], -2.0 / 3.0, places=10)
        momentum_before = system.linear_momentum(state)[1]
        momentum_after = system.linear_momentum(after)[1]
        self.assertAlmostEqual(momentum_after, momentum_before, places=10)
        self.assertAlmostEqual(event.contact_work_j, system.kinetic_energy(after) - system.kinetic_energy(state), places=12)
        self.assertLessEqual(event.contact_work_j, 0.0)
        wrench = system.solve_accelerations(after).contact_wrenches["bar_to_torso_catch"]
        self.assertEqual(wrench.kind, ContactKind.CATCH)
        self.assertAlmostEqual(wrench.normal_world[0], 0.0, places=12)
        self.assertAlmostEqual(wrench.normal_world[1], 1.0, places=12)
        self.assertEqual(wrench.force_b_world_n, (-wrench.force_a_world_n[0], -wrench.force_a_world_n[1]))

    def test_back_rack_load_connection_and_activation_uncertainty_are_explicit(self) -> None:
        actuator = ActuatorHypothesis(
            actuator_id="hip_motor",
            joint_id="hip",
            capacity_n_m=12.0,
            activation_time_s=0.04,
            deactivation_time_s=0.08,
            provenance=evidence("test-torque-capacity", 0.1),
        )
        system = PlanarMultibodySystem(
            (
                body("torso", 5.0, 1.0),
                body("leg", 1.0, 0.2, "LEG_TEST_SEGMENT"),
                body("bar", 1.0, 0.2, "LOAD_TEST_BODY"),
            ),
            (
                PlanarJoint("hip", "torso", "leg", (0.0, -0.3), (0.0, 0.3)),
                PlanarJoint(
                    "back_rack", "torso", "bar", (0.0, -0.2), (0.0, 0.2),
                    kind=JointKind.WELD, connection_kind=ConnectionKind.BACK_RACK,
                ),
            ),
            (),
            gravity_m_s2=9.81,
            clock_id="actuator-hypothesis",
            actuators=(actuator,),
        )
        state = system.make_state(
            {
                "torso": RigidBodyState((0.0, 1.0), 0.0),
                "leg": RigidBodyState((0.0, 0.4), 0.0),
                "bar": RigidBodyState((0.0, 0.6), 0.0),
            },
            activations={"hip_motor": 0.5},
        )
        before = system.solve_accelerations(state, excitations={"hip_motor": 1.0})
        self.assertAlmostEqual(before.actuator_torques_n_m["hip_motor"], 6.0)
        self.assertAlmostEqual(before.actuator_capacity_bounds_n_m["hip_motor"][0], 5.4)
        self.assertAlmostEqual(before.actuator_capacity_bounds_n_m["hip_motor"][1], 6.6)
        self.assertIn(ParameterStatus.MANUFACTURED_TEST_FIXTURE.value, before.parameter_statuses)
        advanced = system.advance(state, 0.01, excitations={"hip_motor": 1.0})
        expected_activation = 1.0 + (0.5 - 1.0) * math.exp(-0.01 / 0.04)
        self.assertAlmostEqual(advanced.state.activations["hip_motor"], expected_activation, places=12)
        after = system.solve_accelerations(advanced.state, excitations={"hip_motor": 1.0})
        self.assertAlmostEqual(
            after.actuator_torques_n_m["hip_motor"],
            12.0 * expected_activation,
            places=12,
        )
        self.assertTrue(all(
            value < 1e-9
            for name, value in after.constraint_residuals.items()
            if name.startswith("hip:") or name.startswith("back_rack:")
        ))

    def test_feedback_reference_controller_tracks_a_declared_joint_target(self) -> None:
        actuator = ActuatorHypothesis(
            "motor", "hinge", 10.0, 0.02, 0.04, evidence("motor-capacity", 0.1)
        )
        system = PlanarMultibodySystem(
            (body("base", 1.0, 1.0), body("link", 1.0, 1.0)),
            (PlanarJoint("hinge", "base", "link", (0.0, 0.0), (0.0, 0.0)),),
            (),
            gravity_m_s2=9.81,
            clock_id="reference-control-test",
            actuators=(actuator,),
        )
        state = system.make_state(
            {
                "base": RigidBodyState((0.0, 1.0), 0.0),
                "link": RigidBodyState((0.0, 1.0), 0.0),
            }
        )
        controller = ReferenceAngleController(
            "manufactured-pd",
            "motor",
            "hinge",
            reference_angle_rad=0.2,
            reference_velocity_rad_s=0.0,
            proportional_gain_n_m_per_rad=4.0,
            derivative_gain_n_m_s_per_rad=2.0,
            provenance=evidence("manufactured-pd-gains", 0.1),
        )
        initial_error = abs(system.joint_state(state, "hinge")[0] - 0.2)
        for _ in range(1000):
            command = controller.excitation(system, state)
            self.assertLessEqual(abs(command), 1.0)
            state = system.advance(state, 0.002, excitations={"motor": command}).state
        final_error = abs(system.joint_state(state, "hinge")[0] - 0.2)
        self.assertLess(final_error, initial_error * 0.1)


class EventAndTopologyTests(unittest.TestCase):
    def test_body_attached_contact_axes_match_finite_difference_kinematics(self) -> None:
        spec = ContactSpec(
            contact_id="rotating_catch_surface",
            body_a_id="bar",
            local_point_a_m=(-0.1, 0.03),
            body_b_id="torso",
            local_point_b_m=(0.08, -0.02),
            normal_direction=(1.0, 0.0),
            normal_frame=ContactNormalFrame.BODY_B,
            friction_coefficient=0.4,
            restitution=0.0,
            kind=ContactKind.CATCH,
        )
        system = PlanarMultibodySystem(
            (body("bar", 1.0, 0.1), body("torso", 5.0, 1.0)),
            (), (spec,), gravity_m_s2=9.81, clock_id="rotating-contact-frame",
        )
        state = system.make_state({
            "bar": RigidBodyState((0.2, 0.3), 0.2, (0.5, -0.2), 0.3),
            "torso": RigidBodyState((-0.1, 0.0), 0.4, (-0.1, 0.1), -0.7),
        })
        now = system.contact_kinematics(state, spec.contact_id)
        self.assertAlmostEqual(now.normal_world[0], math.cos(0.4), places=12)
        self.assertAlmostEqual(now.normal_world[1], math.sin(0.4), places=12)
        step_s = 1e-7
        moved_bodies = {
            body_id: RigidBodyState(
                (body_state.position_m[0] + body_state.velocity_m_s[0] * step_s,
                 body_state.position_m[1] + body_state.velocity_m_s[1] * step_s),
                body_state.angle_rad + body_state.angular_velocity_rad_s * step_s,
                body_state.velocity_m_s,
                body_state.angular_velocity_rad_s,
            )
            for body_id, body_state in state.bodies.items()
        }
        moved = system.make_state(moved_bodies, time_s=step_s, active_contacts=state.contacts)
        later = system.contact_kinematics(moved, spec.contact_id)
        finite_difference_normal = (later.gap_m - now.gap_m) / step_s
        self.assertAlmostEqual(finite_difference_normal, now.normal_velocity_m_s, delta=1e-6)

    def test_rotating_contact_frame_acceleration_jacobian_closes(self) -> None:
        contact = ContactSpec(
            contact_id="catch_surface",
            body_a_id="bar",
            local_point_a_m=(0.0, 0.0),
            body_b_id="torso",
            local_point_b_m=(0.05, 0.0),
            normal_direction=(1.0, 0.0),
            normal_frame=ContactNormalFrame.BODY_B,
            friction_coefficient=0.8,
            restitution=0.0,
            kind=ContactKind.CATCH,
        )
        system = PlanarMultibodySystem(
            (body("bar", 1.0, 0.1), body("torso", 5.0, 1.0)),
            (), (contact,), gravity_m_s2=9.81, clock_id="rotating-contact-constraint",
        )
        state = system.make_state(
            {
                "bar": RigidBodyState((0.0, 1.05), 0.0, (-0.035, 0.0), 0.0),
                "torso": RigidBodyState((0.0, 1.0), math.pi / 2.0, (0.0, 0.0), 0.7),
            },
            active_contacts=("catch_surface",),
        )
        result = system.solve_accelerations(state)
        self.assertTrue(all(value < 1e-8 for value in result.constraint_residuals.values()))

    def test_falling_contact_event_time_converges_and_impact_energy_is_booked(self) -> None:
        definition = body("falling_body", 1.0, 0.1)
        contact = point_contact(
            "floor",
            "falling_body",
            (0.0, -0.05),
            (0.0, 0.0),
            kind=ContactKind.HEEL,
            side="LEFT",
            channel="FZ_LEFT",
            restitution=0.0,
        )
        system = PlanarMultibodySystem((definition,), (), (contact,), gravity_m_s2=9.81, clock_id="root-finding")
        exact_time = math.sqrt(2.0 * 0.95 / 9.81)

        def locate(step_size: float) -> tuple[float, float]:
            state = system.make_state(
                {"falling_body": RigidBodyState((0.0, 1.0), 0.0)}
            )
            for _ in range(1000):
                step = system.advance(state, step_size)
                state = step.state
                if any(event.event_id == "CONTACT_IMPACT" for event in step.events):
                    event = next(event for event in step.events if event.event_id == "CONTACT_IMPACT")
                    return state.time_s, event.contact_work_j
            self.fail("falling body never reached the floor")
            raise AssertionError

        coarse_time, _ = locate(0.04)
        fine_time, impact_work = locate(0.02)
        self.assertLess(abs(fine_time - exact_time), abs(coarse_time - exact_time))
        self.assertLess(abs(fine_time - exact_time), 0.01)
        self.assertLessEqual(impact_work, 0.0)

    def test_simultaneous_two_foot_impact_is_resolved_once_with_separate_impulses(self) -> None:
        system = PlanarMultibodySystem(
            (body("platform", 2.0, 1.0),),
            (),
            (
                point_contact("left", "platform", (-0.4, -0.5), (0.0, 0.0),
                              side="LEFT", channel="FZ_LEFT"),
                point_contact("right", "platform", (0.4, -0.5), (0.0, 0.0),
                              side="RIGHT", channel="FZ_RIGHT"),
            ),
            gravity_m_s2=9.81,
            clock_id="simultaneous-impact",
        )
        state = system.make_state({
            "platform": RigidBodyState((0.0, 0.5), 0.0, (0.0, -1.5))
        })
        step = system.advance(state, 0.01)
        self.assertEqual(len(step.events), 1)
        event = step.events[0]
        self.assertEqual(event.event_id, "CONTACT_IMPACT")
        self.assertEqual(set(event.contact_ids), {"left", "right"})
        self.assertAlmostEqual(sum(event.normal_impulses_n_s.values()), 3.0, places=10)
        self.assertAlmostEqual(event.normal_impulses_n_s["left"], event.normal_impulses_n_s["right"], places=10)
        self.assertLessEqual(event.contact_work_j, 0.0)
        self.assertAlmostEqual(step.state.bodies["platform"].velocity_m_s[1], 0.0, places=10)

    def test_bar_floor_contact_releases_instead_of_pulling_on_a_lifted_load(self) -> None:
        mass, gravity = 2.0, 9.81
        floor = point_contact(
            "bar_floor", "bar", (0.0, -0.1), (0.0, 0.0),
            kind=ContactKind.BAR_FLOOR,
        )
        system = PlanarMultibodySystem(
            (body("bar", mass, 0.1, "LOAD_TEST_BODY"),), (), (floor,),
            gravity_m_s2=gravity, clock_id="bar-floor-support",
        )
        state = system.make_state(
            {"bar": RigidBodyState((0.0, 0.1), 0.0)},
            active_contacts={"bar_floor": ContactState(ContactMode.SLIDE, slip_direction=1)},
        )
        supported = system.solve_accelerations(state)
        self.assertAlmostEqual(supported.contact_wrenches["bar_floor"].normal_force_n, mass * gravity, places=10)
        lifted = system.solve_accelerations(
            state,
            external_wrenches=(AppliedWrench("bar", force_n=(0.0, 3.0 * mass * gravity), source_id="declared-lift"),),
        )
        self.assertEqual(lifted.contact_states["bar_floor"].mode, ContactMode.OPEN)
        self.assertEqual(lifted.contact_wrenches["bar_floor"].normal_force_n, 0.0)
        self.assertAlmostEqual(lifted.accelerations["bar"].linear_m_s2[1], 2.0 * gravity, places=10)

    def test_penetration_and_unknown_support_paths_fail_closed(self) -> None:
        contact = point_contact(
            "foot", "body", (0.0, -0.1), (0.0, 0.0),
            side="LEFT", channel="FZ_LEFT",
        )
        system = PlanarMultibodySystem((body("body", 1.0, 0.2),), (), (contact,), gravity_m_s2=9.81, clock_id="scope")
        penetrating = system.make_state({"body": RigidBodyState((0.0, 0.05), 0.0)})
        with self.assertRaisesRegex(ConstraintSolveError, "penetration"):
            system.solve_accelerations(penetrating)
        clean = system.make_state({"body": RigidBodyState((0.0, 0.5), 0.0)})
        contract = SupportContract("BILATERAL_FEET", ("missing",), ("missing",))
        with self.assertRaisesRegex(DynamicsError, "unknown contacts"):
            system.validate_support_contract(clean, contract)

    def test_invalid_contact_geometry_and_unknown_control_fail_closed(self) -> None:
        with self.assertRaisesRegex(DynamicsError, "unit vector"):
            ContactSpec(
                contact_id="bad_normal", body_a_id="body", local_point_a_m=(0.0, -0.1),
                normal_direction=(0.0, 2.0), friction_coefficient=0.5, restitution=0.0,
                kind=ContactKind.RACK, surface_point_m=(0.0, 0.0),
            )
        system = PlanarMultibodySystem(
            (body("body", 1.0, 0.2),), (), (), gravity_m_s2=9.81, clock_id="bad-control"
        )
        state = system.make_state({"body": RigidBodyState((0.0, 1.0), 0.0)})
        with self.assertRaisesRegex(DynamicsError, "unknown actuator"):
            system.solve_accelerations(state, excitations={"unregistered": 1.0})


class IndependentReviewerFalsifierTests(unittest.TestCase):
    """Reviewer-requested physics falsifiers added during the RES-491 third repair."""

    @staticmethod
    def _two_foot_platform(*, friction: float = 0.0, mirror: bool = False) -> PlanarMultibodySystem:
        sign = -1.0 if mirror else 1.0
        return PlanarMultibodySystem(
            (body("platform", 2.0, 1.0),),
            (),
            (
                point_contact("left", "platform", (-0.4 * sign, -0.5), (0.0, 0.0), friction=friction,
                              side="LEFT", channel="FZ_LEFT", plate_origin=(-0.4 * sign, 0.0)),
                point_contact("right", "platform", (0.4 * sign, -0.5), (0.0, 0.0), friction=friction,
                              side="RIGHT", channel="FZ_RIGHT", plate_origin=(0.4 * sign, 0.0)),
            ),
            gravity_m_s2=9.81,
            clock_id="reviewer-two-foot-platform",
        )

    def test_rotating_frame_rates_and_accelerations_match_central_differences(self) -> None:
        rng = random.Random(491)
        for frame in (ContactNormalFrame.BODY_A, ContactNormalFrame.BODY_B):
            spec = ContactSpec(
                contact_id="moving", body_a_id="bar", local_point_a_m=(-0.1, 0.03),
                normal_direction=(0.6, 0.8), friction_coefficient=0.4, restitution=0.0,
                kind=ContactKind.CATCH, body_b_id="torso", local_point_b_m=(0.08, -0.02),
                normal_frame=frame,
            )
            system = PlanarMultibodySystem(
                (body("bar", 1.0, 0.1), body("torso", 5.0, 1.0)), (), (spec,),
                gravity_m_s2=9.81, clock_id="reviewer-fd",
            )
            for _ in range(25):
                def random_body() -> RigidBodyState:
                    return RigidBodyState(
                        (rng.uniform(-1, 1), rng.uniform(-1, 1)), rng.uniform(-3, 3),
                        (rng.uniform(-2, 2), rng.uniform(-2, 2)), rng.uniform(-6, 6),
                    )

                state = system.make_state({"bar": random_body(), "torso": random_body()})
                qdd = [rng.uniform(-5, 5) for _ in range(6)]
                h = 1e-6

                def shifted(sign: float):
                    moved = {}
                    for index, body_id in enumerate(("bar", "torso")):
                        b = state.bodies[body_id]
                        a = qdd[3 * index:3 * index + 3]
                        moved[body_id] = RigidBodyState(
                            (b.position_m[0] + sign * h * b.velocity_m_s[0] + 0.5 * h * h * a[0],
                             b.position_m[1] + sign * h * b.velocity_m_s[1] + 0.5 * h * h * a[1]),
                            b.angle_rad + sign * h * b.angular_velocity_rad_s + 0.5 * h * h * a[2],
                            (b.velocity_m_s[0] + sign * h * a[0], b.velocity_m_s[1] + sign * h * a[1]),
                            b.angular_velocity_rad_s + sign * h * a[2],
                        )
                    return system._contact_data("moving", system.make_state(moved))

                now, plus, minus = system._contact_data("moving", state), shifted(1.0), shifted(-1.0)
                self.assertAlmostEqual((plus.gap_m - minus.gap_m) / (2 * h), now.normal_velocity_m_s, delta=1e-7)
                self.assertAlmostEqual(
                    (plus.tangent_coordinate_m - minus.tangent_coordinate_m) / (2 * h),
                    now.tangential_velocity_m_s, delta=1e-7,
                )
                normal_acc = sum(j * a for j, a in zip(now.normal_jacobian, qdd)) + now.normal_bias
                tangent_acc = sum(j * a for j, a in zip(now.tangential_jacobian, qdd)) + now.tangential_bias
                self.assertAlmostEqual(
                    (plus.normal_velocity_m_s - minus.normal_velocity_m_s) / (2 * h), normal_acc, delta=1e-6
                )
                self.assertAlmostEqual(
                    (plus.tangential_velocity_m_s - minus.tangential_velocity_m_s) / (2 * h),
                    tangent_acc, delta=1e-6,
                )

    def test_sliding_friction_power_is_negative_mu_n_times_slip_speed(self) -> None:
        system = PlanarMultibodySystem(
            (body("block", 2.0, 0.1),), (),
            (point_contact("pad", "block", (0.0, 0.0), (0.0, 0.0), friction=0.5,
                           side="LEFT", channel="FZ_LEFT"),),
            gravity_m_s2=9.81, clock_id="reviewer-slip",
        )
        state = system.make_state(
            {"block": RigidBodyState((0.0, 0.0), 0.0, (1.2, 0.0))},
            active_contacts={"pad": ContactState(ContactMode.SLIDE, slip_direction=1)},
        )
        result = system.solve_accelerations(state)
        wrench = result.contact_wrenches["pad"]
        self.assertAlmostEqual(wrench.normal_force_n, 2.0 * 9.81, places=10)
        self.assertAlmostEqual(result.contact_power_w, -0.5 * 2.0 * 9.81 * 1.2, places=10)
        self.assertAlmostEqual(result.accelerations["block"].linear_m_s2[0], -0.5 * 9.81, places=10)

    def test_redundant_stick_supports_on_one_rigid_body_fail_closed(self) -> None:
        system = self._two_foot_platform(friction=0.8)
        state = system.make_state(
            {"platform": RigidBodyState((0.0, 0.5), 0.0)}, active_contacts=("left", "right")
        )
        with self.assertRaisesRegex(ConstraintSolveError, "singular or redundant"):
            system.solve_accelerations(
                state, external_wrenches=(AppliedWrench("platform", "push", (5.0, 0.0)),)
            )

    def test_mirrored_geometry_is_equivariant_for_left_right_wrenches(self) -> None:
        loads = {}
        for mirror in (False, True):
            sign = -1.0 if mirror else 1.0
            system = self._two_foot_platform(mirror=mirror)
            sliding = ContactState(ContactMode.SLIDE, slip_direction=1)
            state = system.make_state(
                {"platform": RigidBodyState((0.0, 0.5), 0.0)},
                active_contacts={"left": sliding, "right": sliding},
            )
            load = AppliedWrench("platform", "load", (0.0, -10.0), (0.2 * sign, 0.0))
            result = system.solve_accelerations(state, external_wrenches=(load,))
            loads[mirror] = {k: v.normal_force_n for k, v in result.contact_wrenches.items()}
        self.assertAlmostEqual(loads[False]["left"], 12.31, places=9)
        self.assertAlmostEqual(loads[False]["right"], 17.31, places=9)
        for key in loads[False]:
            self.assertAlmostEqual(loads[False][key], loads[True][key], places=9)

    def test_time_evolving_support_transfer_releases_at_analytic_moment(self) -> None:
        system = self._two_foot_platform()
        sliding = ContactState(ContactMode.SLIDE, slip_direction=1)
        state = system.make_state(
            {"platform": RigidBodyState((0.0, 0.5), 0.0)},
            active_contacts={"left": sliding, "right": sliding},
        )
        dt, release_time, min_gap = 0.0005, None, 0.0
        for _ in range(1000):
            ramp = AppliedWrench("platform", "moment-ramp", moment_n_m=-40.0 * state.time_s)
            step = system.advance(state, dt, external_wrenches=(ramp,))
            if release_time is None and any(e.event_id == "CONTACT_RELEASE" for e in step.events):
                release_time = state.time_s
            state = step.state
            min_gap = min(min_gap, *(system.contact_kinematics(state, c).gap_m for c in ("left", "right")))
        analytic = 2.0 * 9.81 * 0.4 / 40.0
        self.assertIsNotNone(release_time)
        self.assertLessEqual(abs(release_time - analytic), dt)
        self.assertGreater(min_gap, -1e-8)
        self.assertEqual(state.contacts["left"].mode, ContactMode.OPEN)
        self.assertNotEqual(state.contacts["right"].mode, ContactMode.OPEN)

    def test_sliding_contact_captures_stick_after_slip_reverses(self) -> None:
        """Coulomb stick capture: a block that has slid to rest must stop slipping.

        Known RES-491 blocker found by this falsifier: the stepper keeps SLIDE
        and alternates slip direction every step (|v_t| ~ mu*g*dt) instead of
        entering STICK when the required tangential force is inside the cone.
        """
        system = PlanarMultibodySystem(
            (body("block", 2.0, 0.1),), (),
            (point_contact("pad", "block", (0.0, 0.0), (0.0, 0.0), friction=0.5,
                           side="LEFT", channel="FZ_LEFT"),),
            gravity_m_s2=9.81, clock_id="reviewer-stick-capture",
        )
        state = system.make_state(
            {"block": RigidBodyState((0.0, 0.0), 0.0, (1.2, 0.0))},
            active_contacts={"pad": ContactState(ContactMode.SLIDE, slip_direction=1)},
        )
        for _ in range(400):
            state = system.advance(state, 0.001).state
        self.assertEqual(state.contacts["pad"].mode, ContactMode.STICK)
        self.assertLess(abs(state.bodies["block"].velocity_m_s[0]), 1e-9)
        self.assertAlmostEqual(
            state.bodies["block"].position_m[0], 1.2**2 / (2.0 * 0.5 * 9.81), delta=0.003
        )


class VelocityLevelCoulombFalsifierTests(unittest.TestCase):
    """Owner-required stick/slip falsifiers for the velocity-level impulse step.

    Every expected value is an independent closed form (Coulomb block, incline,
    impulse-momentum balance, symmetry) and not a value read back from the solver.
    """

    MU, G, MASS = 0.5, 9.81, 2.0

    def _block(self, *, friction: float | None = None, normal: tuple[float, float] = (0.0, 1.0)) -> PlanarMultibodySystem:
        return PlanarMultibodySystem(
            (body("block", self.MASS, 0.1),), (),
            (point_contact("pad", "block", (0.0, 0.0), (0.0, 0.0),
                           friction=self.MU if friction is None else friction,
                           normal=normal, kind=ContactKind.BAR_FLOOR),),
            gravity_m_s2=self.G, clock_id="stick-slip-falsifier",
        )

    def _assert_step_invariants(self, system: PlanarMultibodySystem, step, mu: dict[str, float]) -> None:
        for contact_id, (normal, tangential) in step.contact_impulses_n_s.items():
            self.assertGreaterEqual(normal, 0.0)
            self.assertLessEqual(abs(tangential), mu[contact_id] * normal + 1e-12)
            self.assertGreater(system.contact_kinematics(step.state, contact_id).gap_m, -1e-9)
        self.assertLessEqual(step.contact_work_j, 1e-12)

    def test_slide_to_stick_capture_converges_without_slip_direction_chatter(self) -> None:
        v0 = 1.2
        exact_stop = v0 / (self.MU * self.G)
        exact_distance = v0**2 / (2.0 * self.MU * self.G)
        distance_errors = []
        for dt in (0.002, 0.001, 0.0005, 0.00025):
            with self.subTest(dt=dt):
                system = self._block()
                state = system.make_state(
                    {"block": RigidBodyState((0.0, 0.0), 0.0, (v0, 0.0))},
                    active_contacts={"pad": ContactState(ContactMode.SLIDE, slip_direction=1)},
                )
                capture_time, stick_steps, directions = None, 0, set()
                for _ in range(int(round((exact_stop + 0.15) / dt))):
                    before = system.kinetic_energy(state)
                    step = system.advance(state, dt)
                    self._assert_step_invariants(system, step, {"pad": self.MU})
                    self.assertLessEqual(system.kinetic_energy(step.state) - before, step.contact_work_j + 1e-12)
                    state = step.state
                    mode = state.contacts["pad"]
                    if mode.mode is ContactMode.SLIDE:
                        self.assertIsNone(capture_time, "contact re-entered SLIDE after capture")
                        directions.add(mode.slip_direction)
                    if any(e.event_id == "STICK_CAPTURE" for e in step.events):
                        capture_time = state.time_s
                    if capture_time is not None:
                        self.assertIs(mode.mode, ContactMode.STICK)
                        self.assertEqual(state.bodies["block"].velocity_m_s[0], 0.0)
                        stick_steps += 1
                self.assertEqual(directions, {1})
                self.assertIsNotNone(capture_time)
                self.assertLessEqual(abs(capture_time - exact_stop), dt + 1e-12)
                self.assertGreaterEqual(stick_steps, 70)
                distance_errors.append(abs(state.bodies["block"].position_m[0] - exact_distance))
        for coarse, fine in zip(distance_errors, distance_errors[1:]):
            self.assertLess(fine, coarse)
        self.assertLess(distance_errors[-1], 2e-4)

    def test_stick_persists_under_subthreshold_load_and_slips_above_capacity(self) -> None:
        dt = 0.001
        capacity = self.MU * self.MASS * self.G
        for fraction, expect_stick in ((0.0, True), (0.6, True), (0.999, True), (1.001, False), (1.5, False)):
            with self.subTest(load_fraction=fraction):
                system = self._block()
                state = system.make_state({"block": RigidBodyState((0.0, 0.0), 0.0)}, active_contacts=("pad",))
                push = AppliedWrench("block", "lateral-load", (fraction * capacity, 0.0))
                for index in range(150):
                    step = system.advance(state, dt, external_wrenches=(push,))
                    self._assert_step_invariants(system, step, {"pad": self.MU})
                    normal, tangential = step.contact_impulses_n_s["pad"]
                    self.assertAlmostEqual(normal, self.MASS * self.G * dt, places=12)
                    momentum_change = self.MASS * (step.state.bodies["block"].velocity_m_s[0] - state.bodies["block"].velocity_m_s[0])
                    self.assertAlmostEqual(momentum_change, fraction * capacity * dt + tangential, places=12)
                    state = step.state
                    if expect_stick:
                        self.assertIs(state.contacts["pad"].mode, ContactMode.STICK)
                        self.assertAlmostEqual(tangential, -fraction * capacity * dt, places=12)
                    else:
                        self.assertIs(state.contacts["pad"].mode, ContactMode.SLIDE)
                        self.assertEqual(state.contacts["pad"].slip_direction, 1)
                        self.assertAlmostEqual(tangential, -capacity * dt, places=12)
                        if index == 0:
                            self.assertTrue(any(e.event_id == "SLIP_ONSET" for e in step.events))
                if expect_stick:
                    self.assertEqual(state.bodies["block"].position_m[0], 0.0)
                else:
                    t = state.time_s
                    expected_a = (fraction - 1.0) * capacity / self.MASS
                    self.assertAlmostEqual(state.bodies["block"].velocity_m_s[0], expected_a * t, places=9)

    def test_near_zero_slip_is_not_captured_when_static_friction_is_infeasible(self) -> None:
        dt = 0.001
        capacity = self.MU * self.MASS * self.G
        for v_initial, load, expected_direction in (
            (1e-12, 1.5 * capacity, 1),     # tiny forward slip, overwhelming forward load
            (5e-4, -2.0 * capacity, -1),    # forward slip, reversing load beyond the cone
            (-5e-4, 2.0 * capacity, 1),
        ):
            with self.subTest(v=v_initial, load=load):
                system = self._block()
                state = system.make_state(
                    {"block": RigidBodyState((0.0, 0.0), 0.0, (v_initial, 0.0))},
                    active_contacts={"pad": ContactState(ContactMode.SLIDE, slip_direction=1 if v_initial > 0 else -1)},
                )
                push = AppliedWrench("block", "adversarial-load", (load, 0.0))
                for _ in range(20):
                    step = system.advance(state, dt, external_wrenches=(push,))
                    self._assert_step_invariants(system, step, {"pad": self.MU})
                    state = step.state
                    self.assertIs(state.contacts["pad"].mode, ContactMode.SLIDE)
                    self.assertEqual(state.contacts["pad"].slip_direction, expected_direction)
                    # Kinetic friction opposes the actual post-step slip.
                    _, tangential = step.contact_impulses_n_s["pad"]
                    self.assertLess(tangential * state.bodies["block"].velocity_m_s[0], 0.0)

    def test_inclined_plane_sticks_below_friction_angle_and_slides_with_analytic_acceleration(self) -> None:
        dt, duration = 0.0005, 0.4
        for angle_deg, mu in ((20.0, 0.5), (35.0, 0.5), (35.0, 0.0)):
            with self.subTest(angle=angle_deg, mu=mu):
                theta = math.radians(angle_deg)
                normal = (-math.sin(theta), math.cos(theta))
                system = self._block(friction=mu, normal=normal)
                state = system.make_state({"block": RigidBodyState((0.0, 0.0), 0.0)}, active_contacts=("pad",))
                for _ in range(int(round(duration / dt))):
                    step = system.advance(state, dt)
                    self._assert_step_invariants(system, step, {"pad": mu})
                    state = step.state
                down_slope = (-math.cos(theta), -math.sin(theta))
                position = state.bodies["block"].position_m
                travelled = position[0] * down_slope[0] + position[1] * down_slope[1]
                if math.tan(theta) < mu:
                    self.assertIs(state.contacts["pad"].mode, ContactMode.STICK)
                    self.assertAlmostEqual(travelled, 0.0, places=12)
                else:
                    accel = self.G * (math.sin(theta) - mu * math.cos(theta))
                    self.assertIs(state.contacts["pad"].mode, ContactMode.SLIDE)
                    self.assertAlmostEqual(travelled, 0.5 * accel * duration**2, delta=accel * duration * dt)

    def test_redundant_sticking_supports_are_refused_by_the_impulse_step(self) -> None:
        system = IndependentReviewerFalsifierTests._two_foot_platform(friction=0.8)
        state = system.make_state({"platform": RigidBodyState((0.0, 0.5), 0.0)}, active_contacts=("left", "right"))
        with self.assertRaisesRegex(ConstraintSolveError, "singular or redundant"):
            system.advance(state, 0.001)

    @staticmethod
    def _two_leg_rack(*, mirror: bool = False, reverse_order: bool = False) -> PlanarMultibodySystem:
        sign = -1.0 if mirror else 1.0
        definitions = (body("torso", 5.0, 1.0), body("left_leg", 1.0, 0.1), body("right_leg", 1.0, 0.1))
        joints = (
            PlanarJoint("left_hip", "torso", "left_leg", (-0.3 * sign, 0.0), (0.0, 0.5)),
            PlanarJoint("right_hip", "torso", "right_leg", (0.3 * sign, 0.0), (0.0, 0.5)),
        )
        contacts = [
            point_contact("left_foot", "left_leg", (0.0, -0.5), (-0.3 * sign, 0.0), friction=0.8,
                          side="LEFT", channel="FZ_LEFT", plate_origin=(-0.3 * sign, 0.0)),
            point_contact("right_foot", "right_leg", (0.0, -0.5), (0.3 * sign, 0.0), friction=0.8,
                          side="RIGHT", channel="FZ_RIGHT", plate_origin=(0.3 * sign, 0.0)),
            point_contact("rack", "torso", (0.0, 0.0), (0.0, 1.0), normal=(sign, 0.0), kind=ContactKind.RACK),
        ]
        if reverse_order:
            contacts.reverse()
        return PlanarMultibodySystem(definitions, joints, tuple(contacts), gravity_m_s2=9.81, clock_id="coupled-legs-rack")

    def _two_leg_state(self, system: PlanarMultibodySystem, *, mirror: bool = False):
        sign = -1.0 if mirror else 1.0
        return system.make_state(
            {
                "torso": RigidBodyState((0.0, 1.0), 0.0),
                "left_leg": RigidBodyState((-0.3 * sign, 0.5), 0.0),
                "right_leg": RigidBodyState((0.3 * sign, 0.5), 0.0),
            },
            active_contacts={
                "left_foot": ContactState(ContactMode.STICK),
                "right_foot": ContactState(ContactMode.STICK),
                "rack": ContactState(ContactMode.SLIDE, slip_direction=1),
            },
        )

    def test_coupled_feet_and_rack_share_one_impulse_solve(self) -> None:
        dt, g = 0.001, 9.81
        mu = {"left_foot": 0.8, "right_foot": 0.8, "rack": 0.0}
        system = self._two_leg_rack()
        state = self._two_leg_state(system)
        into_rack = AppliedWrench("torso", "push-into-rack", (-10.0, 0.0))
        for _ in range(120):
            step = system.advance(state, dt, external_wrenches=(into_rack,))
            self._assert_step_invariants(system, step, mu)
            state = step.state
            impulses = step.contact_impulses_n_s
            self.assertAlmostEqual(impulses["rack"][0], 10.0 * dt, places=10)
            self.assertAlmostEqual(impulses["left_foot"][0], 3.5 * g * dt, places=10)
            self.assertAlmostEqual(impulses["right_foot"][0], 3.5 * g * dt, places=10)
            self.assertAlmostEqual(impulses["left_foot"][1], 0.0, places=10)
            self.assertAlmostEqual(impulses["right_foot"][1], 0.0, places=10)
        self.assertTrue(all(state.contacts[c].mode is not ContactMode.OPEN for c in mu))
        self.assertLess(max(abs(v) for b in state.bodies.values() for v in (*b.velocity_m_s, b.angular_velocity_rad_s)), 1e-10)

        # Pulling away releases the rack; the planted feet then carry coupled,
        # symmetric tangential impulses that close linear momentum exactly.
        state = self._two_leg_state(system)
        away = AppliedWrench("torso", "pull-from-rack", (10.0, 0.0))
        first = system.advance(state, dt, external_wrenches=(away,))
        self.assertIs(first.state.contacts["rack"].mode, ContactMode.OPEN)
        self.assertTrue(any(e.event_id == "CONTACT_RELEASE" for e in first.events))
        state = first.state
        for _ in range(100):
            step = system.advance(state, dt, external_wrenches=(away,))
            self._assert_step_invariants(system, step, mu)
            impulses = step.contact_impulses_n_s
            self.assertIs(step.state.contacts["left_foot"].mode, ContactMode.STICK)
            self.assertIs(step.state.contacts["right_foot"].mode, ContactMode.STICK)
            self.assertAlmostEqual(impulses["left_foot"][1], impulses["right_foot"][1], places=12)
            # Exact impulse-momentum closure of the coupled solve (before the
            # separate mass-metric position/velocity projection).
            generalized = system._assemble_generalized_forces(state, {}, (away,))[0]
            v_plus, solved, _, _ = system._impulse_step_solve(state, generalized, dt)
            self.assertEqual(solved, impulses)
            masses = {"torso": 5.0, "left_leg": 1.0, "right_leg": 1.0}
            momentum_change = sum(
                masses[b] * (v_plus[3 * i] - state.bodies[b].velocity_m_s[0])
                for i, b in enumerate(("torso", "left_leg", "right_leg"))
            )
            horizontal_impulse = 10.0 * dt + sum(impulses[c][1] for c in ("left_foot", "right_foot"))
            self.assertAlmostEqual(momentum_change, horizontal_impulse, places=12)
            # The projection that follows only corrects O(dt^2) drift.
            projected_change = system.linear_momentum(step.state)[0] - system.linear_momentum(state)[0]
            self.assertLess(abs(projected_change - horizontal_impulse), 1e-3 * abs(horizontal_impulse))
            state = step.state
        self.assertGreater(state.bodies["torso"].velocity_m_s[0], 0.0)

    def test_coupled_support_impulses_are_label_order_and_mirror_equivariant(self) -> None:
        dt = 0.001
        results = {}
        for mirror in (False, True):
            for reverse_order in (False, True):
                system = self._two_leg_rack(mirror=mirror, reverse_order=reverse_order)
                state = self._two_leg_state(system, mirror=mirror)
                push = AppliedWrench("torso", "push", (10.0 if mirror else -10.0, -5.0), (0.1 * (-1 if mirror else 1), 0.0))
                for _ in range(30):
                    step = system.advance(state, dt, external_wrenches=(push,))
                    state = step.state
                results[(mirror, reverse_order)] = step.contact_impulses_n_s
        reference = results[(False, False)]
        for key, impulses in results.items():
            for contact_id, (normal, tangential) in reference.items():
                sign = -1.0 if key[0] else 1.0
                self.assertAlmostEqual(impulses[contact_id][0], normal, places=10, msg=str(key))
                self.assertAlmostEqual(impulses[contact_id][1], sign * tangential, places=10, msg=str(key))

    def test_grip_coupled_bar_floor_drag_uses_joint_coupled_effective_mass(self) -> None:
        dt, g, mu = 0.001, 9.81, 0.5
        bar_mass, hand_mass = 20.0, 2.0
        capacity = mu * (bar_mass + hand_mass) * g
        for fraction, expect_stick in ((0.8, True), (1.2, False)):
            with self.subTest(fraction=fraction):
                system = PlanarMultibodySystem(
                    (body("bar", bar_mass, 0.1, "LOAD_TEST_BODY"), body("hand", hand_mass, 0.02)),
                    (PlanarJoint("grip", "bar", "hand", (0.0, 0.0), (0.0, -0.3), connection_kind=ConnectionKind.GRIP),),
                    (point_contact("bar_floor", "bar", (0.0, 0.0), (0.0, 0.0), friction=mu, kind=ContactKind.BAR_FLOOR),),
                    gravity_m_s2=g, clock_id="grip-drag",
                )
                state = system.make_state(
                    {"bar": RigidBodyState((0.0, 0.0), 0.0), "hand": RigidBodyState((0.0, 0.3), 0.0)},
                    active_contacts=("bar_floor",),
                )
                pull = AppliedWrench("hand", "horizontal-pull", (fraction * capacity, 0.0), (0.0, -0.3))
                for _ in range(100):
                    step = system.advance(state, dt, external_wrenches=(pull,))
                    self._assert_step_invariants(system, step, {"bar_floor": mu})
                    # The revolute hand is an inverted pendulum that begins to pivot once
                    # the bar accelerates, so the normal impulse is checked by exact
                    # impulse-momentum closure of the coupled solve, not a static weight.
                    generalized = system._assemble_generalized_forces(state, {}, (pull,))[0]
                    v_plus, solved, _, _ = system._impulse_step_solve(state, generalized, dt)
                    normal, tangential = solved["bar_floor"]
                    for axis, external in ((0, fraction * capacity * dt + tangential),
                                           (1, normal - (bar_mass + hand_mass) * g * dt)):
                        change = sum(
                            m * (v_plus[3 * i + axis] - state.bodies[b].velocity_m_s[axis])
                            for i, (b, m) in enumerate((("bar", bar_mass), ("hand", hand_mass)))
                        )
                        self.assertAlmostEqual(change, external, places=12)
                    if not expect_stick:
                        self.assertAlmostEqual(tangential, -mu * normal, places=14)
                    state = step.state
                if expect_stick:
                    self.assertIs(state.contacts["bar_floor"].mode, ContactMode.STICK)
                    self.assertAlmostEqual(state.bodies["bar"].position_m[0], 0.0, places=12)
                else:
                    self.assertIs(state.contacts["bar_floor"].mode, ContactMode.SLIDE)
                    self.assertEqual(state.contacts["bar_floor"].slip_direction, 1)
                    # Bounded by the frictionless and rigid-composite Coulomb limits.
                    self.assertGreater(state.bodies["bar"].velocity_m_s[0], 0.0)
                    self.assertLess(
                        system.linear_momentum(state)[0],
                        fraction * capacity * state.time_s,
                    )

    def test_rotating_body_frame_contact_sticks_then_slips_with_mirror_symmetry(self) -> None:
        dt, mu = 0.0005, 0.3
        outcomes = {}
        for sign in (1.0, -1.0):
            pin = ContactSpec("pin", "platform", (0.0, 0.0), (0.0, 1.0), 2.0, 0.0,
                              ContactKind.RACK, surface_point_m=(0.0, 0.0))
            seat = ContactSpec("seat", "block", (0.0, 0.0), (0.0, 1.0), mu, 0.0, ContactKind.CATCH,
                               body_b_id="platform", local_point_b_m=(0.3 * sign, 0.0),
                               normal_frame=ContactNormalFrame.BODY_B)
            system = PlanarMultibodySystem(
                (body("platform", 5.0, 0.5), body("block", 1.0, 0.01)), (), (pin, seat),
                gravity_m_s2=9.81, clock_id="rotating-seat",
            )
            omega = -0.8 * sign
            state = system.make_state(
                {
                    "platform": RigidBodyState((0.0, 0.0), 0.0, (0.0, 0.0), omega),
                    "block": RigidBodyState((0.3 * sign, 0.0), 0.0, (0.0, omega * 0.3 * sign), omega),
                },
                active_contacts=("pin", "seat"),
            )
            energy = system.kinetic_energy(state) + system.potential_energy(state)
            onset = None
            for _ in range(1200):
                step = system.advance(state, dt)
                self._assert_step_invariants(system, step, {"pin": 2.0, "seat": mu})
                state = step.state
                new_energy = system.kinetic_energy(state) + system.potential_energy(state)
                self.assertLessEqual(new_energy, energy + 1e-9)
                energy = new_energy
                seat_kinematics = system.contact_kinematics(state, "seat")
                if onset is None:
                    if any(e.event_id == "SLIP_ONSET" for e in step.events):
                        onset = (state.time_s, state.bodies["platform"].angle_rad)
                        self.assertEqual(state.contacts["seat"].slip_direction, int(sign))
                    else:
                        self.assertIs(state.contacts["seat"].mode, ContactMode.STICK)
                        self.assertLess(abs(seat_kinematics.tangential_velocity_m_s), 1e-9)
                if state.contacts["seat"].mode is ContactMode.OPEN:
                    break
            self.assertIsNotNone(onset)
            outcomes[sign] = onset
        self.assertAlmostEqual(outcomes[1.0][0], outcomes[-1.0][0], places=12)
        self.assertAlmostEqual(outcomes[1.0][1], -outcomes[-1.0][1], places=9)
        # Static tilt bound: slip cannot start before the quasi-static friction angle
        # is approached; dynamic unloading (platform angular acceleration) only advances it.
        self.assertLess(abs(outcomes[1.0][1]), math.atan(mu) + 1e-6)


if __name__ == "__main__":
    unittest.main()
