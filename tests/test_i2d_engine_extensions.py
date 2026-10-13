"""RES-492 extensions to the RES-491 planar multibody engine.

Each change is checked against an independent formulation: the Schur
complement against the full dense saddle-point matrix.
"""

from __future__ import annotations

import random
import unittest

from biosyngen.i1.linalg import solve
from biosyngen.multibody import (
    ContactKind,
    ContactMode,
    ContactState,
    PlanarJoint,
    PlanarMultibodySystem,
    RigidBodyState,
)
from tests.test_i2c_multibody import body, point_contact


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


if __name__ == "__main__":
    unittest.main()
