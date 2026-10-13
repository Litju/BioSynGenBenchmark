"""RES-482 integrator cross-check against an independent solver (SciPy DOP853).

This verifies the hand-written Dormand-Prince stepping, step control and
breakpoint handling. It does not verify the equations of motion: those come
from the same derivative function here and are checked by the invariant
tests. It requires SciPy and is skipped (with a reason) when SciPy is
unavailable, e.g. on the canonical standard-library-only interpreter. Run:

    py -3.11 -m unittest tests.test_i1_solver_crosscheck -v
"""

from __future__ import annotations

import unittest

try:
    from scipy.integrate import solve_ivp
except ImportError:  # pragma: no cover - environment dependent
    solve_ivp = None

from biosyngen.i1.scenarios import reference_cmj
from biosyngen.i1.simulate import ContactMode, ContactState, _Engine, simulate_jump


@unittest.skipIf(solve_ivp is None, "SciPy not installed in this interpreter")
class ScipyCrossCheck(unittest.TestCase):
    def _compare(self, mode: ContactMode, t0: float, t1: float) -> float:
        scenario = reference_cmj()
        traj = simulate_jump(scenario)
        eng = _Engine(scenario)
        start = traj.sample_at_or_after(t0)
        end = traj.sample_at_or_after(t1)
        self.assertEqual(start.mode, mode)
        self.assertEqual(end.mode, mode)
        n = eng.n
        y0 = list(start.q) + list(start.qd) + list(start.activation) + [0.0, 0.0, 0.0]
        cs = ContactState(mode, 0, start.heel_m[0] if mode is ContactMode.FLAT else 0.0)
        sol = solve_ivp(
            lambda t, y: eng.derivative(t, list(y), cs),
            (start.time_s, end.time_s),
            y0,
            method="DOP853",
            rtol=1e-11,
            atol=1e-12,
        )
        self.assertTrue(sol.success)
        y_ref = sol.y[:, -1]
        worst = max(abs(y_ref[i] - v) for i, v in enumerate(list(end.q) + list(end.qd)))
        self.assertLess(worst, 1e-5, msg=f"max state difference {worst:.3e}")
        return worst

    def test_unweighting_segment_matches_dop853(self) -> None:
        # Between control breakpoints (0.30 s and 0.64 s), flat-foot contact.
        self._compare(ContactMode.FLAT, 0.35, 0.60)

    def test_flight_segment_matches_dop853(self) -> None:
        traj = simulate_jump(reference_cmj())
        takeoff = traj.event_time("TAKEOFF")
        self._compare(ContactMode.FLIGHT, takeoff + 0.01, takeoff + 0.30)


if __name__ == "__main__":
    unittest.main()
