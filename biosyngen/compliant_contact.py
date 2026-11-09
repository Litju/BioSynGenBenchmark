"""Finite Hunt–Crossley point contact, SI units, uncalibrated material law.

F = k d**n max(0, 1 + a d_dot); U = k d**(n+1)/(n+1).
The nonadhesive clamp preserves nonnegative dissipation even during forced
separation. No rigid impulse is converted into a force peak.
Reference: https://simbody.github.io/simbody-3.6-doxygen/api/classSimTK_1_1HuntCrossleyForce.html
Here a includes the reference's 3/2 coefficient. Friction is a regularized
Coulomb law; it permits creep and does not assert exact static sticking.

Two declared step schemes exist (RES-609):

* EXPLICIT_INTERVAL_START_V1 (historical default): the response at interval
  start is held over the step. Its regularized friction has an effective
  viscosity mu*N/epsilon that can exceed the explicit stability bound, so the
  contact can do net positive work. Retained as RED evidence.
* IMPLICIT_DISCRETE_GRADIENT_V2: the applied forces are the fixed point of the
  law evaluated at the interval-end velocity, with the elastic part replaced
  by the discrete gradient -(U(g1)-U(g0))/(g1-g0) over the actual gap change.
  Elastic work then equals -dU, normal damping and friction dissipate at the
  end velocity. The fixed point is solved by Newton on a finite-difference
  linearization of one full articulated step, then checked against a true
  step; nonconvergence raises ImplicitContactNotConverged (never accepted).
"""
from dataclasses import dataclass, replace
import math

from .multibody import (AppliedWrench, ConstraintSolveError, ContactMode, ContactState, DynamicsError,
                        PlanarMultibodySystem)


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise DynamicsError(f"{name} must be finite numeric data")
    return value


EXPLICIT_INTERVAL_START_V1 = "EXPLICIT_INTERVAL_START_V1"
IMPLICIT_DISCRETE_GRADIENT_V2 = "IMPLICIT_DISCRETE_GRADIENT_V2"
COMPLIANT_SCHEMES = (EXPLICIT_INTERVAL_START_V1, IMPLICIT_DISCRETE_GRADIENT_V2)


class ImplicitContactNotConverged(ConstraintSolveError):
    """The interval-end contact fixed point was not reached within budget."""


@dataclass(frozen=True, slots=True)
class CompliantResponse:
    normal_n: float
    tangent_n: float
    stored_energy_j: float
    dissipated_power_w: float
    # V2 only: exact discrete channels over the interval (None under V1).
    scheme: str = EXPLICIT_INTERVAL_START_V1
    stored_energy_end_j: float | None = None
    dissipated_energy_j: float | None = None
    contact_work_j: float | None = None
    fixed_point_residual_n: float | None = None
    iterations: int = 0


@dataclass(frozen=True, slots=True)
class HuntCrossleyContact:
    stiffness: float = 500_000.0
    exponent: float = 1.5
    damping_s_m: float = 0.5
    friction: float = 0.8
    slip_transition_m_s: float = 0.01

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            finite(getattr(self, name), name)
        if self.stiffness <= 0 or self.exponent < 1 or self.damping_s_m < 0 or self.friction < 0 or self.slip_transition_m_s <= 0:
            raise DynamicsError("inadmissible compliant material parameters")

    def evaluate(self, gap_m, normal_velocity_m_s, tangent_velocity_m_s=0.0):
        for value, name in ((gap_m, "gap"), (normal_velocity_m_s, "normal velocity"),
                            (tangent_velocity_m_s, "tangential velocity")):
            finite(value, name)
        depth = max(0.0, -gap_m)
        elastic = self.stiffness * depth ** self.exponent
        normal = elastic * max(0.0, 1.0 - self.damping_s_m * normal_velocity_m_s)
        tangent = -self.friction * normal * math.tanh(tangent_velocity_m_s / self.slip_transition_m_s)
        stored = self.stiffness * depth ** (self.exponent + 1) / (self.exponent + 1)
        dissipated = -(normal - elastic) * normal_velocity_m_s - tangent * tangent_velocity_m_s
        for value in (normal, tangent, stored, dissipated):
            finite(value, "computed contact response")
        return CompliantResponse(normal, tangent, stored, dissipated)

    def potential(self, gap_m):
        depth = max(0.0, -gap_m)
        return self.stiffness * depth ** (self.exponent + 1) / (self.exponent + 1)

    def implicit_force(self, gap0_m, gap1_m, normal_velocity1_m_s, tangent_velocity1_m_s):
        """(normal, tangent) at interval end with a discrete-gradient elastic part."""
        if abs(gap1_m - gap0_m) > 1e-13:
            elastic = (self.potential(gap0_m) - self.potential(gap1_m)) / (gap1_m - gap0_m)
        else:
            elastic = self.stiffness * max(0.0, -0.5 * (gap0_m + gap1_m)) ** self.exponent
        elastic = max(0.0, elastic)
        normal = elastic * max(0.0, 1.0 - self.damping_s_m * normal_velocity1_m_s)
        tangent = -self.friction * normal * math.tanh(tangent_velocity1_m_s / self.slip_transition_m_s)
        return normal, tangent


class CompliantPlanarStepper:
    """Replace selected world contacts with finite forces, preserving rigid others.

    Responses are evaluated at interval start (explicit first-order stepping).
    Finite compliance permits material deformation below the nominal surface.
    Refinement is mandatory; this adapter supplies no automatic stability claim.
    """
    def __init__(self, system, laws, *, release_times=None, scheme=EXPLICIT_INTERVAL_START_V1,
                 fixed_point_tolerance=1e-7, max_corrections=8):
        if scheme not in COMPLIANT_SCHEMES:
            raise DynamicsError(f"unknown compliant step scheme {scheme!r}")
        self.scheme = scheme
        self.fixed_point_tolerance = finite(fixed_point_tolerance, "fixed-point tolerance")
        self.max_corrections = max_corrections
        self.counters = {"implicit_steps": 0, "free_steps": 0, "step_evaluations": 0,
                         "max_relative_fixed_point_residual": 0.0, "max_corrections_used": 0}
        self.geometry = system
        self.laws = dict(laws)
        self.release_times = dict(release_times or {})
        rigid_ids = {s.contact_id for s in system.contacts if s.contact_id not in laws}
        if not set(self.release_times) <= rigid_ids:
            raise DynamicsError("release times must identify rigid contacts")
        for time in self.release_times.values():
            if finite(time, "support release time") < 0:
                raise DynamicsError("support release time must be nonnegative")
        self._removed = frozenset()
        self.specs = {s.contact_id: s for s in system.contacts if s.contact_id in laws}
        if not self.laws or set(self.specs) != set(self.laws):
            raise DynamicsError("compliant contacts must identify existing surfaces")
        if any(s.body_b_id is not None for s in self.specs.values()):
            raise DynamicsError("compliant body-body contact is not implemented")
        if any(not isinstance(law, HuntCrossleyContact) for law in self.laws.values()):
            raise DynamicsError("explicit compliant constitutive law required")
        self.dynamics = PlanarMultibodySystem(
            system.bodies, system.joints,
            [s for s in system.contacts if s.contact_id not in laws],
            gravity_m_s2=system.gravity_m_s2, clock_id=system.clock_id,
            actuators=system.actuators, linear_solver=system._linear_solver,
            linear_solver_id=system.linear_solver_id, projection_policy=system.projection_policy)

    def advance(self, state, dt_s, *, excitations=None, external_wrenches=()):
        removed = frozenset(cid for cid, time in self.release_times.items() if state.time_s >= time)
        if any(state.time_s < time < state.time_s + dt_s for time in self.release_times.values()):
            raise DynamicsError("integration interval must end at support release")
        if removed != self._removed:
            self.dynamics = PlanarMultibodySystem(
                self.geometry.bodies, self.geometry.joints,
                [s for s in self.geometry.contacts if s.contact_id not in self.laws and s.contact_id not in removed],
                gravity_m_s2=self.geometry.gravity_m_s2, clock_id=self.geometry.clock_id,
                actuators=self.geometry.actuators, linear_solver=self.geometry._linear_solver,
                linear_solver_id=self.geometry.linear_solver_id, projection_policy=self.geometry.projection_policy)
            self._removed = removed
        if self.scheme == IMPLICIT_DISCRETE_GRADIENT_V2:
            return self._advance_implicit(state, dt_s, removed, excitations, external_wrenches)
        wrenches = list(external_wrenches)
        responses = {}
        for cid, spec in self.specs.items():
            kin = self.geometry.contact_kinematics(state, cid)
            response = self.laws[cid].evaluate(kin.gap_m, kin.normal_velocity_m_s,
                                                kin.tangential_velocity_m_s)
            responses[cid] = response
            force = tuple(response.normal_n*n + response.tangent_n*t
                          for n, t in zip(kin.normal_world, kin.tangent_world))
            wrenches.append(AppliedWrench(spec.body_a_id, cid, force, spec.local_point_a_m))
        rigid_state = replace(state, contacts={k: v for k, v in state.contacts.items()
                                             if k not in self.laws and k not in removed})
        step = self.dynamics.advance(rigid_state, dt_s, excitations=excitations,
                                     external_wrenches=wrenches)
        return self._label(step, removed), responses

    def _label(self, step, removed):
        contacts = dict(step.state.contacts)
        contacts.update({cid: ContactState() for cid in removed})
        contacts.update({cid: ContactState() for cid in self.laws})
        geometric_state = replace(step.state, contacts=contacts)
        for cid in self.laws:
            kin = self.geometry.contact_kinematics(geometric_state, cid)
            contacts[cid] = (ContactState(ContactMode.SLIDE, slip_direction=1 if kin.tangential_velocity_m_s >= 0 else -1)
                             if kin.gap_m < 0 else ContactState())
        return replace(step, state=replace(step.state, contacts=contacts))

    # -- IMPLICIT_DISCRETE_GRADIENT_V2 ---------------------------------------
    def _trial(self, args, forces, frames):
        state, dt_s, removed, excitations, external_wrenches = args
        wrenches = list(external_wrenches)
        for cid, (normal, tangent) in forces.items():
            n, t = frames[cid]
            wrenches.append(AppliedWrench(self.specs[cid].body_a_id, cid,
                                          (normal*n[0] + tangent*t[0], normal*n[1] + tangent*t[1]),
                                          self.specs[cid].local_point_a_m))
        rigid_state = replace(state, contacts={k: v for k, v in state.contacts.items()
                                             if k not in self.laws and k not in removed})
        step = self.dynamics.advance(rigid_state, dt_s, excitations=excitations, external_wrenches=wrenches)
        self.counters["step_evaluations"] += 1
        labelled = self._label(step, removed)
        if labelled.elapsed_s != dt_s:
            raise ImplicitContactNotConverged("rigid event inside an implicit compliant interval")
        return labelled, {cid: self.geometry.contact_kinematics(labelled.state, cid) for cid in self.specs}

    def _advance_implicit(self, state, dt_s, removed, excitations, external_wrenches):
        args = (state, dt_s, removed, excitations, external_wrenches)
        start = {cid: self.geometry.contact_kinematics(state, cid) for cid in self.specs}
        frames = {cid: (k.normal_world, k.tangent_world) for cid, k in start.items()}
        explicit = {}
        for cid, kin in start.items():
            r = self.laws[cid].evaluate(kin.gap_m, kin.normal_velocity_m_s, kin.tangential_velocity_m_s)
            explicit[cid] = (r.normal_n, r.tangent_n)
        step, end = self._trial(args, explicit, frames)
        active = sorted(cid for cid in self.specs if start[cid].gap_m < 0 or end[cid].gap_m < 0)
        if not active:
            self.counters["free_steps"] += 1
            return step, {cid: self._response(cid, start[cid], end[cid], (0.0, 0.0), dt_s, 0.0, 0)
                          for cid in self.specs}

        def outputs(kin):
            return [v for cid in active for v in (kin[cid].gap_m, kin[cid].normal_velocity_m_s,
                                                  kin[cid].tangential_velocity_m_s)]

        def law(y):
            out = []
            for i, cid in enumerate(active):
                out.extend(self.laws[cid].implicit_force(start[cid].gap_m, y[3*i], y[3*i+1], y[3*i+2]))
            return out

        def forces_of(x):
            return {cid: (x[2*i], x[2*i+1]) for i, cid in enumerate(active)}

        x0 = [v for cid in active for v in explicit[cid]]
        y0 = outputs(end)
        size = len(x0)
        # Within one step the articulated response is affine in applied force
        # (fixed rigid modes), so forward differences give its exact slope.
        columns = []
        for j in range(size):
            probe = list(x0)
            delta = max(1.0, 1e-3 * abs(probe[j]))
            probe[j] += delta
            _, kin = self._trial(args, forces_of(probe), frames)
            columns.append([(a - b) / delta for a, b in zip(outputs(kin), y0)])
        matrix = [[columns[j][i] for j in range(size)] for i in range(3 * len(active))]
        relative = math.inf
        for correction in range(1, self.max_corrections + 1):
            x = _newton_fixed_point(x0, y0, matrix, law)
            step, end = self._trial(args, forces_of(x), frames)
            y = outputs(end)
            phi = law(y)
            scale = max(1.0, max(abs(v) for v in phi))
            relative = max(abs(a - b) for a, b in zip(x, phi)) / scale
            if relative <= self.fixed_point_tolerance:
                break
            # Broyden rank-one update keeps the linearization consistent with
            # the newest true step (active-set change inside the interval).
            dx = [a - b for a, b in zip(x, x0)]
            dy = [a - b for a, b in zip(y, y0)]
            norm = sum(v * v for v in dx)
            if norm > 0.0:
                predicted = [sum(row[j] * dx[j] for j in range(size)) for row in matrix]
                for i, row in enumerate(matrix):
                    for j in range(size):
                        row[j] += (dy[i] - predicted[i]) * dx[j] / norm
            x0, y0 = x, y
        else:
            raise ImplicitContactNotConverged(
                f"implicit compliant fixed point relative residual {relative:.3e} after "
                f"{self.max_corrections} corrections at t={state.time_s:.6f} s")
        counters = self.counters
        counters["implicit_steps"] += 1
        counters["max_relative_fixed_point_residual"] = max(counters["max_relative_fixed_point_residual"], relative)
        counters["max_corrections_used"] = max(counters["max_corrections_used"], correction)
        solved = forces_of(x)
        return step, {cid: self._response(cid, start[cid], end[cid], solved.get(cid, (0.0, 0.0)), dt_s,
                                          relative, correction) for cid in self.specs}

    def _response(self, cid, k0, k1, force, dt_s, residual, iterations):
        """Exact interval channels: work = F . dp, dissipation = -work - dU."""
        law = self.laws[cid]
        u0, u1 = law.potential(k0.gap_m), law.potential(k1.gap_m)
        normal, tangent = force
        n, t = k0.normal_world, k0.tangent_world
        dp = (k1.point_a_world_m[0] - k0.point_a_world_m[0], k1.point_a_world_m[1] - k0.point_a_world_m[1])
        work = (normal*n[0] + tangent*t[0]) * dp[0] + (normal*n[1] + tangent*t[1]) * dp[1]
        dissipated = -work - (u1 - u0)
        return CompliantResponse(normal, tangent, u0, dissipated / dt_s, IMPLICIT_DISCRETE_GRADIENT_V2,
                                 u1, dissipated, work, residual, iterations)


def _solve_dense(matrix, rhs):
    n = len(rhs)
    a = [list(row) + [value] for row, value in zip(matrix, rhs)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-300:
            raise ImplicitContactNotConverged("singular implicit contact Newton system")
        a[col], a[pivot] = a[pivot], a[col]
        for r in range(col + 1, n):
            f = a[r][col] / a[col][col]
            for c in range(col, n + 1):
                a[r][c] -= f * a[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (a[r][n] - sum(a[r][c] * x[c] for c in range(r + 1, n))) / a[r][r]
    return x


def _newton_fixed_point(x0, y0, matrix, law, iterations=60):
    """Solve x = law(y0 + A (x - x0)) by damped Newton with a finite-difference Jacobian."""
    size = len(x0)

    def g(x):
        y = [y0[i] + sum(matrix[i][j] * (x[j] - x0[j]) for j in range(size)) for i in range(len(y0))]
        return [a - b for a, b in zip(x, law(y))]

    x = list(x0)
    gx = g(x)
    for _ in range(iterations):
        norm = max(abs(v) for v in gx)
        if norm <= 1e-11 * max(1.0, max(abs(v) for v in x)):
            break
        columns = []
        for j in range(size):
            h = max(1e-7, 1e-8 * abs(x[j]))
            probe = list(x)
            probe[j] += h
            columns.append([(a - b) / h for a, b in zip(g(probe), gx)])
        jacobian = [[columns[j][i] for j in range(size)] for i in range(size)]
        step = _solve_dense(jacobian, [-v for v in gx])
        scale = 1.0
        for _ in range(40):
            trial = [a + scale * b for a, b in zip(x, step)]
            gt = g(trial)
            if max(abs(v) for v in gt) < norm:
                break
            scale *= .5
        x, gx = trial, gt
    return x
