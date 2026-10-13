"""Hybrid forward dynamics of the actuated planar jumper with unilateral contact.

Contact is rigid, unilateral and Coulomb-frictional. In each contact state the
constrained equations

    [ M    -C^T ] [ q''    ]   [ G - h + Q_joint + Q_coupling            ]
    [ J     0   ] [ lambda ] = [ -J' q' - 2 b J q' - b^2 Phi (Baumgarte) ]

give the accelerations and the contact force multipliers lambda. J holds the
constrained point directions. C holds the directions in which each multiplier
acts: C = J while sticking, and C = J_z + sigma mu J_x for a sliding normal row,
so friction mu*Fz opposes the slip.

Contact states (mode, slide sign sigma):

* FLAT_FOOT stick: heel x, heel z, toe z constrained.  FLAT_FOOT slide: heel z, toe z.
* TOE_ONLY stick: toe x, toe z.                        TOE_ONLY slide: toe z.
* FLIGHT: no constraint.

Transitions are roots of physical event functions, never scheduled times:

* heel Fz -> 0: heel off.
* toe Fz -> 0: takeoff.
* |Ft| -> mu Fz: slip onset.
* slip velocity -> 0: re-stick.
* heel height -> 0 in toe contact: perfectly inelastic heel strike, applied as
  the exact rigid impulse that brings the heel to rest. The kinetic energy lost
  is booked as contact work. Impact speeds above 0.5 m/s are refused as
  unqualified.
* foot reaching the ground in flight ends the run. Landing is not modeled.

Releasing a constraint needs no impulse, so q' is continuous across releases.
Integration uses adaptive Dormand-Prince 5(4), clipped at control breakpoints,
with event roots bisected to the configured time tolerance.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math

from .actuation import activation_derivative, activations_for_torques, joint_torques
from .body import PF, PH, PS, PT, X, XB, Z, ZB, PlanarBody
from .linalg import solve
from .params import BarParameters, BodyParameters, NeuromuscularParameters


class ContactMode(str, Enum):
    FLAT = "FLAT_FOOT"
    TOE = "TOE_ONLY"
    FLIGHT = "FLIGHT"


class InadmissibleScenario(ValueError):
    """The declared world cannot start in a physically admissible state."""


MAX_QUALIFIED_IMPACT_SPEED_M_S = 0.5


@dataclass(frozen=True, slots=True)
class ContactState:
    mode: ContactMode
    slide: int = 0  # 0 stick; +1/-1 = sign of the friction force while sliding
    anchor_x_m: float = 0.0

    @property
    def label(self) -> str:
        if self.mode is ContactMode.FLIGHT:
            return "FLIGHT"
        return f"{self.mode.value}_{'STICK' if self.slide == 0 else 'SLIDE'}"


@dataclass(frozen=True, slots=True)
class Posture:
    """Absolute segment angles (rad, CCW from +x). The foot starts flat (0)."""

    shank_rad: float
    thigh_rad: float
    hat_rad: float
    ankle_x_m: float = 0.0


@dataclass(frozen=True, slots=True)
class SolverSettings:
    rtol: float = 1e-8
    atol: float = 1e-10
    max_step_s: float = 1e-3
    event_time_tolerance_s: float = 1e-10
    baumgarte_rate_s: float = 40.0
    horizon_s: float = 2.0
    collapse_com_height_m: float = 0.35
    max_contact_events: int = 200
    max_rhs_evaluations: int = 600_000


@dataclass(frozen=True, slots=True)
class JumpScenario:
    name: str
    posture: Posture
    strategy: object
    body: BodyParameters = field(default_factory=BodyParameters)
    nm: NeuromuscularParameters = field(default_factory=NeuromuscularParameters)
    bar: BarParameters | None = None
    antagonist_activation: float = 0.02
    settings: SolverSettings = field(default_factory=SolverSettings)


@dataclass(frozen=True, slots=True)
class StaticSolution:
    q: tuple[float, ...]
    activations: tuple[float, ...]
    joint_torques_n_m: tuple[float, float, float]
    heel_fx_n: float
    heel_fz_n: float
    toe_fz_n: float
    cop_x_m: float
    body_com_x_m: float


@dataclass(frozen=True, slots=True)
class JumpSample:
    time_s: float
    mode: ContactMode
    contact_state: str
    q: tuple[float, ...]
    qd: tuple[float, ...]
    qdd: tuple[float, ...]
    excitation: tuple[float, ...]
    activation: tuple[float, ...]
    heel_force_n: tuple[float, float]
    toe_force_n: tuple[float, float]
    grf_x_n: float
    grf_z_n: float
    cop_x_m: float | None
    joint_angle_rad: tuple[float, float, float]
    joint_velocity_rad_s: tuple[float, float, float]
    joint_torque_n_m: tuple[float, float, float]
    actuator_torque_n_m: tuple[float, ...]
    passive_torque_n_m: tuple[float, float, float]
    body_com_m: tuple[float, float]
    body_com_velocity_m_s: tuple[float, float]
    system_com_m: tuple[float, float]
    system_com_velocity_m_s: tuple[float, float]
    bar_m: tuple[float, float] | None
    bar_velocity_m_s: tuple[float, float] | None
    coupling_force_on_bar_n: tuple[float, float]
    heel_m: tuple[float, float]
    toe_m: tuple[float, float]
    kinetic_energy_j: float
    potential_energy_j: float
    work_joint_j: float
    work_coupling_damping_j: float
    work_contact_j: float
    constraint_residual_m: float


@dataclass(frozen=True, slots=True)
class JumpEvent:
    time_s: float
    kind: str
    detail: str = ""
    value: float | None = None


@dataclass(frozen=True, slots=True)
class JumpTrajectory:
    scenario: JumpScenario
    static: StaticSolution
    samples: tuple[JumpSample, ...]
    events: tuple[JumpEvent, ...]
    termination: str
    rhs_evaluations: int

    def event_time(self, kind: str) -> float | None:
        for event in self.events:
            if event.kind == kind:
                return event.time_s
        return None

    def sample_at_or_after(self, time_s: float) -> JumpSample:
        for sample in self.samples:
            if sample.time_s >= time_s:
                return sample
        return self.samples[-1]


# Dormand-Prince 5(4) tableau ------------------------------------------------------
_C = (0.0, 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1.0, 1.0)
_A = (
    (),
    (1 / 5,),
    (3 / 40, 9 / 40),
    (44 / 45, -56 / 15, 32 / 9),
    (19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729),
    (9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656),
    (35 / 384, 0.0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84),
)
_B5 = _A[6] + (0.0,)
_B4 = (5179 / 57600, 0.0, 7571 / 16695, 393 / 640, -92097 / 339200, 187 / 2100, 1 / 40)
_E = tuple(b5 - b4 for b5, b4 in zip(_B5, _B4))


def static_equilibrium(
    body: PlanarBody,
    nm: NeuromuscularParameters,
    posture: Posture,
    antagonist_activation: float,
) -> StaticSolution:
    """Solve the flat-foot static posture: joint torques and ground forces.

    Unknowns are the three net joint torques and the three contact forces
    (heel Fx, heel Fz, toe Fz). The six generalized equilibrium equations of
    the body are solved exactly. A bar hangs from the shoulder with the static
    spring sag that carries its weight. The posture is refused if a normal
    force is tensile, i.e. the static COP falls outside the heel-toe support.
    """
    p = body.params
    n = body.n
    q = [0.0] * n
    q[X] = posture.ankle_x_m
    q[Z] = p.ankle_height_m
    q[PF] = 0.0
    q[PS], q[PT], q[PH] = posture.shank_rad, posture.thigh_rad, posture.hat_rad
    if body.bar is not None:
        sx, sz = body.point(q, body.points["shoulder"])
        q[XB] = sx
        q[ZB] = sz - body.bar.mass_kg * p.gravity_m_s2 / body.bar.coupling_stiffness_n_m
    qd = [0.0] * n
    _, f = body.dynamics_terms(q, qd)
    coupling = body.coupling_generalized_force(q, qd)
    hjx, hjz, _, _ = body.point_jacobian(q, qd, body.points["heel"])
    _, tjz, _, _ = body.point_jacobian(q, qd, body.points["toe"])
    columns = [
        body.joint_generalized_force((1.0, 0.0, 0.0)),
        body.joint_generalized_force((0.0, 1.0, 0.0)),
        body.joint_generalized_force((0.0, 0.0, 1.0)),
        [hjx.get(i, 0.0) for i in range(n)],
        [hjz.get(i, 0.0) for i in range(n)],
        [tjz.get(i, 0.0) for i in range(n)],
    ]
    # Rows 0..5 are the body coordinates; bar rows are satisfied by the sag.
    matrix = [[columns[c][r] for c in range(6)] for r in range(6)]
    rhs = [-(f[r] + coupling[r]) for r in range(6)]
    tau_a, tau_k, tau_h, hfx, hfz, tfz = solve(matrix, rhs)
    heel_x = body.point(q, body.points["heel"])[0]
    toe_x = body.point(q, body.points["toe"])[0]
    if hfz <= 0.0 or tfz <= 0.0:
        raise InadmissibleScenario(
            "static posture needs a tensile heel or toe force (COP outside the foot): "
            f"heel Fz={hfz:.3f} N, toe Fz={tfz:.3f} N"
        )
    angles = body.joint_angles(q)
    try:
        activations = activations_for_torques(nm, (tau_a, tau_k, tau_h), angles, antagonist_activation)
    except ValueError as error:
        raise InadmissibleScenario(str(error)) from error
    cop = (hfz * heel_x + tfz * toe_x) / (hfz + tfz)
    return StaticSolution(
        q=tuple(q),
        activations=tuple(activations),
        joint_torques_n_m=(tau_a, tau_k, tau_h),
        heel_fx_n=hfx,
        heel_fz_n=hfz,
        toe_fz_n=tfz,
        cop_x_m=cop,
        body_com_x_m=body.body_com(q)[0],
    )


class _Engine:
    def __init__(self, scenario: JumpScenario) -> None:
        self.scenario = scenario
        self.body = PlanarBody(scenario.body, scenario.bar)
        self.nm = scenario.nm
        self.n = self.body.n
        self.static = static_equilibrium(
            self.body, self.nm, scenario.posture, scenario.antagonist_activation
        )
        self.quiet = list(self.static.activations)
        self.u_min = self.nm.minimum_excitation
        if min(self.quiet) < self.u_min:
            raise InadmissibleScenario("quiet-stance activation below the minimum excitation")
        self.settings = scenario.settings
        self.mu = scenario.body.friction_coefficient
        self.evaluations = 0
        self.heel_terms = self.body.points["heel"]
        self.toe_terms = self.body.points["toe"]

    def initial_state(self) -> list[float]:
        n = self.n
        y = [0.0] * (2 * n + 9)
        y[:n] = list(self.static.q)
        y[2 * n : 2 * n + 6] = list(self.quiet)
        return y

    # Contact rows ------------------------------------------------------------------
    def _contact_rows(self, q, qd, cs: ContactState):
        """Return rows (constraint J, force column C, bias, phi, phidot, tag)."""
        if cs.mode is ContactMode.FLIGHT:
            return []
        body = self.body
        heel = body.point(q, self.heel_terms)
        toe = body.point(q, self.toe_terms)
        hjx, hjz, hbx, hbz = body.point_jacobian(q, qd, self.heel_terms)
        tjx, tjz, tbx, tbz = body.point_jacobian(q, qd, self.toe_terms)
        hv = body.point_velocity(q, qd, self.heel_terms)
        tv = body.point_velocity(q, qd, self.toe_terms)

        def slid(jz, jx):
            c = dict(jz)
            for k, v in jx.items():
                c[k] = c.get(k, 0.0) + cs.slide * self.mu * v
            return c

        if cs.mode is ContactMode.FLAT:
            if cs.slide == 0:
                return [
                    (hjx, hjx, hbx, heel[0] - cs.anchor_x_m, hv[0], "heel_x"),
                    (hjz, hjz, hbz, heel[1], hv[1], "heel_z"),
                    (tjz, tjz, tbz, toe[1], tv[1], "toe_z"),
                ]
            return [
                (hjz, slid(hjz, hjx), hbz, heel[1], hv[1], "heel_z"),
                (tjz, slid(tjz, tjx), tbz, toe[1], tv[1], "toe_z"),
            ]
        if cs.slide == 0:
            return [
                (tjx, tjx, tbx, toe[0] - cs.anchor_x_m, tv[0], "toe_x"),
                (tjz, tjz, tbz, toe[1], tv[1], "toe_z"),
            ]
        return [(tjz, slid(tjz, tjx), tbz, toe[1], tv[1], "toe_z")]

    def _forces(self, lam, rows, cs: ContactState):
        """Split multipliers into heel and toe (Fx, Fz) ground forces."""
        heel = [0.0, 0.0]
        toe = [0.0, 0.0]
        mu_s = cs.slide * self.mu
        for value, row in zip(lam, rows):
            tag = row[5]
            if tag == "heel_x":
                heel[0] += value
            elif tag == "toe_x":
                toe[0] += value
            elif tag == "heel_z":
                heel[1] += value
                heel[0] += mu_s * value
            elif tag == "toe_z":
                toe[1] += value
                toe[0] += mu_s * value
        return (heel[0], heel[1]), (toe[0], toe[1])

    # Model evaluation --------------------------------------------------------------
    def evaluate(self, t: float, y: list[float], cs: ContactState) -> dict:
        self.evaluations += 1
        body, n = self.body, self.n
        q = y[:n]
        qd = y[n : 2 * n]
        act = y[2 * n : 2 * n + 6]
        u = self.scenario.strategy.excitations(t, cs.mode is ContactMode.FLIGHT, self.quiet, self.u_min)
        tau_act = self.nm.activation_time_constant_s
        tau_deact = self.nm.deactivation_time_constant_s
        adot = [activation_derivative(u[i], act[i], tau_act, tau_deact) for i in range(6)]
        angles = body.joint_angles(q)
        omegas = body.joint_velocities(qd)
        net, actuator, passive = joint_torques(self.nm, act, angles, omegas)
        mass, f = body.dynamics_terms(q, qd)
        qj = body.joint_generalized_force(net)
        qc = body.coupling_generalized_force(q, qd)
        rhs_force = [f[i] + qj[i] + qc[i] for i in range(n)]

        rows = self._contact_rows(q, qd, cs)
        m = len(rows)
        if m:
            size = n + m
            kkt = [[0.0] * size for _ in range(size)]
            for i in range(n):
                kkt[i][:n] = mass[i]
            rhs = rhs_force + [0.0] * m
            b = self.settings.baumgarte_rate_s
            for r, (jrow, crow, bias, phi, phidot, _) in enumerate(rows):
                for col, val in jrow.items():
                    kkt[n + r][col] = val
                for col, val in crow.items():
                    kkt[col][n + r] = -val
                rhs[n + r] = -bias - 2.0 * b * phidot - b * b * phi
            sol = solve(kkt, rhs)
            qdd, lam = sol[:n], sol[n:]
        else:
            qdd, lam = solve(mass, rhs_force), []

        heel_force, toe_force = self._forces(lam, rows, cs)
        heel = body.point(q, self.heel_terms)
        toe = body.point(q, self.toe_terms)
        hv = body.point_velocity(q, qd, self.heel_terms)
        tv = body.point_velocity(q, qd, self.toe_terms)
        p_joint = sum(net[j] * omegas[j] for j in range(3))
        p_damp = body.coupling_damping_power(q, qd)
        # Power of the ground forces on the moving contact points (friction
        # dissipation while sliding; Baumgarte residual work while sticking).
        p_contact = (
            heel_force[0] * hv[0] + heel_force[1] * hv[1] + toe_force[0] * tv[0] + toe_force[1] * tv[1]
        )
        residual = max((abs(r[3]) for r in rows), default=0.0)
        body_min_z = min(body.point(q, body.points[name])[1] for name in ("knee", "hip", "shoulder", "head_top"))
        return {
            "body_min_z": body_min_z,
            "u": u, "act": act, "adot": adot, "qdd": qdd, "lam": lam,
            "net": net, "actuator": actuator, "passive": passive,
            "angles": angles, "omegas": omegas,
            "powers": (p_joint, p_damp, p_contact),
            "heel_force": heel_force, "toe_force": toe_force,
            "heel": heel, "toe": toe, "heel_v": hv, "toe_v": tv, "residual": residual,
        }

    def derivative(self, t: float, y: list[float], cs: ContactState) -> list[float]:
        out = self.evaluate(t, y, cs)
        n = self.n
        return y[n : 2 * n] + out["qdd"] + out["adot"] + list(out["powers"])

    def step(self, t, y, h, cs, k1=None, left_limit=None):
        # Excitations are piecewise constant in time. A step that ends exactly on
        # a control breakpoint evaluates its stages at the left limit so the
        # embedded error estimate is not polluted by the post-switch derivative.
        ks = [k1 if k1 is not None else self.derivative(t, y, cs)]
        size = len(y)
        for stage in range(1, 7):
            yi = y[:]
            for j, cij in enumerate(_A[stage]):
                if cij != 0.0:
                    kj = ks[j]
                    hc = h * cij
                    for i in range(size):
                        yi[i] += hc * kj[i]
            ts = t + _C[stage] * h
            if left_limit is not None:
                ts = min(ts, left_limit)
            ks.append(self.derivative(ts, yi, cs))
        y5 = y[:]
        err = [0.0] * size
        for j in range(7):
            b5, e = _B5[j], _E[j]
            kj = ks[j]
            if b5 != 0.0 or e != 0.0:
                for i in range(size):
                    y5[i] += h * b5 * kj[i]
                    err[i] += h * e * kj[i]
        s = self.settings
        total = 0.0
        for i in range(size):
            sc = s.atol + s.rtol * max(abs(y[i]), abs(y5[i]))
            total += (err[i] / sc) ** 2
        return y5, math.sqrt(total / size), ks[6]

    # Event functions -------------------------------------------------------------
    def event_values(self, cs: ContactState, out: dict) -> dict[str, float]:
        values = self._mode_event_values(cs, out)
        # Only the foot has contact points. Any other body point reaching the
        # ground (e.g. trunk/head after backward rotation in flight) is an
        # unmodeled contact, and the run must end there.
        values["BODY_GROUND_CONTACT_NOT_MODELED"] = out["body_min_z"]
        return values

    def _mode_event_values(self, cs: ContactState, out: dict) -> dict[str, float]:
        mu = self.mu
        hf, tf = out["heel_force"], out["toe_force"]
        if cs.mode is ContactMode.FLAT:
            values = {"HEEL_OFF": hf[1], "TOE_LIFT_NOT_MODELED": tf[1]}
            if cs.slide == 0:
                values["SLIP_ONSET"] = mu * (hf[1] + tf[1]) - abs(hf[0] + tf[0])
            else:
                values["SLIP_STOP"] = -cs.slide * out["heel_v"][0]
            return values
        if cs.mode is ContactMode.TOE:
            values = {"TAKEOFF": tf[1], "HEEL_STRIKE": out["heel"][1]}
            if cs.slide == 0:
                values["SLIP_ONSET"] = mu * tf[1] - abs(tf[0])
            else:
                values["SLIP_STOP"] = -cs.slide * out["toe_v"][0]
            return values
        # Touchdown is any foot point penetrating the ground by more than 1 um.
        return {"TOUCHDOWN": min(out["heel"][1], out["toe"][1]) + 1e-6}

    def heel_strike(self, y: list[float]) -> tuple[list[float], float, tuple[float, float, float], float]:
        """Perfectly inelastic flat-foot impact: exact rigid impulse.

        Returns (new state, kinetic energy change, impulses, pre-impact heel speed).
        """
        body, n = self.body, self.n
        q, qd = y[:n], y[n : 2 * n]
        mass, _ = body.dynamics_terms(q, qd)
        hjx, hjz, _, _ = body.point_jacobian(q, qd, self.heel_terms)
        _, tjz, _, _ = body.point_jacobian(q, qd, self.toe_terms)
        jrows = [[r.get(i, 0.0) for i in range(n)] for r in (hjx, hjz, tjz)]
        minv_jt = [solve(mass, row) for row in jrows]  # columns of M^-1 J^T
        a = [[sum(jrows[r][i] * minv_jt[c][i] for i in range(n)) for c in range(3)] for r in range(3)]
        jv = [sum(jrows[r][i] * qd[i] for i in range(n)) for r in range(3)]
        impulse = solve(a, [-v for v in jv])
        new_qd = [qd[i] + sum(minv_jt[c][i] * impulse[c] for c in range(3)) for i in range(n)]
        ke0 = body.kinetic_energy(q, qd)
        ke1 = body.kinetic_energy(q, new_qd)
        new_y = y[:]
        new_y[n : 2 * n] = new_qd
        new_y[2 * n + 8] += ke1 - ke0
        return new_y, ke1 - ke0, (impulse[0], impulse[1], impulse[2]), -jv[1]

    def sample(self, t: float, y: list[float], cs: ContactState, out: dict) -> JumpSample:
        body, n = self.body, self.n
        q, qd = y[:n], y[n : 2 * n]
        hf, tf = out["heel_force"], out["toe_force"]
        fx, fz = hf[0] + tf[0], hf[1] + tf[1]
        cop = None
        if cs.mode is not ContactMode.FLIGHT and fz > 0.0:
            cop = (hf[1] * out["heel"][0] + tf[1] * out["toe"][0]) / fz
        bar = (q[XB], q[ZB]) if body.bar is not None else None
        bar_v = (qd[XB], qd[ZB]) if body.bar is not None else None
        return JumpSample(
            time_s=t,
            mode=cs.mode,
            contact_state=cs.label,
            q=tuple(q),
            qd=tuple(qd),
            qdd=tuple(out["qdd"]),
            excitation=tuple(out["u"]),
            activation=tuple(out["act"]),
            heel_force_n=hf,
            toe_force_n=tf,
            grf_x_n=fx,
            grf_z_n=fz,
            cop_x_m=cop,
            joint_angle_rad=out["angles"],
            joint_velocity_rad_s=out["omegas"],
            joint_torque_n_m=out["net"],
            actuator_torque_n_m=out["actuator"],
            passive_torque_n_m=out["passive"],
            body_com_m=body.body_com(q),
            body_com_velocity_m_s=body.body_com_velocity(q, qd),
            system_com_m=body.system_com(q),
            system_com_velocity_m_s=body.system_com_velocity(q, qd),
            bar_m=bar,
            bar_velocity_m_s=bar_v,
            coupling_force_on_bar_n=body.coupling_force(q, qd),
            heel_m=out["heel"],
            toe_m=out["toe"],
            kinetic_energy_j=body.kinetic_energy(q, qd),
            potential_energy_j=body.potential_energy(q),
            work_joint_j=y[2 * n + 6],
            work_coupling_damping_j=y[2 * n + 7],
            work_contact_j=y[2 * n + 8],
            constraint_residual_m=out["residual"],
        )


def simulate_jump(scenario: JumpScenario) -> JumpTrajectory:
    """Integrate the declared world until landing contact, refusal or horizon."""
    eng = _Engine(scenario)
    s = scenario.settings
    mu = eng.mu
    t = 0.0
    y = eng.initial_state()
    cs = ContactState(ContactMode.FLAT, 0, eng.body.point(y[: eng.n], eng.heel_terms)[0])
    out = eng.evaluate(t, y, cs)
    samples = [eng.sample(t, y, cs, out)]
    events: list[JumpEvent] = []
    breakpoints = [bp for bp in sorted(set(scenario.strategy.breakpoints())) if 0.0 < bp < s.horizon_s]
    h = 1e-4
    k1 = None
    termination = "HORIZON_REACHED"
    takeoff_time: float | None = None

    def settle(state: ContactState, y_now: list[float]) -> tuple[ContactState, dict]:
        """Re-check unilateral and friction conditions after a state change."""
        for _ in range(4):
            o = eng.evaluate(t, y_now, state)
            hf, tf = o["heel_force"], o["toe_force"]
            if state.mode is ContactMode.TOE and tf[1] <= 0.0:
                state = ContactState(ContactMode.FLIGHT)
                continue
            if state.mode is not ContactMode.FLIGHT and state.slide == 0:
                fz, fx = hf[1] + tf[1], hf[0] + tf[0]
                if abs(fx) > mu * fz:
                    state = ContactState(state.mode, 1 if fx > 0.0 else -1, 0.0)
                    continue
            return state, o
        raise ArithmeticError("contact state did not settle")

    while True:
        if eng.evaluations > s.max_rhs_evaluations:
            events.append(JumpEvent(t, "SOLVER_BUDGET_EXCEEDED", "evaluation budget exhausted"))
            termination = "SOLVER_BUDGET_EXCEEDED"
            break
        if len(events) > s.max_contact_events:
            events.append(JumpEvent(t, "EVENT_CHATTER", "contact event budget exhausted"))
            termination = "EVENT_CHATTER"
            break
        if t >= s.horizon_s - 1e-15:
            termination = "HORIZON_REACHED"
            break
        limit = min(s.max_step_s, s.horizon_s - t)
        next_bp = next((bp for bp in breakpoints if bp > t + 1e-15), None)
        hit_bp = next_bp is not None and next_bp - t <= min(h, limit)
        h_try = next_bp - t if hit_bp else min(h, limit)
        left = math.nextafter(next_bp, -math.inf) if hit_bp else None
        y_new, err, k_last = eng.step(t, y, h_try, cs, k1, left)
        if err > 1.0:
            h = h_try * max(0.2, 0.9 * err ** -0.2)
            if h < 1e-13:
                raise ArithmeticError(f"step size underflow at t={t!r} in {cs.label}")
            continue
        out_new = eng.evaluate(t + h_try, y_new, cs)
        before = eng.event_values(cs, out)
        after = eng.event_values(cs, out_new)
        crossed = [k for k, v in after.items() if before.get(k, 1.0) > 0.0 and v < 0.0]
        if crossed:
            lo, hi, first = 0.0, h_try, crossed
            while hi - lo > s.event_time_tolerance_s:
                mid = 0.5 * (lo + hi)
                y_mid, _, _ = eng.step(t, y, mid, cs, None, left)
                vals = eng.event_values(cs, eng.evaluate(t + mid, y_mid, cs))
                neg = [k for k in crossed if vals[k] < 0.0]
                if neg:
                    hi, first = mid, neg
                else:
                    lo = mid
            if lo > 0.0:
                y, _, _ = eng.step(t, y, lo, cs, None, left)
                t += lo
            out = eng.evaluate(t, y, cs)
            samples.append(eng.sample(t, y, cs, out))
            kind = first[0]
            k1 = None
            if kind == "HEEL_OFF":
                events.append(JumpEvent(t, "HEEL_OFF", "heel normal force reached zero"))
                anchor = out["toe"][0]
                cs, out = settle(ContactState(ContactMode.TOE, cs.slide, anchor), y)
                if cs.mode is ContactMode.FLIGHT:
                    events.append(JumpEvent(t, "TAKEOFF", "heel and toe released together"))
                    takeoff_time = t
            elif kind == "TAKEOFF":
                events.append(JumpEvent(t, "TAKEOFF", "toe normal force reached zero"))
                takeoff_time = t
                cs, out = settle(ContactState(ContactMode.FLIGHT), y)
            elif kind == "SLIP_ONSET":
                fx = out["heel_force"][0] + out["toe_force"][0]
                events.append(JumpEvent(t, "SLIP_ONSET", "friction cone boundary reached", fx))
                cs, out = settle(ContactState(cs.mode, 1 if fx > 0.0 else -1, 0.0), y)
            elif kind == "SLIP_STOP":
                point = out["heel"] if cs.mode is ContactMode.FLAT else out["toe"]
                events.append(JumpEvent(t, "SLIP_STOP", "slip velocity reached zero"))
                cs, out = settle(ContactState(cs.mode, 0, point[0]), y)
            elif kind == "HEEL_STRIKE":
                new_y, d_ke, impulse, speed = eng.heel_strike(y)
                valid = (
                    speed <= MAX_QUALIFIED_IMPACT_SPEED_M_S
                    and impulse[1] >= 0.0
                    and impulse[2] >= 0.0
                    and abs(impulse[0]) <= mu * (impulse[1] + impulse[2])
                )
                if not valid:
                    events.append(JumpEvent(t, "HEEL_IMPACT_NOT_QUALIFIED", f"heel speed {speed:.3f} m/s", speed))
                    termination = "HEEL_IMPACT_NOT_QUALIFIED"
                    break
                y = new_y
                events.append(
                    JumpEvent(t, "HEEL_STRIKE", f"inelastic impulse; heel speed {speed:.4f} m/s; dKE {d_ke:.5f} J", d_ke)
                )
                cs, out = settle(ContactState(ContactMode.FLAT, 0, out["heel"][0]), y)
            elif kind == "TOUCHDOWN":
                # A flight is sustained only if the foot returns after the
                # ballistic apex of the system COM (t >= v_takeoff / g). An
                # earlier re-contact is a toe tap/stutter, not a jump landing.
                flight = t - (takeoff_time if takeoff_time is not None else t)
                apex_time = 0.0
                if takeoff_time is not None:
                    v_to = next(x for x in samples if x.time_s >= takeoff_time).system_com_velocity_m_s[1]
                    apex_time = max(0.0, v_to) / scenario.body.gravity_m_s2
                reason = (
                    "LANDING_CONTACT_NOT_MODELED"
                    if takeoff_time is not None and flight >= apex_time
                    else "TAKEOFF_NOT_SUSTAINED"
                )
                events.append(JumpEvent(t, reason, f"foot reached ground after {flight:.4f} s of flight", flight))
                termination = reason
                break
            else:
                events.append(JumpEvent(t, kind, "run ended: mechanism outside the model"))
                termination = kind
                break
            samples.append(eng.sample(t, y, cs, out))
            h = max(min(h, s.max_step_s), 1e-6)
            continue

        t += h_try
        y = y_new
        out = out_new
        k1 = k_last
        samples.append(eng.sample(t, y, cs, out))
        if hit_bp:
            events.append(JumpEvent(t, "CONTROL_BREAKPOINT", "declared excitation change"))
            k1 = None  # excitation is discontinuous at a breakpoint
        if cs.mode is not ContactMode.FLIGHT and samples[-1].body_com_m[1] < s.collapse_com_height_m:
            events.append(JumpEvent(t, "COLLAPSE", "body COM below declared collapse height"))
            termination = "COLLAPSE"
            break
        factor = 5.0 if err == 0.0 else min(5.0, max(0.2, 0.9 * err ** -0.2))
        h = h_try * factor

    return JumpTrajectory(
        scenario=scenario,
        static=eng.static,
        samples=tuple(samples),
        events=tuple(events),
        termination=termination,
        rhs_evaluations=eng.evaluations,
    )
