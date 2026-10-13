"""Candidate-independent checks over declared public measurement arrays.

This module deliberately depends only on the Python standard library. It does
not import the generator, its latent state, or its observation implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import sys


NUMERICAL_CEILING = "NUMERICAL_VERIFICATION_ONLY"
SIMULATED_CEILING = "SIMULATED_MEASUREMENT_ONLY"
BIOLOGICAL_CEILING = "BIOLOGICAL_VALIDATION_NOT_ESTABLISHED"
PAIRED_CEILING = "REAL_PAIRED_FORCE_LPT_NOT_ESTABLISHED"


class ClaimKind(str, Enum):
    FORCE_IMPULSE_BALANCE = "force_impulse_balance"
    BALLISTIC_FLIGHT = "ballistic_flight"
    CABLE_GEOMETRY = "cable_geometry"
    BILATERAL_FORCE = "bilateral_force"
    BIOLOGICAL_VALIDITY = "biological_validity"
    REAL_PAIRED_FORCE_LPT = "real_paired_force_lpt"


class Outcome(str, Enum):
    PASS = "PASS"
    REJECTED = "REJECTED"
    NOT_EVALUABLE = "NOT_EVALUABLE"


class SupportCoverage(str, Enum):
    TOTAL_SYSTEM = "TOTAL_SYSTEM"
    TWO_FOOT_CHANNELS = "TWO_FOOT_CHANNELS"
    PARTIAL = "PARTIAL"


@dataclass(frozen=True, slots=True)
class ClockContract:
    """Declared affine map from simulation seconds to native clock seconds."""

    sample_period_native_s: float
    offset_native_s: float
    rate_scale: float
    qualified: bool

    def __post_init__(self) -> None:
        _positive_finite("sample_period_native_s", self.sample_period_native_s)
        _finite("offset_native_s", self.offset_native_s)
        _positive_finite("rate_scale", self.rate_scale)
        if not isinstance(self.qualified, bool):
            raise ValueError("qualified must be a boolean")

    def simulation_time_s(self, native_time_s: float) -> float:
        _finite("native_time_s", native_time_s)
        return (native_time_s - self.offset_native_s) / self.rate_scale


@dataclass(frozen=True, slots=True)
class ForceMeasurements:
    sample_indices: tuple[int, ...]
    native_times_s: tuple[float, ...]
    total_force_n: tuple[float | None, ...]
    clock: ClockContract
    support_coverage: SupportCoverage
    source_stage: str
    units: str = "N"
    frame: str = "inertial_lab_z"
    positive_direction: str = "upward"
    left_force_n: tuple[float | None, ...] | None = None
    right_force_n: tuple[float | None, ...] | None = None
    bilateral_split_fraction: float | None = None


@dataclass(frozen=True, slots=True)
class LptMeasurements:
    sample_indices: tuple[int, ...]
    native_times_s: tuple[float, ...]
    cable_length_m: tuple[float | None, ...]
    cable_rate_m_s: tuple[float | None, ...]
    cable_angle_rad: tuple[float | None, ...]
    clock: ClockContract
    source_stage: str
    attachment_target: str
    length_units: str = "m"
    rate_units: str = "m/s"
    angle_units: str = "rad"


@dataclass(frozen=True, slots=True)
class ForceImpulseReference:
    """Independent known-answer terms for M * delta-v = support + other impulses."""

    mass_kg: float
    start_simulation_time_s: float
    end_simulation_time_s: float
    initial_velocity_m_s: float
    final_velocity_m_s: float
    other_external_impulse_n_s: float
    external_impulses_complete: bool
    tolerance_n_s: float
    time_tolerance_s: float
    expected_clock: ClockContract | None = None


@dataclass(frozen=True, slots=True)
class FlightReference:
    simulation_times_s: tuple[float, ...]
    tolerance_n: float
    time_tolerance_s: float
    expected_force_n: float = 0.0
    expected_clock: ClockContract | None = None


@dataclass(frozen=True, slots=True)
class BilateralForcePoint:
    simulation_time_s: float
    total_force_n: float
    left_force_n: float
    right_force_n: float


@dataclass(frozen=True, slots=True)
class BilateralForceReference:
    points: tuple[BilateralForcePoint, ...]
    tolerance_n: float
    time_tolerance_s: float
    expected_clock: ClockContract | None = None


@dataclass(frozen=True, slots=True)
class CableGeometryPoint:
    simulation_time_s: float
    anchor_xz_m: tuple[float, float]
    attachment_xz_m: tuple[float, float]
    attachment_velocity_xz_m_s: tuple[float, float]


@dataclass(frozen=True, slots=True)
class CableGeometryReference:
    points: tuple[CableGeometryPoint, ...]
    length_tolerance_m: float
    rate_tolerance_m_s: float
    angle_tolerance_rad: float
    time_tolerance_s: float
    expected_clock: ClockContract | None = None


@dataclass(frozen=True, slots=True)
class EmpiricalReference:
    source_ids: tuple[str, ...]
    protocol_comparable: bool
    force_calibrated: bool
    lpt_calibrated: bool
    synchronized_pair: bool


@dataclass(frozen=True, slots=True)
class MeasurementSubmission:
    """Public input boundary. Candidate latent truth is not a field."""

    candidate_id: str
    claims: tuple[ClaimKind, ...]
    force: ForceMeasurements | None = None
    lpt: LptMeasurements | None = None
    impulse_reference: ForceImpulseReference | None = None
    flight_reference: FlightReference | None = None
    bilateral_reference: BilateralForceReference | None = None
    cable_reference: CableGeometryReference | None = None
    empirical_reference: EmpiricalReference | None = None


@dataclass(frozen=True, slots=True)
class ClaimFinding:
    claim: ClaimKind
    outcome: Outcome
    code: str
    reason: str
    evidence_ceiling: str
    metrics: tuple[tuple[str, float], ...] = ()


@dataclass(frozen=True, slots=True)
class EvaluationCounts:
    evaluated: int
    eligible: int
    admitted: int
    rejected: int
    not_evaluable: int


@dataclass(frozen=True, slots=True)
class EvaluationReport:
    candidate_id: str
    findings: tuple[ClaimFinding, ...]
    counts: EvaluationCounts


class _InvalidData(Exception):
    def __init__(self, code: str, reason: str) -> None:
        super().__init__(reason)
        self.code = code


class _UnqualifiedData(Exception):
    pass


def evaluate(submission: MeasurementSubmission) -> EvaluationReport:
    """Evaluate only requested claims against declared public evidence."""
    if not isinstance(submission, MeasurementSubmission):
        raise TypeError("evaluate accepts a MeasurementSubmission public boundary")
    if len(set(submission.claims)) != len(submission.claims):
        raise ValueError("claim requests must not contain duplicates")

    findings: list[ClaimFinding] = []
    for claim in submission.claims:
        if claim is ClaimKind.FORCE_IMPULSE_BALANCE:
            finding = _evaluate_impulse(submission)
        elif claim is ClaimKind.BALLISTIC_FLIGHT:
            finding = _evaluate_flight(submission)
        elif claim is ClaimKind.CABLE_GEOMETRY:
            finding = _evaluate_cable_geometry(submission)
        elif claim is ClaimKind.BILATERAL_FORCE:
            finding = _evaluate_bilateral_force(submission)
        elif claim is ClaimKind.BIOLOGICAL_VALIDITY:
            finding = _evaluate_empirical_biological(submission)
        elif claim is ClaimKind.REAL_PAIRED_FORCE_LPT:
            finding = _evaluate_paired_empirical(submission)
        else:
            finding = _not_evaluable(
                claim,
                "UNSUPPORTED_CLAIM",
                "This evaluator does not implement the requested claim.",
                SIMULATED_CEILING,
            )
        findings.append(finding)

    counts = EvaluationCounts(
        evaluated=len(findings),
        eligible=sum(f.outcome is not Outcome.NOT_EVALUABLE for f in findings),
        admitted=sum(f.outcome is Outcome.PASS for f in findings),
        rejected=sum(f.outcome is Outcome.REJECTED for f in findings),
        not_evaluable=sum(f.outcome is Outcome.NOT_EVALUABLE for f in findings),
    )
    return EvaluationReport(
        candidate_id=submission.candidate_id,
        findings=tuple(findings),
        counts=counts,
    )


def _evaluate_impulse(submission: MeasurementSubmission) -> ClaimFinding:
    claim = ClaimKind.FORCE_IMPULSE_BALANCE
    force = submission.force
    reference = submission.impulse_reference
    if force is None or reference is None:
        return _not_evaluable(
            claim,
            "IMPULSE_REFERENCE_MISSING",
            "A public FORCE series and independent numerical impulse reference are required.",
            NUMERICAL_CEILING,
        )
    if reference.expected_clock is None:
        return _not_evaluable(
            claim,
            "CLOCK_REFERENCE_MISSING",
            "An independent clock reference is required for impulse comparison.",
            NUMERICAL_CEILING,
        )
    try:
        times = _validate_force(force)
        _check_expected_clock(force.clock, reference.expected_clock)
        _validate_impulse_reference(reference)
    except _UnqualifiedData as error:
        return _not_evaluable(claim, "CLOCK_UNQUALIFIED", str(error), NUMERICAL_CEILING)
    except _InvalidData as error:
        return _rejected(claim, error.code, str(error), NUMERICAL_CEILING)

    if force.support_coverage is SupportCoverage.PARTIAL:
        return _not_evaluable(
            claim,
            "SUPPORT_COVERAGE_INCOMPLETE",
            "Momentum balance needs all declared external support reactions.",
            NUMERICAL_CEILING,
        )
    if not reference.external_impulses_complete:
        return _not_evaluable(
            claim,
            "OTHER_EXTERNAL_IMPULSES_INCOMPLETE",
            (
                "The known-answer reference does not cover all other external "
                "impulses, including gravity."
            ),
            NUMERICAL_CEILING,
        )
    if len(times) < 2:
        return _not_evaluable(
            claim,
            "INSUFFICIENT_FORCE_SAMPLES",
            "Impulse integration requires at least two timed samples.",
            NUMERICAL_CEILING,
        )
    start_index = _find_time_index(
        times, reference.start_simulation_time_s, reference.time_tolerance_s
    )
    end_index = _find_time_index(
        times, reference.end_simulation_time_s, reference.time_tolerance_s
    )
    if start_index is None or end_index is None or end_index <= start_index:
        return _not_evaluable(
            claim,
            "IMPULSE_INTERVAL_NOT_OBSERVED",
            "The declared impulse interval must have observed start and end samples.",
            NUMERICAL_CEILING,
        )
    force_values = force.total_force_n[start_index : end_index + 1]
    interval_times = times[start_index : end_index + 1]
    if any(value is None for value in force_values):
        return _not_evaluable(
            claim,
            "FORCE_SAMPLE_MISSING",
            "A missing FORCE sample prevents this numerical impulse check.",
            NUMERICAL_CEILING,
        )

    impulse_n_s = _trapezoid(interval_times, tuple(float(v) for v in force_values))
    momentum_change_n_s = reference.mass_kg * (
        reference.final_velocity_m_s - reference.initial_velocity_m_s
    )
    residual_n_s = (
        impulse_n_s
        + reference.other_external_impulse_n_s
        - momentum_change_n_s
    )
    metrics = (
        ("support_impulse_n_s", impulse_n_s),
        ("momentum_change_n_s", momentum_change_n_s),
        ("balance_residual_n_s", residual_n_s),
    )
    if abs(residual_n_s) > reference.tolerance_n_s:
        return _rejected(
            claim,
            "INCONSISTENT_IMPULSE",
            "Integrated covered external impulse disagrees with the independent momentum change.",
            NUMERICAL_CEILING,
            metrics,
        )
    return _passed(
        claim,
        "KNOWN_ANSWER_IMPULSE_PASS",
        "Covered external impulse matches the independent numerical reference.",
        NUMERICAL_CEILING,
        metrics,
    )


def _evaluate_flight(submission: MeasurementSubmission) -> ClaimFinding:
    claim = ClaimKind.BALLISTIC_FLIGHT
    force = submission.force
    reference = submission.flight_reference
    if force is None or reference is None:
        return _not_evaluable(
            claim,
            "FLIGHT_REFERENCE_MISSING",
            "A public FORCE series and independent flight-time reference are required.",
            NUMERICAL_CEILING,
        )
    if reference.expected_clock is None:
        return _not_evaluable(
            claim,
            "CLOCK_REFERENCE_MISSING",
            "An independent clock reference is required for flight comparison.",
            NUMERICAL_CEILING,
        )
    try:
        times = _validate_force(force)
        _check_expected_clock(force.clock, reference.expected_clock)
        _positive_finite("tolerance_n", reference.tolerance_n)
        _positive_finite("time_tolerance_s", reference.time_tolerance_s)
        _finite("expected_force_n", reference.expected_force_n)
        for time_s in reference.simulation_times_s:
            _finite("flight simulation time", time_s)
    except _UnqualifiedData as error:
        return _not_evaluable(claim, "CLOCK_UNQUALIFIED", str(error), NUMERICAL_CEILING)
    except _InvalidData as error:
        return _rejected(claim, error.code, str(error), NUMERICAL_CEILING)

    if force.support_coverage is SupportCoverage.PARTIAL:
        return _not_evaluable(
            claim,
            "SUPPORT_COVERAGE_INCOMPLETE",
            "The declared series does not cover the required external support.",
            NUMERICAL_CEILING,
        )
    if not reference.simulation_times_s:
        return _not_evaluable(
            claim,
            "FLIGHT_REFERENCE_EMPTY",
            "At least one independent flight time is required.",
            NUMERICAL_CEILING,
        )
    for reference_time in reference.simulation_times_s:
        sample_index = _find_time_index(times, reference_time, reference.time_tolerance_s)
        if sample_index is None:
            return _not_evaluable(
                claim,
                "FLIGHT_TIME_NOT_OBSERVED",
                "An independent flight reference time has no matching public sample.",
                NUMERICAL_CEILING,
            )
        measured_force = force.total_force_n[sample_index]
        if measured_force is None:
            return _not_evaluable(
                claim,
                "FORCE_SAMPLE_MISSING",
                "A missing FORCE sample prevents this flight check.",
                NUMERICAL_CEILING,
            )
        if abs(measured_force - reference.expected_force_n) > reference.tolerance_n:
            return _rejected(
                claim,
                "IMPOSSIBLE_FLIGHT_GRF",
                "Public FORCE conflicts with the independent ideal-flight reference.",
                NUMERICAL_CEILING,
                (("flight_force_residual_n", measured_force - reference.expected_force_n),),
            )
    return _passed(
        claim,
        "BALLISTIC_FLIGHT_FORCE_PASS",
        "Public FORCE matches the independent ideal-flight reference at declared times.",
        NUMERICAL_CEILING,
    )


def _evaluate_cable_geometry(submission: MeasurementSubmission) -> ClaimFinding:
    claim = ClaimKind.CABLE_GEOMETRY
    lpt = submission.lpt
    reference = submission.cable_reference
    if lpt is None or reference is None:
        return _not_evaluable(
            claim,
            "CABLE_REFERENCE_MISSING",
            "Public LPT arrays and independent cable geometry points are required.",
            NUMERICAL_CEILING,
        )
    if reference.expected_clock is None:
        return _not_evaluable(
            claim,
            "CLOCK_REFERENCE_MISSING",
            "An independent clock reference is required for cable comparison.",
            NUMERICAL_CEILING,
        )
    try:
        times = _validate_lpt(lpt)
        _check_expected_clock(lpt.clock, reference.expected_clock)
        _positive_finite("length_tolerance_m", reference.length_tolerance_m)
        _positive_finite("rate_tolerance_m_s", reference.rate_tolerance_m_s)
        _positive_finite("angle_tolerance_rad", reference.angle_tolerance_rad)
        _positive_finite("time_tolerance_s", reference.time_tolerance_s)
    except _UnqualifiedData as error:
        return _not_evaluable(claim, "CLOCK_UNQUALIFIED", str(error), NUMERICAL_CEILING)
    except _InvalidData as error:
        return _rejected(claim, error.code, str(error), NUMERICAL_CEILING)

    residuals: list[tuple[str, float]] = []
    for point in reference.points:
        try:
            _finite("reference simulation time", point.simulation_time_s)
            length, rate, angle = _reference_cable_kinematics(point)
        except (ValueError, ZeroDivisionError) as error:
            return _rejected(
                claim,
                "INVALID_CABLE_REFERENCE",
                f"Independent cable reference is invalid: {error}",
                NUMERICAL_CEILING,
            )
        sample_index = _find_time_index(times, point.simulation_time_s, reference.time_tolerance_s)
        if sample_index is None:
            return _not_evaluable(
                claim,
                "CABLE_TIME_NOT_OBSERVED",
                "An independent geometry time has no matching public LPT sample.",
                NUMERICAL_CEILING,
            )
        measured = (
            lpt.cable_length_m[sample_index],
            lpt.cable_rate_m_s[sample_index],
            lpt.cable_angle_rad[sample_index],
        )
        if any(value is None for value in measured):
            return _not_evaluable(
                claim,
                "LPT_SAMPLE_MISSING",
                "A missing LPT value prevents this geometry check.",
                NUMERICAL_CEILING,
            )
        length_residual = float(measured[0]) - length
        rate_residual = float(measured[1]) - rate
        angle_residual = _wrap_angle(float(measured[2]) - angle)
        if abs(length_residual) > reference.length_tolerance_m:
            return _rejected(
                claim,
                "LPT_LENGTH_MISMATCH",
                "Observed cable length conflicts with the independent anchor geometry.",
                NUMERICAL_CEILING,
                (("cable_length_residual_m", length_residual),),
            )
        if abs(rate_residual) > reference.rate_tolerance_m_s:
            return _rejected(
                claim,
                "LPT_PROJECTION_MISMATCH",
                "Observed cable rate conflicts with the independent directional projection.",
                NUMERICAL_CEILING,
                (("cable_rate_residual_m_s", rate_residual),),
            )
        if abs(angle_residual) > reference.angle_tolerance_rad:
            return _rejected(
                claim,
                "LPT_ANGLE_MISMATCH",
                "Observed cable angle conflicts with the independent anchor geometry.",
                NUMERICAL_CEILING,
                (("cable_angle_residual_rad", angle_residual),),
            )
        residuals.extend(
            (
                ("cable_length_residual_m", length_residual),
                ("cable_rate_residual_m_s", rate_residual),
                ("cable_angle_residual_rad", angle_residual),
            )
        )
    if not reference.points:
        return _not_evaluable(
            claim,
            "CABLE_REFERENCE_EMPTY",
            "At least one independent cable geometry point is required.",
            NUMERICAL_CEILING,
        )
    return _passed(
        claim,
        "CABLE_GEOMETRY_PASS",
        "Public LPT length, rate, and angle match independent geometric references.",
        NUMERICAL_CEILING,
        tuple(residuals),
    )


def _evaluate_bilateral_force(submission: MeasurementSubmission) -> ClaimFinding:
    claim = ClaimKind.BILATERAL_FORCE
    force = submission.force
    if force is None:
        return _not_evaluable(
            claim,
            "BILATERAL_CHANNELS_MISSING",
            "No public FORCE channels were supplied.",
            SIMULATED_CEILING,
        )
    try:
        _validate_force(force)
    except _UnqualifiedData as error:
        return _not_evaluable(claim, "CLOCK_UNQUALIFIED", str(error), SIMULATED_CEILING)
    except _InvalidData as error:
        return _rejected(claim, error.code, str(error), SIMULATED_CEILING)

    has_left = force.left_force_n is not None
    has_right = force.right_force_n is not None
    if has_left != has_right:
        return _rejected(
            claim,
            "BILATERAL_CHANNEL_SHAPE_INVALID",
            "Left and right channels must be declared together.",
            SIMULATED_CEILING,
        )
    if not has_left:
        return _not_evaluable(
            claim,
            "COMBINED_FORCE_NOT_BILATERAL",
            "A combined support channel cannot identify left and right force.",
            SIMULATED_CEILING,
        )
    if (
        force.support_coverage is not SupportCoverage.TWO_FOOT_CHANNELS
        or force.bilateral_split_fraction is None
    ):
        return _rejected(
            claim,
            "INVENTED_BILATERAL_FROM_COMBINED_FORCE",
            (
                "Bilateral values were supplied without two declared foot "
                "channels and an explicit split assumption."
            ),
            SIMULATED_CEILING,
        )
    reference = submission.bilateral_reference
    if reference is None:
        return _not_evaluable(
            claim,
            "BILATERAL_REFERENCE_MISSING",
            "Declared foot channels need an independent numerical split reference for this check.",
            NUMERICAL_CEILING,
        )
    if reference.expected_clock is None:
        return _not_evaluable(
            claim,
            "CLOCK_REFERENCE_MISSING",
            "An independent clock reference is required for bilateral comparison.",
            NUMERICAL_CEILING,
        )
    try:
        _check_expected_clock(force.clock, reference.expected_clock)
        _positive_finite("tolerance_n", reference.tolerance_n)
        _positive_finite("time_tolerance_s", reference.time_tolerance_s)
    except _InvalidData as error:
        return _rejected(claim, error.code, str(error), NUMERICAL_CEILING)
    except _UnqualifiedData as error:
        return _not_evaluable(claim, "CLOCK_REFERENCE_MISSING", str(error), NUMERICAL_CEILING)
    if not reference.points:
        return _not_evaluable(
            claim,
            "BILATERAL_REFERENCE_EMPTY",
            "At least one independent bilateral known-answer point is required.",
            NUMERICAL_CEILING,
        )
    if any(value is None for value in force.left_force_n) or any(
        value is None for value in force.right_force_n
    ):
        return _not_evaluable(
            claim,
            "BILATERAL_SAMPLE_MISSING",
            "A missing foot channel prevents this bilateral check.",
            SIMULATED_CEILING,
        )
    times = _validated_times(force.sample_indices, force.native_times_s, force.clock)
    for point in reference.points:
        for value in (
            point.simulation_time_s,
            point.total_force_n,
            point.left_force_n,
            point.right_force_n,
        ):
            try:
                _finite("bilateral reference value", value)
            except ValueError as error:
                return _rejected(
                    claim,
                    "BILATERAL_REFERENCE_INVALID",
                    str(error),
                    NUMERICAL_CEILING,
                )
        sample_index = _find_time_index(
            times, point.simulation_time_s, reference.time_tolerance_s
        )
        if sample_index is None:
            return _not_evaluable(
                claim,
                "BILATERAL_TIME_NOT_OBSERVED",
                "An independent bilateral reference time has no matching FORCE sample.",
                NUMERICAL_CEILING,
            )
        measured = (
            force.total_force_n[sample_index],
            force.left_force_n[sample_index],
            force.right_force_n[sample_index],
        )
        if any(value is None for value in measured):
            return _not_evaluable(
                claim,
                "BILATERAL_SAMPLE_MISSING",
                "A missing total or foot FORCE value prevents this numerical check.",
                NUMERICAL_CEILING,
            )
        reference_sum = point.left_force_n + point.right_force_n
        if abs(reference_sum - point.total_force_n) > reference.tolerance_n:
            return _rejected(
                claim,
                "BILATERAL_REFERENCE_INCONSISTENT",
                "Independent left and right reference forces do not sum to the reference total.",
                NUMERICAL_CEILING,
            )
        assumed_left = point.total_force_n * force.bilateral_split_fraction
        assumed_right = point.total_force_n - assumed_left
        if (
            abs(point.left_force_n - assumed_left) > reference.tolerance_n
            or abs(point.right_force_n - assumed_right) > reference.tolerance_n
        ):
            return _rejected(
                claim,
                "BILATERAL_PARTITION_MISMATCH",
                "Independent bilateral reference conflicts with the declared split assumption.",
                NUMERICAL_CEILING,
            )
        for actual, expected, channel in zip(
            measured,
            (point.total_force_n, point.left_force_n, point.right_force_n),
            ("total", "left", "right"),
        ):
            residual = float(actual) - expected
            if abs(residual) > reference.tolerance_n:
                return _rejected(
                    claim,
                    "BILATERAL_KNOWN_ANSWER_MISMATCH",
                    f"Public {channel} FORCE disagrees with the independent split reference.",
                    NUMERICAL_CEILING,
                    ((f"{channel}_force_residual_n", residual),),
                )
    return _passed(
        claim,
        "BILATERAL_KNOWN_ANSWER_PASS",
        (
            "Declared foot channels match an independent numerical split; this "
            "does not establish empirical force partition fidelity."
        ),
        NUMERICAL_CEILING,
    )


def _evaluate_paired_empirical(submission: MeasurementSubmission) -> ClaimFinding:
    claim = ClaimKind.REAL_PAIRED_FORCE_LPT
    reference = submission.empirical_reference
    if (
        reference is None
        or not reference.source_ids
        or not reference.protocol_comparable
        or not reference.force_calibrated
        or not reference.lpt_calibrated
        or not reference.synchronized_pair
    ):
        return _not_evaluable(
            claim,
            "PAIRED_EMPIRICAL_REFERENCE_MISSING",
            (
                "No calibrated, protocol-comparable synchronized real FORCE/LPT "
                "reference is admitted in I0."
            ),
            PAIRED_CEILING,
        )
    return _not_evaluable(
        claim,
        "EMPIRICAL_EVALUATION_NOT_IMPLEMENTED",
        (
            "I0 does not implement empirical device or human validation even "
            "when a reference is described."
        ),
        PAIRED_CEILING,
    )


def _validate_force(force: ForceMeasurements) -> tuple[float, ...]:
    if not force.clock.qualified:
        raise _UnqualifiedData("FORCE clock qualification is absent.")
    if force.units != "N" or force.positive_direction != "upward" or not force.frame:
        raise _InvalidData("FORCE_CONTRACT_INVALID", "FORCE units, sign, or frame are undeclared.")
    if not force.source_stage:
        raise _InvalidData("SOURCE_STAGE_MISSING", "FORCE source stage is undeclared.")
    lengths = [len(force.sample_indices), len(force.native_times_s), len(force.total_force_n)]
    if len(set(lengths)) != 1 or not lengths[0]:
        raise _InvalidData(
            "FORCE_ARRAY_SHAPE_INVALID",
            "FORCE arrays must have equal nonzero length.",
        )
    if (force.left_force_n is None) != (force.right_force_n is None):
        raise _InvalidData(
            "BILATERAL_CHANNEL_SHAPE_INVALID",
            "Left and right channels must be declared together.",
        )
    if force.left_force_n is not None:
        if len(force.left_force_n) != lengths[0] or len(force.right_force_n) != lengths[0]:
            raise _InvalidData(
                "BILATERAL_CHANNEL_SHAPE_INVALID",
                "Bilateral array lengths must match total FORCE.",
            )
        if force.bilateral_split_fraction is not None and not (
            0.0 <= force.bilateral_split_fraction <= 1.0
        ):
            raise _InvalidData(
                "BILATERAL_PARTITION_INVALID",
                "Declared force split fraction is outside [0, 1].",
            )
    for value in force.total_force_n:
        if value is not None:
            _finite_or_invalid("total FORCE", value)
    if force.left_force_n is not None and force.right_force_n is not None:
        for left, right in zip(force.left_force_n, force.right_force_n):
            if left is not None:
                _finite_or_invalid("left FORCE", left)
            if right is not None:
                _finite_or_invalid("right FORCE", right)
    if not isinstance(force.support_coverage, SupportCoverage):
        raise _InvalidData("SUPPORT_COVERAGE_INVALID", "FORCE support coverage is undeclared.")
    return _validated_times(force.sample_indices, force.native_times_s, force.clock)


def _validate_lpt(lpt: LptMeasurements) -> tuple[float, ...]:
    if not lpt.clock.qualified:
        raise _UnqualifiedData("LPT clock qualification is absent.")
    if lpt.length_units != "m" or lpt.rate_units != "m/s" or lpt.angle_units != "rad":
        raise _InvalidData("LPT_UNITS_INVALID", "LPT units must be m, m/s, and rad.")
    if not lpt.source_stage or lpt.attachment_target not in {"body_com", "bar_attachment"}:
        raise _InvalidData(
            "LPT_METADATA_MISSING",
            "LPT source stage and attachment target are required.",
        )
    lengths = [
        len(lpt.sample_indices),
        len(lpt.native_times_s),
        len(lpt.cable_length_m),
        len(lpt.cable_rate_m_s),
        len(lpt.cable_angle_rad),
    ]
    if len(set(lengths)) != 1 or not lengths[0]:
        raise _InvalidData("LPT_ARRAY_SHAPE_INVALID", "LPT arrays must have equal nonzero length.")
    for length, rate, angle in zip(
        lpt.cable_length_m, lpt.cable_rate_m_s, lpt.cable_angle_rad
    ):
        if (length is None) != (rate is None) or (rate is None) != (angle is None):
            raise _InvalidData(
                "LPT_ARRAY_SHAPE_INVALID",
                "A sample must carry all LPT values or none.",
            )
        if length is not None:
            _finite_or_invalid("cable length", length)
            _finite_or_invalid("cable rate", rate)
            _finite_or_invalid("cable angle", angle)
    return _validated_times(lpt.sample_indices, lpt.native_times_s, lpt.clock)


def _validated_times(
    sample_indices: tuple[int, ...],
    native_times_s: tuple[float, ...],
    clock: ClockContract,
) -> tuple[float, ...]:
    if len(sample_indices) != len(native_times_s):
        raise _InvalidData(
            "TIMEBASE_INCONSISTENT",
            "Sample indices and native times differ in length.",
        )
    simulation_times: list[float] = []
    previous_index = -1
    previous_native: float | None = None
    for index, native_time_s in zip(sample_indices, native_times_s):
        if (
            not isinstance(index, int)
            or isinstance(index, bool)
            or index < 0
            or index <= previous_index
        ):
            raise _InvalidData(
                "TIMEBASE_INCONSISTENT",
                "Sample indices must be increasing nonnegative integers.",
            )
        _finite_or_invalid("native timestamp", native_time_s)
        expected_native = clock.offset_native_s + index * clock.sample_period_native_s
        tolerance = 32.0 * sys.float_info.epsilon * max(1.0, abs(expected_native))
        if abs(native_time_s - expected_native) > tolerance:
            raise _InvalidData(
                "TIMEBASE_INCONSISTENT",
                "Native timestamps disagree with the declared clock.",
            )
        if previous_native is not None and native_time_s <= previous_native:
            raise _InvalidData(
                "TIMEBASE_INCONSISTENT",
                "Native timestamps are not strictly increasing.",
            )
        simulation_time_s = clock.simulation_time_s(native_time_s)
        if simulation_times and simulation_time_s <= simulation_times[-1]:
            raise _InvalidData(
                "TIMEBASE_INCONSISTENT",
                "Mapped simulation times are not strictly increasing.",
            )
        simulation_times.append(simulation_time_s)
        previous_index = index
        previous_native = native_time_s
    return tuple(simulation_times)


def _validate_impulse_reference(reference: ForceImpulseReference) -> None:
    _positive_finite("mass_kg", reference.mass_kg)
    _finite("start_simulation_time_s", reference.start_simulation_time_s)
    _finite("end_simulation_time_s", reference.end_simulation_time_s)
    if reference.end_simulation_time_s <= reference.start_simulation_time_s:
        raise _InvalidData(
            "IMPULSE_INTERVAL_INVALID",
            "Impulse end time must follow its start time.",
        )
    _finite("initial_velocity_m_s", reference.initial_velocity_m_s)
    _finite("final_velocity_m_s", reference.final_velocity_m_s)
    _finite("other_external_impulse_n_s", reference.other_external_impulse_n_s)
    if not isinstance(reference.external_impulses_complete, bool):
        raise _InvalidData(
            "EXTERNAL_IMPULSE_COVERAGE_INVALID",
            "Reference must declare whether all other external impulses are covered.",
        )
    _positive_finite("tolerance_n_s", reference.tolerance_n_s)
    _positive_finite("time_tolerance_s", reference.time_tolerance_s)


def _check_expected_clock(
    observed: ClockContract,
    expected: ClockContract | None,
) -> None:
    if expected is None:
        raise _UnqualifiedData("An independent clock reference is required.")
    if observed != expected:
        raise _InvalidData(
            "TIMEBASE_INCONSISTENT",
            "Observed clock metadata differs from the independent clock reference.",
        )


def _reference_cable_kinematics(point: CableGeometryPoint) -> tuple[float, float, float]:
    ax, az = point.anchor_xz_m
    px, pz = point.attachment_xz_m
    vx, vz = point.attachment_velocity_xz_m_s
    for name, value in (
        ("anchor_x_m", ax),
        ("anchor_z_m", az),
        ("attachment_x_m", px),
        ("attachment_z_m", pz),
        ("attachment_velocity_x_m_s", vx),
        ("attachment_velocity_z_m_s", vz),
    ):
        _finite(name, value)
    dx = px - ax
    dz = pz - az
    length = math.hypot(dx, dz)
    if length == 0.0:
        raise ValueError("anchor and attachment coincide")
    rate = (dx * vx + dz * vz) / length
    angle = math.atan2(dz, dx)
    return length, rate, angle


def _trapezoid(times: tuple[float, ...], values: tuple[float, ...]) -> float:
    total = 0.0
    for left_time, right_time, left_value, right_value in zip(
        times, times[1:], values, values[1:]
    ):
        total += 0.5 * (left_value + right_value) * (right_time - left_time)
    return total


def _find_time_index(
    times: tuple[float, ...],
    target_time_s: float,
    tolerance_s: float,
) -> int | None:
    _positive_finite("time match tolerance", tolerance_s)
    if not times:
        return None
    closest = min(range(len(times)), key=lambda index: abs(times[index] - target_time_s))
    return closest if abs(times[closest] - target_time_s) <= tolerance_s else None


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def _evaluate_empirical_biological(submission: MeasurementSubmission) -> ClaimFinding:
    reference = submission.empirical_reference
    if reference is None or not reference.source_ids or not reference.protocol_comparable:
        return _not_evaluable(
            ClaimKind.BIOLOGICAL_VALIDITY,
            "BIOLOGICAL_REFERENCE_UNAVAILABLE",
            "No protocol-comparable independent human reference is admitted in I0.",
            BIOLOGICAL_CEILING,
        )
    return _not_evaluable(
        ClaimKind.BIOLOGICAL_VALIDITY,
        "BIOLOGICAL_EVALUATION_NOT_IMPLEMENTED",
        "I0 does not implement empirical biological validation.",
        BIOLOGICAL_CEILING,
    )


def _finite_or_invalid(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise _InvalidData("NONFINITE_MEASUREMENT", f"{name} must be finite.")


def _finite(name: str, value: float) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def _positive_finite(name: str, value: float) -> None:
    _finite(name, value)
    if value <= 0.0:
        raise _InvalidData("INVALID_NUMERICAL_TOLERANCE", f"{name} must be positive.")


def _not_evaluable(
    claim: ClaimKind,
    code: str,
    reason: str,
    ceiling: str,
) -> ClaimFinding:
    return ClaimFinding(claim, Outcome.NOT_EVALUABLE, code, reason, ceiling)


def _rejected(
    claim: ClaimKind,
    code: str,
    reason: str,
    ceiling: str,
    metrics: tuple[tuple[str, float], ...] = (),
) -> ClaimFinding:
    return ClaimFinding(claim, Outcome.REJECTED, code, reason, ceiling, metrics)


def _passed(
    claim: ClaimKind,
    code: str,
    reason: str,
    ceiling: str,
    metrics: tuple[tuple[str, float], ...] = (),
) -> ClaimFinding:
    return ClaimFinding(claim, Outcome.PASS, code, reason, ceiling, metrics)
