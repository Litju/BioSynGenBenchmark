"""RES-624 Gate C: typed nominal and anthropometric qualification scopes.

A single aggregate boolean cannot distinguish "the nominal 20 kg hold is
qualified" from "the hand model generalizes across the preregistered
anthropometries". This module derives three typed verdicts from a
qualification report:

``NOMINAL_RECEIVING_HOLD_QUALIFIED``
    The mandatory nominal physical cases, the preserved RED refusal, every
    declared adversarial refusal and every positive control pass. A failing
    *required anthropometric* case does not corrupt this narrower claim, and an
    explicitly exploratory subject cannot invalidate it.

``ANTHROPOMETRIC_GENERALIZATION_QUALIFIED``
    Every preregistered required subject case is present, qualified, certified
    and identity-consistent (declared mass/stature, exported human mass, and a
    distinct compiled parameter set per subject). A missing, failing,
    mislabeled or evidence-inconsistent required subject prohibits it.

``OVERALL_DECLARED_SCOPE_QUALIFIED``
    Both of the above; otherwise ``NOMINAL_SCOPE_QUALIFIED_GENERALIZATION_NOT_QUALIFIED``
    or ``NOT_QUALIFIED``. A generalized population claim is never made.

NUMERICAL_VERIFICATION_ONLY.
"""
from __future__ import annotations

import math

REQUIRED_NOMINAL = ("nominal_0p3s_red_window", "nominal_2s", "nominal_10s")
REQUIRED_ANTHROPOMETRIC = {
    "anthropometry_70kg_1p70m": dict(body_mass_kg=70.0, stature_m=1.70),
    "anthropometry_80kg_1p80m": dict(body_mass_kg=80.0, stature_m=1.80),
}
EXPLORATORY_PREFIX = "exploratory_"
NOMINAL_VERDICT = "NOMINAL_RECEIVING_HOLD_QUALIFIED"
NOMINAL_NOT_VERDICT = "NOMINAL_RECEIVING_HOLD_NOT_QUALIFIED"
GENERALIZATION_VERDICT = "ANTHROPOMETRIC_GENERALIZATION_QUALIFIED"
GENERALIZATION_NOT_VERDICT = "ANTHROPOMETRIC_GENERALIZATION_NOT_QUALIFIED"
OVERALL_VERDICT = "OVERALL_DECLARED_SCOPE_QUALIFIED"
OVERALL_NOMINAL_ONLY = "NOMINAL_SCOPE_QUALIFIED_GENERALIZATION_NOT_QUALIFIED"
OVERALL_NOT_VERDICT = "NOT_QUALIFIED"
MIN_SUPPORT_RATIO = .98
MIN_ACTIVE_CONTACTS = 2
MAX_COMPATIBLE_PENETRATION_M = 1.4e-3
MAX_BAR_DISPLACEMENT_M = .014
MASS_TOLERANCE_KG = 1e-3


def is_exploratory(name, case):
    return str(name).startswith(EXPLORATORY_PREFIX) or bool(case.get("exploratory"))


def is_negative(case):
    expectation = str(case.get("expectation", ""))
    return expectation.startswith("REFUSED") or "pre-release" in expectation


def _finite_number(value):
    """Only finite JSON numeric scalars are admissible for physical evidence."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) else None


def evidence_problems(case):
    """Fail closed on missing, malformed or nonfinite positive-case evidence."""
    ev = case.get("evidence")
    if not isinstance(ev, dict):
        return ["PHYSICAL_EVIDENCE_MISSING"]
    problems = []
    def quantity(key):
        return _finite_number(ev.get(key))

    support = quantity("hand_support_impulse_ratio")
    if support is None or support < MIN_SUPPORT_RATIO:
        problems.append("HAND_SUPPORT_RATIO_BELOW_NOMINAL")
    contacts = quantity("min_grasp_active_contacts")
    if contacts is None or contacts < MIN_ACTIVE_CONTACTS or not contacts.is_integer():
        problems.append("INSUFFICIENT_ACTIVE_GRASP_CONTACTS")
    displacement = quantity("bar_displacement_in_hand_m")
    if displacement is None or displacement < 0 or displacement >= MAX_BAR_DISPLACEMENT_M:
        problems.append("BAR_NOT_STABLE_IN_HAND")
    penetration = quantity("max_penetration_m")
    if penetration is None or penetration < 0 or penetration > MAX_COMPATIBLE_PENETRATION_M:
        problems.append("PENETRATION_BEYOND_DECLARED_BOUND")
    for key, problem in (("posture_saturation_samples", "POSTURE_ACTUATOR_SATURATION"),
                         ("contact_problems", "CONTACT_INADMISSIBILITY_RECORDED")):
        count = quantity(key)
        if count is None or count != 0:
            problems.append(problem)
    identities = ev.get("grasp_identity_problems")
    if not isinstance(identities, list) or identities:
        problems.append("GRASP_IDENTITY_PROBLEMS_RECORDED")
    pose = ev.get("relative_bar_pose")
    for key, limit, prefix in (("max_tilt_deg", 2., "RELATIVE_TILT"),
                               ("max_axial_slip_m", .002, "AXIAL_SLIP")):
        values = pose.get(key) if isinstance(pose, dict) else None
        for side in ("l", "r"):
            value = _finite_number(values.get(side)) if isinstance(values, dict) else None
            if value is None or value < 0 or value >= limit:
                problems.append(f"{prefix}_{side.upper()}_BEYOND_GRIP")
    return problems

def case_problems(case, name):
    if not case:
        return [f"{name}:MISSING"]
    problems = []
    if is_negative(case):
        if case.get("expectation_met") is not True:
            problems.append(f"{name}:NEGATIVE_EXPECTATION_NOT_MET")
        return problems
    if case.get("outcome") != "RECEIVING_HOLD_QUALIFIED":
        problems.append(f"{name}:OUTCOME={case.get('outcome')}")
    if case.get("expectation_met") is not True:
        problems.append(f"{name}:EXPECTATION_NOT_MET")
    if case.get("static_verdict") != "STATIC_EQUILIBRIUM_CERTIFIED":
        problems.append(f"{name}:STATIC={case.get('static_verdict')}")
    problems += [f"{name}:{p}" for p in evidence_problems(case)]
    return problems


def subject_problems(case, required):
    subject = case.get("subject")
    if not isinstance(subject, dict) or not subject:
        return ["SUBJECT_METADATA_MISSING"]
    problems = []
    for field, expected, tolerance, label in (
        ("body_mass_kg", required["body_mass_kg"], 1e-9, "SUBJECT_MASS"),
        ("stature_m", required["stature_m"], 1e-9, "SUBJECT_STATURE"),
        ("model_human_mass_kg", required["body_mass_kg"], MASS_TOLERANCE_KG, "MODEL_HUMAN_MASS"),
    ):
        measured = _finite_number(subject.get(field))
        if measured is None or abs(measured-expected) > tolerance:
            problems.append(f"{label}={subject.get(field)}!=REQUIRED={expected}")
    if subject.get("mass_consistent") is not True:
        problems.append("SUBJECT_MASS_INCONSISTENT")
    return problems

def qualify_scope(report, required_nominal=REQUIRED_NOMINAL, required_anthropometric=REQUIRED_ANTHROPOMETRIC):
    cases = report.get("cases") or {}
    nominal_problems = []
    for name in required_nominal:
        nominal_problems += case_problems(cases.get(name), name)
    for name, case in cases.items():
        if name in required_anthropometric or name in required_nominal or is_exploratory(name, case):
            continue
        nominal_problems += case_problems(case, name)
    red = report.get("red_canary") or {}
    if red.get("refused") is not True:
        nominal_problems.append("FROZEN_RES618_RED_NOT_REFUSED")
    adversarial = report.get("adversarial") or {}
    if not adversarial:
        nominal_problems.append("ADVERSARIAL_CONTROLS_MISSING")
    for name, entry in adversarial.items():
        if entry.get("refused") is not True:
            nominal_problems.append(f"ADVERSARIAL_NOT_REFUSED:{name}")
    positives = report.get("adversarial_positive") or {}
    if not positives:
        nominal_problems.append("POSITIVE_CONTROLS_MISSING")
    for name, entry in positives.items():
        if entry.get("qualified") is not True:
            nominal_problems.append(f"POSITIVE_NOT_QUALIFIED:{name}")
    generalisation_problems = []
    models = (report.get("receipt") or {}).get("models") or {}
    hashes = []
    for name, required in required_anthropometric.items():
        case = cases.get(name)
        generalisation_problems += case_problems(case, name)
        if case:
            generalisation_problems += [f"{name}:{p}" for p in subject_problems(case, required)]
        model = models.get(name) or {}
        if model.get("parameter_set_sha256"):
            hashes.append((name, model["parameter_set_sha256"]))
        else:
            generalisation_problems.append(f"{name}:PARAMETER_SET_IDENTITY_MISSING")
    if len({h for _, h in hashes}) != len(hashes):
        generalisation_problems.append("REQUIRED_SUBJECTS_SHARE_A_MODEL_PARAMETER_SET")
    nominal_ok = not nominal_problems
    generalisation_ok = nominal_ok and not generalisation_problems
    if generalisation_ok:
        overall = OVERALL_VERDICT
        statement = ("declared scope qualified: the nominal 20 kg overhead receiving hold AND every preregistered "
                     "required subject case (70 kg/1.70 m, 80 kg/1.80 m) pass. This is a model-scoped numerical "
                     "result, not a population generalization or biological validation.")
    elif nominal_ok:
        overall = OVERALL_NOMINAL_ONLY
        statement = ("only the nominal 20 kg overhead receiving hold is qualified. Anthropometric generalization "
                     "is NOT established; the required subject cases are missing, failing or identity-inconsistent.")
    else:
        overall = OVERALL_NOT_VERDICT
        statement = "neither the nominal hold nor anthropometric generalization is qualified."
    return dict(
        nominal=dict(verdict=NOMINAL_VERDICT if nominal_ok else NOMINAL_NOT_VERDICT, qualified=nominal_ok,
                     required_cases=list(required_nominal), problems=nominal_problems),
        anthropometric_generalization=dict(
            verdict=GENERALIZATION_VERDICT if generalisation_ok else GENERALIZATION_NOT_VERDICT,
            qualified=generalisation_ok, required_cases=dict(required_anthropometric),
            problems=generalisation_problems),
        overall=dict(verdict=overall, nominal_only=bool(nominal_ok and not generalisation_ok), statement=statement),
        scope_statement=statement,
        exploratory_cases=[n for n, c in cases.items() if is_exploratory(n, c)],
    )
