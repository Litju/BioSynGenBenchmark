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


def evidence_problems(case):
    """Independent evidence checks on a positive case: a relabelled pass cannot hide a weak hold."""
    ev = case.get("evidence") or {}
    problems = []
    if ev.get("hand_support_impulse_ratio") is None or ev["hand_support_impulse_ratio"] < MIN_SUPPORT_RATIO:
        problems.append("HAND_SUPPORT_RATIO_BELOW_NOMINAL")
    if ev.get("min_grasp_active_contacts") is None or ev["min_grasp_active_contacts"] < MIN_ACTIVE_CONTACTS:
        problems.append("INSUFFICIENT_ACTIVE_GRASP_CONTACTS")
    if ev.get("bar_displacement_in_hand_m") is None or ev["bar_displacement_in_hand_m"] >= MAX_BAR_DISPLACEMENT_M:
        problems.append("BAR_NOT_STABLE_IN_HAND")
    if ev.get("max_penetration_m") is None or ev["max_penetration_m"] > MAX_COMPATIBLE_PENETRATION_M:
        problems.append("PENETRATION_BEYOND_DECLARED_BOUND")
    if ev.get("posture_saturation_samples") != 0:
        problems.append("POSTURE_ACTUATOR_SATURATION")
    if ev.get("contact_problems") != 0:
        problems.append("CONTACT_INADMISSIBILITY_RECORDED")
    if ev.get("grasp_identity_problems"):
        problems.append("GRASP_IDENTITY_PROBLEMS_RECORDED")
    pose = ev.get("relative_bar_pose") or {}
    for side in ("l", "r"):
        if (pose.get("max_tilt_deg") or {}).get(side, 0.) >= 2.:
            problems.append(f"RELATIVE_TILT_{side.upper()}_BEYOND_GRIP")
        if (pose.get("max_axial_slip_m") or {}).get(side, 0.) >= .002:
            problems.append(f"AXIAL_SLIP_{side.upper()}_BEYOND_GRIP")
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
    subject = case.get("subject") or {}
    problems = []
    if not subject:
        return ["SUBJECT_METADATA_MISSING"]
    if abs(float(subject.get("body_mass_kg", "nan"))-required["body_mass_kg"]) > 1e-9:
        problems.append(f"SUBJECT_MASS={subject.get('body_mass_kg')}!=REQUIRED={required['body_mass_kg']}")
    if abs(float(subject.get("stature_m", "nan"))-required["stature_m"]) > 1e-9:
        problems.append(f"SUBJECT_STATURE={subject.get('stature_m')}!=REQUIRED={required['stature_m']}")
    model_mass = subject.get("model_human_mass_kg")
    if model_mass is None or abs(float(model_mass)-required["body_mass_kg"]) > MASS_TOLERANCE_KG:
        problems.append(f"MODEL_HUMAN_MASS={model_mass}!=REQUIRED={required['body_mass_kg']}")
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
