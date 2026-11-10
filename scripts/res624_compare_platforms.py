"""RES-624 Gate I: compare Windows and Linux qualification outputs.

    python -m scripts.res624_compare_platforms --root <dir with human-v2-receiving-*/> --out comparison.json

Bitwise equality is NOT required: the contact solver, BLAS and libm differ across platforms and a rigid-body
contact simulation amplifies last-bit differences. Qualification is a categorical decision, so the verdicts
(final verdict, per-case outcome and failure categories, adversarial refusals, numerical-verification checks)
must be identical. The measured quantities must agree within tolerances far below the acceptance margins:

  * bar displacement in hand           |delta| <= 1 mm        (acceptance bound is one 14 mm shaft radius)
  * hand support / floor ratios         |delta| <= 0.01        (1 % of bar / system weight)
  * energy-ledger relative residual     |delta| <= 0.005      (acceptance bound 1 % of gross flux)
  * energy-ledger observed order        both within 1 +/- 0.1 (first-order integrator residual)
  * bar trajectory refinement           both platforms: differences < 1 mm and decreasing with dt. The ratio of
                                        two ~1e-5 m differences is noise-amplified by contact chatter across
                                        platforms (hosted measurement 1.88 vs 2.37), so it is reported, not gated.
  * physical negative controls (release, halved arm capacity) end in chaotic impact: only the preregistered
    expectation (and its named failure category) is compared, not incidental secondary categories.
  * exact Git HEAD, MuJoCo, SciPy and NumPy versions must match.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

TOL = dict(displacement_m=1e-3, ratio=1e-2, energy_relative=5e-3, trajectory_m=1e-3)


def load(root, name, platform):
    return json.loads((Path(root)/f"human-v2-receiving-{platform}"/name).read_text(encoding="utf-8"))


def compare(root):
    plats = sorted(p.name.removeprefix("human-v2-receiving-") for p in Path(root).glob("human-v2-receiving-*"))
    if len(plats) != 2:
        return dict(outcome="PLATFORM_COMPARISON_FAILED", reason=f"expected two platform artifacts, found {plats}")
    a, b = plats
    problems, deltas = [], {}
    qa, qb = (load(root, "qualification/receiving_qualification.json", p) for p in plats)
    na, nb = (load(root, "gateF/numerical_verification.json", p) for p in plats)
    for key in ("head", "mujoco", "scipy", "numpy"):
        if qa["receipt"][key] != qb["receipt"][key]:
            problems.append(f"receipt {key} differs: {qa['receipt'][key]} vs {qb['receipt'][key]}")
    if qa["final_verdict"] != qb["final_verdict"]:
        problems.append("final verdict differs")
    for name in sorted(set(qa["cases"]) | set(qb["cases"])):
        ca, cb = qa["cases"].get(name), qb["cases"].get(name)
        if ca is None or cb is None:
            problems.append(f"case {name} missing on one platform")
            continue
        negative = ca["expectation"] != "QUALIFIED"
        for k in (("static_verdict", "expectation_met") if negative else ("static_verdict", "outcome", "all_failures", "expectation_met")):
            if ca[k] != cb[k]:
                problems.append(f"{name}.{k}: {ca[k]} vs {cb[k]}")
        ea, eb = ca["evidence"], cb["evidence"]
        for k, tol in (("bar_displacement_in_hand_m", TOL["displacement_m"]), ("hand_support_ratio_mean", TOL["ratio"]),
                       ("floor_vertical_over_weight_mean", TOL["ratio"])):
            if k in ea and k in eb:
                d = abs(ea[k]-eb[k])
                deltas[f"{name}.{k}"] = d
                if d > tol and ca["expectation"] == "QUALIFIED":
                    problems.append(f"{name}.{k} differs by {d:.3e} > {tol}")
    for k in qa["adversarial"]:
        if qa["adversarial"][k]["observed"] != qb["adversarial"][k]["observed"]:
            problems.append(f"adversarial {k} observed differs")
    if na["checks"] != nb["checks"]:
        problems.append("numerical-verification checks differ")
    d = abs(na["energy"]["relative_to_gross_flux"]-nb["energy"]["relative_to_gross_flux"])
    deltas["energy"] = d
    if d > TOL["energy_relative"]:
        problems.append(f"energy differs by {d:.3e}")
    for plat, n in ((a, na), (b, nb)):
        order = n["energy_refinement"]["observed_order"]
        if not all(o is not None and abs(o-1) <= .1 for o in order):
            problems.append(f"{plat} energy-ledger observed order {order} not 1 +/- 0.1")
        t = n["timestep_refinement"]
        d1, d2 = t["bar_traj_max_diff_1ms_vs_0p5ms_m"], t["bar_traj_max_diff_0p5ms_vs_0p25ms_m"]
        if not (d2 < d1 < TOL["trajectory_m"]):
            problems.append(f"{plat} trajectory refinement not converging below 1 mm: {d1:.3e}, {d2:.3e}")
    deltas["refinement_ratio_informational"] = abs(na["timestep_refinement"]["observed_ratio"]-nb["timestep_refinement"]["observed_ratio"])
    return dict(outcome="PLATFORMS_CONSISTENT" if not problems else "PLATFORM_COMPARISON_FAILED", platforms=plats,
                tolerances=TOL, max_deltas=deltas, problems=problems,
                bitwise_equal_numerical_fingerprint=na["fingerprint"]["sha256"] == nb["fingerprint"]["sha256"],
                note="categorical verdicts must match exactly; numeric fields within the declared tolerances; bitwise equality not required")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    r = compare(a.root)
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(r, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    print(json.dumps(r, indent=2))
    return 0 if r["outcome"] == "PLATFORMS_CONSISTENT" else 1


if __name__ == "__main__":
    sys.exit(main())
