"""RES-507 curated adversarial controller grid, without gains/posture fitting.

Clean pull duration 0.8/1.0/1.2 s × ankle target 0.05/-0.2/-0.4 rad.
Plus the historical weak/heavy CMJ hypothesis (90 kg, 1.88 m, unscaled
capacities). Failure is a recorded result, not a reason to tune the case away.
"""
import argparse
from pathlib import Path
import json
from biosyngen.i2.human import Anthropometry, Posture
from biosyngen.i2.jumps import build_jump
from biosyngen.i2.olympic import PullProtocol, build_pull
from scripts.qualify_i3_artifacts import CEILINGS, inspect_case


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",default="runs/i3/res507/controller-stress")
    args=parser.parse_args(argv)
    project=Path(__file__).resolve().parents[1]
    out=(project/args.out).resolve()
    if not out.is_relative_to(project) or out==project or (out.exists() and any(out.iterdir())):
        parser.error("use a fresh project subdirectory")
    out.mkdir(parents=True,exist_ok=True)
    cases={}
    definitions=[("CMJ_WEAK_HEAVY",lambda:build_jump("CMJ",anthropometry=Anthropometry(body_mass_kg=90,stature_m=1.88)))]
    definitions += [(f"PULL_T{duration}_A{ankle}",lambda d=duration,a=ankle:build_pull(
        PullProtocol(pull_s=d,extension=Posture(a,-0.05,0.02,0,0))))
        for duration in (0.8,1.0,1.2) for ankle in (0.05,-0.2,-0.4)]
    for name,builder in definitions:
        print("STRESS",name,flush=True)
        try:
            cases[name]=inspect_case(name,builder(),out)
        except (ValueError,ArithmeticError) as exc:
            cases[name]={"model_task_completed":False,"termination":"CASE_REFUSED_OR_FAILED","error":str(exc)}
    report={"issue":"RES-507","claim_ceilings":CEILINGS,"cases":cases,
            "completed":{"n":len(cases),"completed":sum(c['model_task_completed'] for c in cases.values())},
            "sampling":"curated adversarial hypotheses informed by source review; not independent random population sampling",
            "retuning":False}
    (out/"stress.json").write_text(json.dumps(report,indent=2,sort_keys=True)+"\n",encoding="utf-8",newline="\n")
    print(json.dumps(report["completed"]))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
