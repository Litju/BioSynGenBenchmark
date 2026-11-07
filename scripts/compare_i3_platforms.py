"""Independently check CI platform values against exact analytical references.

Recomputes errors and triangle bounds; submitted expected values/tolerances are
not trusted. Numeric budgets are regression acceptance ceilings, not alleged
bounds for the multibody integrator. Bounds apply only to these rational cases.
"""
import argparse
from fractions import Fraction
import json
import math
from pathlib import Path
from biosyngen_benchmark.i3_analytic_contract import EXPECTED, rational_record


def compare(reports):
    if len(reports)!=2 or {r.get('platform') for r in reports}!={'win32','linux'}:
        raise ValueError("one Windows and one Linux report required")
    if reports[0].get('source_commit')!=reports[1].get('source_commit'):
        raise ValueError("platform reports must refer to the same source commit")
    rows={}
    for name,(expected,budget) in EXPECTED.items():
        values=[];errors=[]
        for report in reports:
            if report.get('schema')!='I3_ANALYTIC_PLATFORM_V1' or set(report['metrics'])!=set(EXPECTED):
                raise ValueError("unknown or incomplete numeric contract")
            value=report['metrics'][name]['value']
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                raise ValueError("nonfinite metric")
            exact=Fraction.from_float(float(value));error=abs(exact-expected)
            if error>budget:
                raise ValueError(f"{name} exceeds independently fixed analytic budget")
            values.append(exact);errors.append(error)
        difference=abs(values[0]-values[1]);bound=sum(errors)
        if difference>bound:
            raise ValueError("exact triangle bound violated")
        rows[name]={"cross_platform_difference_exact":rational_record(difference),
                    "certified_triangle_bound_exact":rational_record(bound),"regression_budget":rational_record(budget)}
    return {"passed":True,"metrics":rows,"scope":"specified exact-rational fixtures only"}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root",required=True)
    parser.add_argument("--out",required=True)
    args=parser.parse_args(argv)
    paths=sorted(Path(args.root).glob("**/numeric-probe.json"))
    result=compare([json.loads(p.read_text(encoding='utf-8')) for p in paths])
    result['sources']=[str(p) for p in paths]
    path=Path(args.out);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result,indent=2,sort_keys=True)+"\n",encoding='utf-8',newline='\n')
    print("PASS: independent Windows/Linux analytical comparison")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
