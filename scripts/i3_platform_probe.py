"""Hosted-platform manufactured numerical probe; exact references are separate."""
import argparse
import json
from pathlib import Path
import sys
import subprocess
from fractions import Fraction
from biosyngen.multibody import (
    AppliedWrench, ContactKind, ContactSpec, ParameterProvenance, ParameterStatus,
    PlanarMultibodySystem, RigidBodyDefinition, RigidBodyState,
)
from biosyngen_benchmark.i3_analytic_contract import EXPECTED, rational_record


def body(mass,inertia):
    evidence=lambda name:ParameterProvenance(name,ParameterStatus.MANUFACTURED_TEST_FIXTURE,"I3 independent analytical CI contract",0)
    return RigidBodyDefinition("body",mass,inertia,evidence("mass"),evidence("inertia"))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",default="runs/i3/res506/local-probe/numeric-probe.json")
    args=parser.parse_args(argv)
    contacts=tuple(ContactSpec(name,"body",(x,-0.8),(0,1),0,0,ContactKind.BENCH,
                               surface_point_m=(x,0),friction_group="plane")
                   for name,x in (("left",-0.42),("right",0.42)))
    system=PlanarMultibodySystem((body(3.7,0.3),),(),contacts,gravity_m_s2=9.80665,clock_id="RATIONAL_CI")
    state=system.make_state({"body":RigidBodyState((0,0.8),0)},active_contacts=("left","right"))
    result=system.solve_accelerations(state,external_wrenches=(AppliedWrench("body","couple",moment_n_m=2.1),))
    values={"support_left_n":result.contact_wrenches['left'].normal_force_n,
            "support_right_n":result.contact_wrenches['right'].normal_force_n}
    contact=ContactSpec("floor","body",(0,0),(0,1),0.37,0,ContactKind.BAR_FLOOR,surface_point_m=(0,0))
    system=PlanarMultibodySystem((body(2.3,0.2),),(),(contact,),gravity_m_s2=9.80665,clock_id="RATIONAL_CI")
    state=system.make_state({"body":RigidBodyState((0,0),0,(0.01,0))},active_contacts=("floor",))
    step=system.advance(state,0.02)
    values.update(capture_velocity_m_s=step.state.bodies['body'].velocity_m_s[0],
                  capture_normal_impulse_n_s=step.contact_impulses_n_s['floor'][0],
                  capture_tangent_impulse_n_s=step.contact_impulses_n_s['floor'][1])
    contact=ContactSpec("floor","body",(0,0),(0,1),0,0.3,ContactKind.BAR_FLOOR,surface_point_m=(0,0))
    system=PlanarMultibodySystem((body(3.2,0.2),),(),(contact,),gravity_m_s2=9.80665,clock_id="RATIONAL_CI")
    state=system.make_state({"body":RigidBodyState((0,0),0,(0,-1.7))})
    after,event=system.resolve_impact(state,("floor",))
    values.update(impact_velocity_m_s=after.bodies['body'].velocity_m_s[1],
                  impact_impulse_n_s=event.normal_impulses_n_s['floor'],impact_work_j=event.contact_work_j)
    metrics={}
    passed=True
    for name,value in values.items():
        expected,budget=EXPECTED[name]
        error=abs(Fraction.from_float(float(value))-expected)
        metrics[name]={"value":value,"exact_expected":rational_record(expected),
                       "exact_forward_error":rational_record(error),"budget":rational_record(budget)}
        passed &= error<=budget
    report={"issue":"RES-506","schema":"I3_ANALYTIC_PLATFORM_V1","platform":sys.platform,"python":sys.version,
            "source_commit":subprocess.check_output(["git","rev-parse","HEAD"]).decode().strip(),"metrics":metrics,"passed":passed,
            "scope":"exact-rational fixture error only; not a bound for arbitrary exercise trajectories or empirical validation"}
    path=Path(args.out);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+"\n",encoding="utf-8",newline="\n")
    print(json.dumps({"passed":passed,"metrics":len(metrics),"platform":sys.platform}))
    return 0 if passed else 1


if __name__=="__main__":
    raise SystemExit(main())
