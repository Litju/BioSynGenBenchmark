"""Temporary scoped I1 platform diagnostic; manufactured submissions only."""
from dataclasses import asdict
import json
import sys
from scripts.i1_benchmark_suite import build_cases
from biosyngen_benchmark.i1 import evaluate_i1, _quadrature, _windows


def main():
    cases,_=build_cases()
    for case in cases:
        if case.expected.get('WORK_ENERGY_LEDGER')!='PASS':
            continue
        finding=evaluate_i1(case.submission).finding('WORK_ENERGY_LEDGER')
        print('[I3-ENERGY]',json.dumps({'platform':sys.platform,'case':case.name,'finding':asdict(finding)},sort_keys=True),flush=True)
        if finding.outcome!='PASS':
            kin=case.submission['kinematics'];frames=kin['frames']
            windows,_=_windows(frames,kin.get('declared_event_boundaries',kin.get('declared_impulsive_events',[])))
            for indices in windows:
                ts=[frames[i]['t'] for i in indices]
                power=[frames[i]['joint_power_w']+frames[i]['internal_dissipation_power_w']
                       +sum(c['fx']*c['vx']+c['fz']*c['vz'] for c in frames[i]['contacts']) for i in indices]
                integral,error,stable,coarse=_quadrature(ts,power)
                if not stable:
                    print('[I3-ENERGY-WINDOW]',json.dumps({'case':case.name,'times':ts,'power':power,'integral':integral,'fine_delta':error,'coarse_delta':coarse}),flush=True)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
