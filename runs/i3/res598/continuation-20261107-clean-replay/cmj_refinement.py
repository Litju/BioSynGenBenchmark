"""Bounded CMJ angular refinement; independently evaluated declared numerical budget."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
from biosyngen.i2.jumps import build_jump
from biosyngen.i2.export import export_submission
from biosyngen_benchmark.i2_evaluator import evaluate_i2
results=[]
for dt in (.001,.0005,.00025,.00003125):
    trial=build_jump('CMJ').run(dt)
    submission=export_submission(trial)
    submission['claims'].append('ANGULAR_MOMENTUM_CLOSURE')
    result=evaluate_i2(submission)['findings']['ANGULAR_MOMENTUM_CLOSURE']
    row=dict(dt_s=dt,termination=trial.termination,angular=result)
    results.append(row)
    print(json.dumps(row),flush=True)
receipt=dict(source_sha=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
             evaluator_sha256=hashlib.sha256(Path('biosyngen_benchmark/i2_evaluator.py').read_bytes()).hexdigest(),
             scope='NUMERICAL_VERIFICATION_ONLY; one CMJ fixture, no independent final-SHA acceptance',cases=results)
Path(__file__).with_suffix('.json').write_text(json.dumps(receipt,indent=2)+'\n',encoding='utf-8')
assert all(b['angular']['max_prefix_residual_kg_m2_s']<a['angular']['max_prefix_residual_kg_m2_s'] for a,b in zip(results,results[1:]))
assert results[-1]['angular']['outcome']=='PASS'
