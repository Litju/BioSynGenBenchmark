from dataclasses import replace
from biosyngen.i2.drop_jump import build_drop_jump_candidate
from biosyngen.i2.human import Posture
for hip in (1.2,1.6,2.0):
 s=build_drop_jump_candidate(duration_s=3)
 s.controller.stages=[replace(x,reference=Posture(.35,-.9,hip)) if x.name=='FLIGHT' else x for x in s.controller.stages]
 t=s.run(.0004)
 print(hip,t.termination,t.frames[-1].t_end_s,s.model.system.linear_momentum(t.frames[-1].state),t.control_commands,flush=True)
