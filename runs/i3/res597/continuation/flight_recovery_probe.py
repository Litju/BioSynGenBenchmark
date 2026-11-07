from dataclasses import replace
from biosyngen.i2.drop_jump import build_drop_jump_candidate
from biosyngen.i2.human import Posture, balanced_posture
from biosyngen.i2.simulate import ControlStage
from biosyngen.i2.jumps import _gains, feet_released, foot_touched
for effort in (.3,.45,.6):
 s=build_drop_jump_candidate(duration_s=3)
 m=s.model
 landing=ControlStage('FINAL_LANDING',balanced_posture(m,.35,-.9,.04),_gains(150,1500,1500),_gains(30,250,250),True,lambda m,s,t:t>.35,transition_s=.15,balance_gain_n_m_per_m=3000,balance_damping_n_m_s_per_m=1200)
 stand=ControlStage('STAND',balanced_posture(m,.08,-.12,.04),_gains(150,800,800),_gains(100,100,100),True,transition_s=.5,balance_gain_n_m_per_m=3000,balance_damping_n_m_s_per_m=1200)
 s.controller.stages=[replace(x,reference=Posture(-.6,-.02,0),effort_cap={'ankle':effort},balance_gain_n_m_per_m=3000,balance_damping_n_m_s_per_m=1200) if x.name=='REACTIVE_PROPULSION' else replace(x,reference=Posture(-.2,-.3,.3)) if x.name=='FLIGHT' else landing if x.name=='FINAL_LANDING' else x for x in s.controller.stages]+[stand]
 s.controller.balance_reference_x_m=.04
 t=s.run(.0004)
 print(effort,t.termination,t.frames[-1].t_end_s,m.system.linear_momentum(t.frames[-1].state),t.control_commands,flush=True)
