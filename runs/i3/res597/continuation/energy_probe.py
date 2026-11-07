from biosyngen.i2.drop_jump import build_drop_jump_candidate
for dt in (.0004,.0002,.0001):
 s=build_drop_jump_candidate(duration_s=.5)
 t=s.run(dt)
 sys=s.model.system
 end=t.frames[-1].state
 stored=0
 for cid in end.contacts:
  kin=sys.contact_kinematics(end,cid)
  depth=max(0,-kin.gap_m)
  stored+=s.contact_law.stiffness*depth**(s.contact_law.exponent+1)/(s.contact_law.exponent+1)
 work=sum(f.joint_power_w*(f.t_end_s-f.t_start_s) for f in t.frames)
 loss=sum(sum(f.contact_dissipated_power_w.values())*(f.t_end_s-f.t_start_s) for f in t.frames)
 e0=sys.kinetic_energy(t.initial_state)+sys.potential_energy(t.initial_state)
 ef=sys.kinetic_energy(end)+sys.potential_energy(end)+stored
 print(dt,ef-e0-work+loss,loss,work,flush=True)
