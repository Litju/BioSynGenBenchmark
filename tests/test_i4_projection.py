"""RES-517 contact-law oracles, independent of controller completion counts."""
import json
import math
from pathlib import Path
import unittest

from biosyngen.multibody import (
    AppliedWrench, ConstraintSolveError, ContactKind, ContactMode, ContactNormalFrame, ContactSpec, ContactState, MultibodyState, ForcePlateFrame, PlanarJoint, PlanarMultibodySystem,
    ParameterProvenance, ParameterStatus, RigidBodyDefinition, RigidBodyState,
)
from biosyngen.i2.human import Posture
from biosyngen.i2.olympic import PullProtocol, build_pull

def restore_state(value):
    return MultibodyState(value["time_s"],
        {k: RigidBodyState(**v) for k, v in value["bodies"].items()},
        {k: ContactState(**{**v, "mode": ContactMode(v["mode"])}) for k, v in value["contacts"].items()},
        value["activations"])



def body(name, mass=2.0, inertia=0.3):
    def evidence(suffix):
        return ParameterProvenance(name+suffix, ParameterStatus.MANUFACTURED_TEST_FIXTURE,
                                   "RES-517 analytical contact fixture", 0.0)
    return RigidBodyDefinition(name,mass,inertia,evidence(":m"),evidence(":I"))


def block(mu=0.2):
    contact = ContactSpec("floor","block",(0,0),(0,1),mu,0,ContactKind.BAR_FLOOR,surface_point_m=(0,0))
    return PlanarMultibodySystem((body("block"),),(),(contact,),gravity_m_s2=10,clock_id="ANALYTIC")


class ProjectionContactTests(unittest.TestCase):
    def test_second_historical_witness_and_projection_joint_impulse_closure(self):
        saved=json.loads((Path(__file__).parent/"fixtures/res522_contact_second.json").read_text())
        self.assertEqual(saved["witnesses"][0]["excess_n_s"],0.0001479106384707496)
        for witness in saved["witnesses"]:
            system=build_pull(PullProtocol(pull_s=1.2,extension=Posture(-0.2,-0.05,0.02,0,0))).model.system
            original=system._project_velocities
            checks=[]

            def project(state, modes, **kwargs):
                original_kkt=system._solve_kkt
                solved=None
                def kkt(*args, **kw):
                    nonlocal solved
                    result=original_kkt(*args, **kw)
                    solved=(args[1],result[1])
                    return result
                system._solve_kkt=kkt
                try:
                    after=original(state,modes,**kwargs)
                finally:
                    system._solve_kkt=original_kkt
                impulse=[0.]*system.dof_count
                for row, amount in zip(*solved):
                    if row.kind=="JOINT":
                        for i,value in enumerate(row.force_column):
                            impulse[i]+=value*amount
                for cid,(n,t) in kwargs["contact_ledger"].items():
                    data=system._contact_data(cid,state)
                    for i in range(system.dof_count):
                        impulse[i]+=n*data.normal_jacobian[i]+t*data.tangential_jacobian[i]
                v0,v1=system._velocity_vector(state),system._velocity_vector(after)
                for m,a,b,p in zip(system._mass_diagonal(),v0,v1,impulse):
                    self.assertAlmostEqual(m*(b-a),p,delta=1e-8)
                work=sum(p*(a+b)/2 for p,a,b in zip(impulse,v0,v1))
                self.assertAlmostEqual(system.kinetic_energy(after)-system.kinetic_energy(state),work,delta=1e-8)
                checks.append(work)
                return after

            system._project_velocities=project
            step=system.advance(restore_state(witness["before"]),witness["dt_s"],excitations=witness["excitations"],
                                external_wrenches=tuple(AppliedWrench(**w) for w in witness["wrenches"]))
            self.assertTrue(checks)
            self.assertLess(max(abs(r.velocity) for r in system._joint_rows(step.state)),1e-9)
            for side in ("left","right"):
                ids=(side+"_heel",side+"_toe")
                n,t=(sum(ledger.get(c,(0,0))[axis] for ledger in
                         (step.contact_impulses_n_s,step.projection_impulses_n_s) for c in ids) for axis in (0,1))
                self.assertGreaterEqual(n,-1e-8)
                self.assertLessEqual(abs(t),0.8*n+1e-8)

    def test_redundant_projection_refuses_instead_of_inventing_force_shares(self):
        contacts=tuple(ContactSpec(n,"block",(0,0),(0,1),0.2,0,ContactKind.BAR_FLOOR,surface_point_m=(0,0)) for n in ("a","b"))
        system=PlanarMultibodySystem((body("block"),),(),contacts,gravity_m_s2=10,clock_id="SINGULAR")
        state=system.make_state({"block":RigidBodyState((0,0),0)},active_contacts=("a","b"))
        with self.assertRaises(ConstraintSolveError):
            system._project_velocities(state,state.contacts)

    def test_rotating_internal_contact_preserves_external_impulse_balance(self):
        pin=ContactSpec("pin","plane",(0,0),(0,1),2,0,ContactKind.RACK,surface_point_m=(0,0))
        seat=ContactSpec("seat","block",(0,0),(0,1),0,0,ContactKind.CATCH,
                         body_b_id="plane",local_point_b_m=(0,0),normal_frame=ContactNormalFrame.BODY_B)
        errors=[]
        for dt in (0.002,0.001,0.0005):
            system=PlanarMultibodySystem((body("plane",5,1),body("block",1,0.1)),(),(pin,seat),gravity_m_s2=10,clock_id="ROTATING")
            state=system.make_state({"plane":RigidBodyState((0,0),0,(0,0),0.5),
                                     "block":RigidBodyState((0.3,0),0,(0,0.15))},active_contacts=("pin","seat"))
            initial_energy=system.kinetic_energy(state)+system.potential_energy(state)
            for _ in range(round(0.05/dt)):
                before=state
                step=system.advance(state,dt)
                self.assertEqual(step.elapsed_s,dt)
                state=step.state
                for axis in (0,1):
                    change=sum(b.mass_kg*(state.bodies[b.body_id].velocity_m_s[axis]-before.bodies[b.body_id].velocity_m_s[axis]) for b in system.bodies)
                    external=-6*10*dt if axis==1 else 0.
                    for ledger,frame in ((step.contact_impulses_n_s,before),(step.projection_impulses_n_s,state)):
                        n,t=ledger.get("pin",(0,0))
                        k=system.contact_kinematics(frame,"pin")
                        external+=n*k.normal_world[axis]+t*k.tangent_world[axis]
                    self.assertAlmostEqual(change,external,places=11)
                total=system._transport_contact_impulses(step.contact_impulses_n_s,before,state)
                pn,pt=total.get("seat",(0,0))
                dn,dtan=step.projection_impulses_n_s.get("seat",(0,0))
                self.assertGreaterEqual(pn+dn,-1e-8)
                self.assertLessEqual(abs(pt+dtan),1e-8)  # frictionless moving normal
            errors.append(abs(system.kinetic_energy(state)+system.potential_energy(state)-initial_energy))
        # Smooth maintained-contact fixture, not a global nonsmooth order claim.
        self.assertGreater(errors[0],errors[1])
        self.assertGreater(errors[1],errors[2])

    def test_grazing_event_preserves_joint_velocity_constraints(self):
        saved=json.loads((Path(__file__).parent/"fixtures/i4_grazing_projection.json").read_text())
        system=build_pull(PullProtocol(pull_s=0.8)).model.system
        step=system.advance(restore_state(saved["before"]),saved["dt_s"],excitations=saved["excitations"],
                            external_wrenches=tuple(AppliedWrench(**w) for w in saved["wrenches"]))
        self.assertLess(max(abs(r.velocity) for r in system._joint_rows(step.state)),1e-9)

    def test_force_driven_block_at_and_across_friction_boundary(self):
        for force in (3.9,4.0,4.1,-4.1):
            with self.subTest(force=force):
                system=block()
                state=system.make_state({"block":RigidBodyState((0,0),0)},active_contacts=("floor",))
                step=system.advance(state,0.1,external_wrenches=(AppliedWrench("block","push",(force,0)),))
                expected_t=-max(-0.4,min(0.4,force*0.1))
                n,t=step.contact_impulses_n_s["floor"]
                dn,dt=step.projection_impulses_n_s.get("floor",(0,0))
                self.assertAlmostEqual(n+dn,2.,places=12)
                self.assertAlmostEqual(t+dt,expected_t,places=12)
                self.assertAlmostEqual(step.state.bodies["block"].velocity_m_s[0],(force*0.1+expected_t)/2,places=12)

    def test_upward_force_releases_support(self):
        system=block()
        state=system.make_state({"block":RigidBodyState((0,0),0)},active_contacts=("floor",))
        step=system.advance(state,0.1,external_wrenches=(AppliedWrench("block","lift",(0,25)),))
        self.assertIs(step.state.contacts["floor"].mode,ContactMode.OPEN)
        self.assertAlmostEqual(step.state.bodies["block"].velocity_m_s[1],0.25,places=12)
        self.assertAlmostEqual(sum(v[0] for v in step.contact_impulses_n_s.values())+sum(v[0] for v in step.projection_impulses_n_s.values()),0.,places=12)

    def test_joint_coupled_mass_matches_reduced_analytical_system(self):
        floor=ContactSpec("floor","a",(0,0),(0,1),0.2,0,ContactKind.BAR_FLOOR,surface_point_m=(0,0))
        joint=PlanarJoint("coupler","a","b",(0,0),(0,0))
        system=PlanarMultibodySystem((body("a",2),body("b",3)),(joint,),(floor,),gravity_m_s2=10,clock_id="JOINT")
        state=system.make_state({n:RigidBodyState((0,0),0) for n in ("a","b")},active_contacts=("floor",))
        step=system.advance(state,0.1,external_wrenches=(AppliedWrench("b","push",(12,0)),))
        for value in step.state.bodies.values():
            self.assertAlmostEqual(value.velocity_m_s[0],0.04,places=12)
            self.assertAlmostEqual(value.velocity_m_s[1],0.,places=12)
        n,t=step.contact_impulses_n_s["floor"]
        dn,dt=step.projection_impulses_n_s.get("floor",(0,0))
        self.assertAlmostEqual(n+dn,5.,places=12)
        self.assertAlmostEqual(t+dt,-1.,places=12)

    def test_two_patches_are_invariant_to_order_and_mirror(self):
        answers=[]
        for reverse in (False,True):
            for sign in (-1,1):
                contacts=tuple(ContactSpec(side+end,side,(x,-0.05),(0,1),0.4,0,ContactKind.FOOT,
                                           surface_point_m=(0,0),friction_group=side,
                                           side=side.upper(),measurement_channel_id="FZ_"+side.upper(),
                                           plate_frame=ForcePlateFrame(side,side.upper(),"FZ_"+side.upper(),(0,0),(1,0),(0,1)))
                               for side in ("left","right") for end,x in (("heel",-0.1),("toe",0.1)))
                bodies=(body("left"),body("right"))
                system=PlanarMultibodySystem(bodies[::-1] if reverse else bodies,(),contacts[::-1] if reverse else contacts,
                                             gravity_m_s2=10,clock_id="TWO_PATCHES")
                state=system.make_state({s:RigidBodyState((x,0.05),0) for s,x in (("left",-0.4),("right",0.4))},
                                        active_contacts=tuple(c.contact_id for c in contacts))
                for _ in range(20):
                    step=system.advance(state,0.001,external_wrenches=tuple(AppliedWrench(s,"push",(sign*8.1,0)) for s in ("left","right")))
                    for side in ("left","right"):
                        ids=(side+"heel",side+"toe")
                        n=sum(step.contact_impulses_n_s.get(c,(0,0))[0]+step.projection_impulses_n_s.get(c,(0,0))[0] for c in ids)
                        t=sum(step.contact_impulses_n_s.get(c,(0,0))[1]+step.projection_impulses_n_s.get(c,(0,0))[1] for c in ids)
                        self.assertLessEqual(abs(t),0.4*n+1e-8)
                    state=step.state
                answers.append(sign*state.bodies["left"].velocity_m_s[0])
        for answer in answers:
            self.assertAlmostEqual(answer,0.001,places=11)

    def test_rotating_frame_transport_preserves_world_impulse(self):
        contact=ContactSpec("seat","a",(0,0),(0,1),0.5,0,ContactKind.CATCH,
                            body_b_id="b",local_point_b_m=(0,0),normal_frame=ContactNormalFrame.BODY_B)
        system=PlanarMultibodySystem((body("a"),body("b")),(),(contact,),gravity_m_s2=10,clock_id="ROTATING")
        old=system.make_state({n:RigidBodyState((0,0),0) for n in ("a","b")})
        new=system.make_state({"a":RigidBodyState((0,0),0),"b":RigidBodyState((0,0),math.pi/6)})
        n,t=system._transport_contact_impulses({"seat":(1.,0.1)},old,new)["seat"]
        self.assertAlmostEqual(-n*0.5+t*math.sqrt(3)/2,0.1,places=12)
        self.assertAlmostEqual(n*math.sqrt(3)/2+t*0.5,1.,places=12)

    def test_simultaneous_restitution_impact_and_energy(self):
        contacts=tuple(ContactSpec(n,n,(0,0),(0,1),0.2,0.3,ContactKind.BAR_FLOOR,surface_point_m=(0,0)) for n in ("a","b"))
        system=PlanarMultibodySystem((body("a"),body("b",3)),(),contacts,gravity_m_s2=10,clock_id="IMPACTS")
        state=system.make_state({n:RigidBodyState((0,0),0,(0,-2)) for n in ("a","b")})
        after,event=system.resolve_impact(state,("a","b"))
        self.assertAlmostEqual(event.normal_impulses_n_s["a"],5.2,places=12)
        self.assertAlmostEqual(event.normal_impulses_n_s["b"],7.8,places=12)
        for value in after.bodies.values():
            self.assertAlmostEqual(value.velocity_m_s[1],0.6,places=12)
        self.assertAlmostEqual(event.contact_work_j,5*(0.6**2-2**2)/2,places=12)

    def test_saved_full_step_combined_cone(self):
        source=Path(__file__).parent/"fixtures/res522_contact_first.json"
        saved=json.loads(source.read_text())
        system=build_pull(PullProtocol(pull_s=0.8,extension=Posture(-0.2,-0.05,0.02,0,0))).model.system
        before=restore_state(saved["before"])
        step=system.advance(before,saved["dt_s"],excitations=saved["excitations"],
                            external_wrenches=tuple(AppliedWrench(**w) for w in saved["external_wrenches"]))
        for side in ("left","right"):
            ids=(side+"_heel",side+"_toe")
            n=sum(step.contact_impulses_n_s.get(c,(0,0))[0]+step.projection_impulses_n_s.get(c,(0,0))[0] for c in ids)
            t=sum(step.contact_impulses_n_s.get(c,(0,0))[1]+step.projection_impulses_n_s.get(c,(0,0))[1] for c in ids)
            self.assertGreaterEqual(n,-1e-8)
            self.assertLessEqual(abs(t),0.8*n+1e-8)

    def test_accumulated_cone_edge_must_slide_not_force_stick(self):
        system=block()
        state=system.make_state({"block":RigidBodyState((0,0),0,(0.3,0))},active_contacts=("floor",))
        ledger={}
        after=system._project_velocities(state,state.contacts,contact_ledger=ledger,
                                        accumulated_impulses={"floor":(1.0,-0.2)},impulse_state=state)
        self.assertAlmostEqual(after.bodies["block"].velocity_m_s[0],0.3,places=12)
        self.assertIs(after.contacts["floor"].mode,ContactMode.SLIDE)
        n,t=ledger.get("floor",(0,0))
        self.assertLessEqual(abs(-0.2+t),0.2*(1+n)+1e-8)

    def test_static_friction_uses_existing_normal_budget(self):
        system=block()
        state=system.make_state({"block":RigidBodyState((0,0),0,(0.03,0))},active_contacts=("floor",))
        ledger={}
        after=system._project_velocities(state,state.contacts,contact_ledger=ledger,
                                        accumulated_impulses={"floor":(1.,0.)},impulse_state=state)
        self.assertAlmostEqual(after.bodies["block"].velocity_m_s[0],0.,places=12)
        self.assertAlmostEqual(ledger["floor"][1],-0.06,places=12)
        self.assertIs(after.contacts["floor"].mode,ContactMode.STICK)

    def test_free_body_needs_no_projection(self):
        system=PlanarMultibodySystem((body("free"),),(),(),gravity_m_s2=10,clock_id="FREE")
        state=system.make_state({"free":RigidBodyState((0,1),0,(1,2),3)})
        ledger={}
        self.assertEqual(system._project_velocities(state,state.contacts,contact_ledger=ledger),state)
        self.assertEqual(ledger,{})


if __name__=="__main__":
    unittest.main()
