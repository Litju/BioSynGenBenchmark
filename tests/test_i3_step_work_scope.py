"""RES-507 expose the work quadrature deficit during within-step capture."""
import unittest
from biosyngen.multibody import (
    ContactKind, ContactSpec, ParameterProvenance, ParameterStatus,
    PlanarMultibodySystem, RigidBodyDefinition, RigidBodyState,
)


class StepWorkScopeTests(unittest.TestCase):
    def test_capture_endpoint_quadrature_is_not_exact_continuous_friction_work(self):
        evidence=lambda name:ParameterProvenance(name,ParameterStatus.MANUFACTURED_TEST_FIXTURE,"RES-507 analytical capture",0)
        body=RigidBodyDefinition("block",2.3,0.2,evidence("mass"),evidence("inertia"))
        contact=ContactSpec("floor","block",(0,0),(0,1),0.37,0,ContactKind.BAR_FLOOR,surface_point_m=(0,0))
        system=PlanarMultibodySystem((body,),(),(contact,),gravity_m_s2=9.80665,clock_id="CAPTURE_WORK")
        state=system.make_state({"block":RigidBodyState((0,0),0,(0.01,0))},active_contacts=("floor",))
        result=system.advance(state,0.02)
        expected_physical_work=-2.3*0.01**2/2
        self.assertAlmostEqual(system.kinetic_energy(result.state)-system.kinetic_energy(state),expected_physical_work,places=12)
        self.assertEqual(result.contact_work_estimate_j,0.0)
        self.assertGreater(abs(result.contact_work_estimate_j-expected_physical_work),1e-5)
        self.assertIn("quadrature",result.contact_work_definition)
