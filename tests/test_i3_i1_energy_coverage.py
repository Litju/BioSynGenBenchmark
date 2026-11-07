"""Independent hidden-between-window energy falsifiers; no generator imports."""
import unittest
from biosyngen_benchmark import controls
from biosyngen_benchmark.i1 import evaluate_i1


class EnergyWindowCoverageTests(unittest.TestCase):
    def test_one_sample_energy_spike_between_old_endpoints_is_rejected(self):
        kin=controls.tumbling_ballistic_control(duration_s=0.04)
        kin['frames'][10]['internal_elastic_energy_j']=1.0
        report=evaluate_i1({'candidate_id':'isolated_elastic_spike','claims':['WORK_ENERGY_LEDGER'],'kinematics':kin})
        self.assertEqual(report.outcome('WORK_ENERGY_LEDGER'),'REJECTED')

    def test_shifted_hidden_energy_spike_is_also_rejected(self):
        for index in (1,5,11,19,22,29):
            kin=controls.tumbling_ballistic_control(duration_s=0.06)
            kin['frames'][index]['internal_elastic_energy_j']=1.0
            report=evaluate_i1({'candidate_id':'shifted_elastic_spike','claims':['WORK_ENERGY_LEDGER'],'kinematics':kin})
            with self.subTest(index=index):
                self.assertEqual(report.outcome('WORK_ENERGY_LEDGER'),'REJECTED')
