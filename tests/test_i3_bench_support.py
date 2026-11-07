"""RES-507 rack-omission falsifier."""
import unittest
from biosyngen.i2.bench import BenchRefused, system_kinetics_from_plates

class BenchSupportTests(unittest.TestCase):
    def test_bench_whole_system_force_requires_explicit_rack_reaction(self):
        with self.assertRaisesRegex(BenchRefused, "rack"):
            system_kinetics_from_plates(100, 600)
        self.assertEqual(system_kinetics_from_plates(100, 600, rack_reaction_n=300), 1000)
        self.assertEqual(system_kinetics_from_plates(100, 600, rack_reaction_n=0), 700)
