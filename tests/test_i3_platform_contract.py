"""RES-506 hostile hosted-receipt controls; no generator reference imports."""
import unittest
from biosyngen_benchmark.i3_analytic_contract import EXPECTED
from scripts.compare_i3_platforms import compare


class PlatformContractTests(unittest.TestCase):
    def references(self):
        return [{'platform':platform,'schema':'I3_ANALYTIC_PLATFORM_V1',
                 'metrics':{name:{'value':float(expected)} for name,(expected,_) in EXPECTED.items()}}
                for platform in ('win32','linux')]

    def test_exact_references_pass_and_values_get_exact_error_bounds(self):
        self.assertTrue(compare(self.references())['passed'])

    def test_bad_value_cannot_buy_a_pass_by_declaring_a_tolerance(self):
        reports=self.references()
        reports[0]['metrics']['support_left_n'].update(value=10000,budget={'numerator':10**100,'denominator':1})
        with self.assertRaises(ValueError): compare(reports)

    def test_missing_platform_nonfinite_or_missing_metric_refuses(self):
        for mutation in ('same_platform','nonfinite','missing'):
            reports=self.references()
            if mutation=='same_platform': reports[0]['platform']='linux'
            elif mutation=='nonfinite': reports[0]['metrics']['support_left_n']['value']=float('nan')
            else: del reports[0]['metrics']['support_left_n']
            with self.subTest(mutation=mutation),self.assertRaises(ValueError): compare(reports)
