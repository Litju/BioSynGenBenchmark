"""Mandatory CI solver qualification: import/install failure or skips fail closed."""
import sys
import unittest
import scipy
import numpy


def main():
    print(sys.version, "SciPy", scipy.__version__, "NumPy", numpy.__version__, flush=True)
    suite=unittest.defaultTestLoader.loadTestsFromName("tests.test_i1_solver_crosscheck")
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() and result.testsRun==2 and not result.skipped else 1


if __name__=="__main__":
    raise SystemExit(main())
