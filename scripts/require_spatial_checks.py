"""Mandatory spatial-backend CI gate: missing MuJoCo, failures or skips fail closed.

    python -m scripts.require_spatial_checks [extra test modules ...]
"""
import sys
import unittest

import mujoco
import numpy
import scipy

MODULES = ["tests.test_res610_spatial"]


def main(argv=None):
    modules = MODULES + list(argv if argv is not None else sys.argv[1:])
    print(sys.version, "MuJoCo", mujoco.__version__, "SciPy", scipy.__version__, "NumPy", numpy.__version__, flush=True)
    suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(name) for name in modules)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() and result.testsRun > 0 and not result.skipped else 1


if __name__ == "__main__":
    raise SystemExit(main())
