"""Re-run I3 controls against one immutable project source module from Git.

Returns the real unittest exit code. Only explicitly scoped project modules
may be loaded; no external corpus, checkout or nested Codex process is used.
"""
import argparse
import importlib
import subprocess
import sys
import types
import unittest

MODULES = {"biosyngen.multibody", "biosyngen.i2.squat", "biosyngen.i2.bench", "biosyngen.i2.power_clean"}
BASELINE = "3322d89499050eea0004f9e8882666e2a40d5a9a"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--module", choices=sorted(MODULES), required=True)
    parser.add_argument("--tests", required=True)
    parser.add_argument("--ref", default=BASELINE)
    args = parser.parse_args()
    if not args.tests.startswith("tests.test_i3_"):
        parser.error("only scoped I3 test modules are allowed")
    path = args.module.replace(".", "/") + ".py"
    revision = subprocess.check_output(["git", "rev-parse", "--verify", args.ref + "^{commit}"]).decode().strip()
    source = subprocess.check_output(["git", "show", revision + ":" + path])
    importlib.import_module(args.module.rsplit(".", 1)[0])
    module = types.ModuleType(args.module)
    module.__package__ = args.module.rsplit(".", 1)[0]
    module.__file__ = "git:" + revision + ":" + path
    sys.modules[args.module] = module
    exec(compile(source, module.__file__, "exec"), module.__dict__)
    print("FROZEN_SOURCE", revision, path, flush=True)
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromName(args.tests))
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
