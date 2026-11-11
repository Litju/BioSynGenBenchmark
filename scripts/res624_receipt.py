"""RES-624 Gate H: reproducible qualification-receipt fields bound to the exact code and platform.

``full_receipt()`` records the Git HEAD, worktree cleanliness (everything outside the evidence output
directories counts), hashes of every RES-624 source and of the compiled model/parameter set, library
versions, platform/CPU, solver configuration, controller and independent-evaluator identity.
A receipt with ``worktree_clean == False`` must never be used to certify a commit.
"""
from __future__ import annotations

import hashlib
import platform
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import scipy

SOURCES = (
    "biosyngen/spatial/model_v2.py", "biosyngen/spatial/plant_v2.py", "biosyngen/spatial/anthropometry.py",
    "biosyngen/spatial/grasp_fixture.py", "biosyngen/spatial/grasp_mechanics.py", "biosyngen/spatial/grasp_diagnostics.py",
    "biosyngen/spatial/receiving_grasp_v2.py", "biosyngen/spatial/receiving_v2.py",
    "biosyngen_benchmark/receiving_acceptance.py", "biosyngen_benchmark/spatial_oracle.py", "biosyngen_benchmark/rigid_body_check.py",
    "biosyngen_benchmark/grasp_contract.py",
    "scripts/res624_receiving_campaign.py", "scripts/res624_receiving_qualification.py", "scripts/res624_numerical_verification.py",
    "scripts/res624_adversarial_scenes.py", "scripts/res624_red_canary.py", "scripts/res624_anthropometric_diagnosis.py",
    "scripts/res618_fullbody_qualification.py",
)
EVALUATOR = ("biosyngen_benchmark/receiving_acceptance.py", "biosyngen_benchmark/spatial_oracle.py",
             "biosyngen_benchmark/rigid_body_check.py", "biosyngen_benchmark/grasp_contract.py")


def _git(*args):
    return subprocess.check_output(["git", *args], text=True).strip()


def _sha(paths):
    return {p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}


def solver_config(model):
    o = model.opt
    return dict(timestep_s=float(o.timestep), integrator=int(o.integrator), cone=int(o.cone), impratio=float(o.impratio),
                solver=int(o.solver), iterations=int(o.iterations), tolerance=float(o.tolerance), noslip_iterations=int(o.noslip_iterations),
                gravity=o.gravity.tolist())


def model_identity(plant):
    m = plant.model
    parts = [m.body_mass, m.body_inertia, m.body_pos, m.geom_size, m.geom_friction, m.jnt_range, m.actuator_forcerange, m.dof_armature]
    h = hashlib.sha256()
    for a in parts:
        h.update(np.ascontiguousarray(a, dtype=np.float64).tobytes())
    return dict(parameter_set_sha256=h.hexdigest(), nbody=int(m.nbody), ngeom=int(m.ngeom), nu=int(m.nu),
                bar_mass_kg=float(m.body_mass[plant.body_id("barbell")]), human_mass_kg=float(m.body_mass[1:].sum()-m.body_mass[plant.body_id("barbell")]),
                solver=solver_config(m))


def full_receipt(plants=None, ignore_prefixes=("runs/i3/res624/",)):
    status = [ln for ln in _git("status", "--porcelain").splitlines()
              if not any(ln[3:].replace("\\", "/").startswith(p) for p in ignore_prefixes)]
    r = dict(head=_git("rev-parse", "HEAD"), branch=_git("rev-parse", "--abbrev-ref", "HEAD"),
             worktree_clean=not status, uncommitted_outside_evidence=status,
             python=sys.version.split()[0], mujoco=mujoco.__version__, scipy=scipy.__version__, numpy=np.__version__,
             platform=platform.platform(), machine=platform.machine(), processor=platform.processor(),
             evaluator=dict(identity="biosyngen_benchmark.receiving_acceptance.evaluate_receiving", sha256=_sha(EVALUATOR)),
             controller=dict(identity="biosyngen.spatial.receiving_grasp_v2.EquilibriumHoldController"),
             source_sha256=_sha(SOURCES))
    from biosyngen.spatial.receiving_grasp_v2 import EquilibriumHoldController as C
    r["controller"].update(com_kp=C.COM_KP, com_kd=C.COM_KD, grip_fraction=.6)
    if plants:
        r["models"] = {k: model_identity(p) for k, p in plants.items()}
    return r
