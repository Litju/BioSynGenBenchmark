"""RES-618 inertial checks on plain exports; no candidate/engine imports.

Tensors are about each segment COM in world axes. Parallel-axis transport
is performed here independently of the model builder and MuJoCo.
"""
import math
import numpy as np


def aggregate(segments):
    if not segments:
        raise ValueError("empty decomposition")
    names, masses, positions, tensors = [], [], [], []
    for row in segments:
        name, mass = row["name"], row["mass_kg"]
        p, i = np.asarray(row["com_world_m"], dtype=float), np.asarray(row["inertia_world_kg_m2"], dtype=float)
        if not name or name in names or row["frame"] != "WORLD_X_ANTERIOR_Y_LEFT_Z_UP":
            raise ValueError("duplicate identity or undeclared frame")
        if isinstance(mass, bool) or not isinstance(mass, (int, float)) or not math.isfinite(mass) or mass <= 0:
            raise ValueError("invalid mass")
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError("invalid COM")
        if i.shape != (3, 3) or not np.isfinite(i).all() or not np.allclose(i, i.T, atol=1e-14, rtol=0):
            raise ValueError("invalid tensor")
        eigen = np.linalg.eigvalsh(i)
        if eigen[0] <= 0 or eigen[-1] > eigen[0] + eigen[1] + 1e-12:
            raise ValueError("unrealizable tensor")
        names.append(name)
        masses.append(mass)
        positions.append(p)
        tensors.append(i)
    total = math.fsum(masses)
    com = np.sum(np.asarray(masses)[:, None] * positions, axis=0) / total
    tensor = np.zeros((3, 3))
    for mass, p, i in zip(masses, positions, tensors):
        offset = p - com
        tensor += i + mass * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))
    return dict(mass_kg=total, com_world_m=com.tolist(), inertia_world_kg_m2=tensor.tolist())


def compare_decomposition(parent, children, *, mass_tolerance_kg=1e-10, com_tolerance_m=1e-10,
                          inertia_tolerance_kg_m2=1e-10):
    """Numerical conservation only; caller must supply independently justified budgets."""
    for tolerance in (mass_tolerance_kg, com_tolerance_m, inertia_tolerance_kg_m2):
        if isinstance(tolerance, bool) or not math.isfinite(tolerance) or tolerance < 0:
            raise ValueError("invalid conservation budget")
    a, b = aggregate([parent]), aggregate(children)
    errors = dict(mass_error_kg=abs(a["mass_kg"]-b["mass_kg"]),
                  com_error_m=float(np.linalg.norm(np.asarray(a["com_world_m"])-b["com_world_m"])),
                  inertia_error_kg_m2=float(np.linalg.norm(np.asarray(a["inertia_world_kg_m2"])-b["inertia_world_kg_m2"])))
    passed = (errors["mass_error_kg"] <= mass_tolerance_kg and errors["com_error_m"] <= com_tolerance_m
              and errors["inertia_error_kg_m2"] <= inertia_tolerance_kg_m2)
    return dict(outcome="PASS" if passed else "FAIL", **errors)
