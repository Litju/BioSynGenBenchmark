"""RES-610 executed backend comparison on shared analytical fixtures.

    python -m scripts.res610_backend_comparison --out runs/i3/res598/working/res610-backends

Every candidate runs the same manufactured fixtures with closed-form truth:

A. Torque-free asymmetric rigid body (I = 0.1/0.2/0.3 kg m2) spinning near its
   intermediate axis for 5 s: kinetic energy and the world angular-momentum
   vector are exact invariants.
B. Planar double pendulum (two 1 kg point-like rods) released from rest for
   5 s: total mechanical energy is an exact invariant.
C. 1 kg box resting on a plane for 2 s: settled vertical contact force = m g.
D. 1 kg box sliding at 2 m/s with Coulomb mu = 0.5: stopping distance
   v^2 / (2 mu g) = 0.4077 m.

Availability (installation and import on this host) is measured, not assumed.
Wall time and peak process memory are recorded per backend. Results are a
numerical feasibility comparison only; no biological or device claim.
"""
from __future__ import annotations

import argparse
import importlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

G = 9.81
INERTIA = (0.1, 0.2, 0.3)
OMEGA0 = (0.01, 4.0, 0.02)           # rad/s, body frame: near the intermediate axis
SLIDE_V0, MU = 2.0, 0.5


def peak_rss_mb():
    if sys.platform == "win32":
        import ctypes
        from ctypes import wintypes

        class Counters(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                        ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                        ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                        ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]
        counters = Counters()
        counters.cb = ctypes.sizeof(Counters)
        current = ctypes.windll.kernel32.GetCurrentProcess
        current.restype = wintypes.HANDLE
        info = ctypes.windll.psapi.GetProcessMemoryInfo
        info.argtypes = (wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD)
        info.restype = wintypes.BOOL
        if not info(current(), ctypes.byref(counters), counters.cb):
            return float("nan")
        return counters.PeakWorkingSetSize / 2**20
    import resource
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024


def analytic():
    i1, i2, i3 = INERTIA
    w = OMEGA0
    energy = .5 * sum(i * x * x for i, x in zip(INERTIA, w))
    momentum = math.sqrt(sum((i * x) ** 2 for i, x in zip(INERTIA, w)))
    return dict(spin_energy_j=energy, spin_momentum_norm=momentum, rest_force_n=G,
                slide_distance_m=SLIDE_V0 ** 2 / (2 * MU * G))


# ---------------------------------------------------------------- MuJoCo
def mujoco_fixtures(dt):
    import mujoco
    import numpy as np
    out = {}
    spin = f"""<mujoco><option timestep="{dt}" gravity="0 0 0" integrator="RK4"/>
      <worldbody><body><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="{INERTIA[0]} {INERTIA[1]} {INERTIA[2]}"/>
      </body></worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(spin)
    d = mujoco.MjData(m)
    d.qvel[3:6] = OMEGA0
    mujoco.mj_forward(m, d)

    def spin_inv():
        rot = d.xmat[1].reshape(3, 3)
        w_body = d.qvel[3:6]
        h_world = rot @ (np.array(INERTIA) * w_body)
        return .5 * float(np.sum(np.array(INERTIA) * w_body ** 2)), h_world
    e0, h0 = spin_inv()
    worst_e = worst_h = 0.0
    for _ in range(int(round(5.0 / dt))):
        mujoco.mj_step(m, d)
        e, h = spin_inv()
        worst_e, worst_h = max(worst_e, abs(e - e0)), max(worst_h, float(np.linalg.norm(h - h0)))
    out["spin"] = dict(max_energy_error_j=worst_e, max_momentum_vector_error=worst_h, energy0_j=e0)

    pend = f"""<mujoco><option timestep="{dt}" integrator="RK4"><flag contact="disable" energy="enable"/></option>
      <worldbody><body pos="0 0 2"><joint type="hinge" axis="0 1 0"/>
        <inertial pos="0.5 0 0" mass="1" diaginertia="1e-4 0.0833 0.0833"/>
        <body pos="1 0 0"><joint type="hinge" axis="0 1 0"/><inertial pos="0.5 0 0" mass="1" diaginertia="1e-4 0.0833 0.0833"/>
        </body></body></worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(pend)
    d = mujoco.MjData(m)
    d.qpos[:] = (0.3, 0.2)
    mujoco.mj_forward(m, d)
    e0 = float(d.energy.sum())
    worst = 0.0
    for _ in range(int(round(5.0 / dt))):
        mujoco.mj_step(m, d)
        worst = max(worst, abs(float(d.energy.sum()) - e0))
    out["double_pendulum"] = dict(max_energy_error_j=worst)

    def box(v0, mu):
        xml = f"""<mujoco><option timestep="{dt}" cone="elliptic" impratio="10"/><worldbody>
          <geom type="plane" size="5 5 .1" friction="{mu} 0.005 0.0001"/>
          <body pos="0 0 0.1"><freejoint/><geom type="box" size=".1 .1 .1" mass="1" friction="{mu} 0.005 0.0001"/></body>
          </worldbody></mujoco>"""
        mm = mujoco.MjModel.from_xml_string(xml)
        dd = mujoco.MjData(mm)
        dd.qvel[0] = v0
        return mm, dd
    m, d = box(0.0, MU)
    for _ in range(int(round(2.0 / dt))):
        mujoco.mj_step(m, d)
    force = np.zeros(6)
    total = 0.0
    for i in range(d.ncon):
        mujoco.mj_contactForce(m, d, i, force)
        total += float(force[0])
    out["rest"] = dict(settled_normal_force_n=total, penetration_m=float(0.1 - d.qpos[2]))
    m, d = box(SLIDE_V0, MU)
    for _ in range(int(round(2.0 / dt))):
        mujoco.mj_step(m, d)
    out["slide"] = dict(distance_m=float(d.qpos[0]), final_speed_m_s=float(abs(d.qvel[0])))
    return out


# ---------------------------------------------------------------- OpenSim
def opensim_fixtures(dt):
    import opensim as osim
    osim.Logger.setLevelString("error")
    out = {}

    def run(model, state, duration, sampler=None):
        manager = osim.Manager(model)
        manager.setIntegratorMethod(osim.Manager.IntegratorMethod_RungeKutta3)
        manager.setIntegratorAccuracy(1e-9)
        manager.setIntegratorMaximumStepSize(dt)
        state.setTime(0)
        manager.initialize(state)
        worst = 0.0
        steps = 50
        for k in range(1, steps + 1):
            state = manager.integrate(duration * k / steps)
            if sampler is not None:
                worst = max(worst, sampler(state))
        # The integrated state is owned by the manager; copy before it is freed.
        return osim.State(state), worst

    model = osim.Model()
    model.setGravity(osim.Vec3(0, 0, 0))
    body = osim.Body("spin", 1.0, osim.Vec3(0), osim.Inertia(*INERTIA))
    model.addBody(body)
    model.addJoint(osim.FreeJoint("free", model.getGround(), body))
    state = model.initSystem()
    coordinates = model.getCoordinateSet()
    for axis, w in enumerate(OMEGA0):
        coordinates.get(axis).setSpeedValue(state, w)
    model.realizeVelocity(state)
    matter = model.getMatterSubsystem()

    def momentum(s):
        model.realizeVelocity(s)
        h = matter.calcSystemCentralMomentum(s).get(0)
        return (h.get(0), h.get(1), h.get(2))
    e0, h0 = model.calcKineticEnergy(state), momentum(state)

    def spin_err(s):
        h = momentum(s)
        return max(abs(model.calcKineticEnergy(s) - e0), math.dist(h, h0))
    _, worst = run(model, state, 5.0, spin_err)
    out["spin"] = dict(max_invariant_error=worst, energy0_j=e0,
                       note="FreeJoint coordinates are body-fixed rotations; speed initialization is OpenSim-native")

    model = osim.Model()
    model.setGravity(osim.Vec3(0, -G, 0))
    a = osim.Body("a", 1.0, osim.Vec3(0.5, 0, 0), osim.Inertia(1e-4, 0.0833, 0.0833))
    b = osim.Body("b", 1.0, osim.Vec3(0.5, 0, 0), osim.Inertia(1e-4, 0.0833, 0.0833))
    model.addBody(a)
    model.addBody(b)
    model.addJoint(osim.PinJoint("p1", model.getGround(), osim.Vec3(0, 2, 0), osim.Vec3(0), a, osim.Vec3(0), osim.Vec3(0)))
    model.addJoint(osim.PinJoint("p2", a, osim.Vec3(1, 0, 0), osim.Vec3(0), b, osim.Vec3(0), osim.Vec3(0)))
    state = model.initSystem()
    model.getCoordinateSet().get(0).setValue(state, 0.3)
    model.getCoordinateSet().get(1).setValue(state, 0.2)
    model.realizeVelocity(state)
    total0 = model.calcKineticEnergy(state) + model.calcPotentialEnergy(state)

    def pend_err(s):
        model.realizeVelocity(s)
        return abs(model.calcKineticEnergy(s) + model.calcPotentialEnergy(s) - total0)
    _, worst = run(model, state, 5.0, pend_err)
    out["double_pendulum"] = dict(max_energy_error_j=worst)

    def box(v0):
        model = osim.Model()
        model.setGravity(osim.Vec3(0, -G, 0))
        body = osim.Body("box", 1.0, osim.Vec3(0), osim.Inertia(1/600, 1/600, 1/600))
        model.addBody(body)
        model.addJoint(osim.FreeJoint("free", model.getGround(), body))
        floor = osim.ContactHalfSpace(osim.Vec3(0), osim.Vec3(0, 0, -math.pi / 2), model.getGround(), "floor")
        model.addContactGeometry(floor)
        force = osim.HuntCrossleyForce()
        force.setName("contact")
        force.addGeometry("floor")
        for i, (x, z) in enumerate(((.1, .1), (.1, -.1), (-.1, .1), (-.1, -.1))):
            model.addContactGeometry(osim.ContactSphere(0.005, osim.Vec3(x, -0.1, z), body, f"c{i}"))
            force.addGeometry(f"c{i}")
        force.setStiffness(1e7)
        force.setDissipation(1.0)
        force.setStaticFriction(MU)
        force.setDynamicFriction(MU)
        force.setViscousFriction(0.0)
        force.setTransitionVelocity(0.01)
        model.addForce(force)
        state = model.initSystem()
        coordinates = model.getCoordinateSet()
        coordinates.get(4).setValue(state, 0.1045)
        coordinates.get(3).setSpeedValue(state, v0)
        return model, state, force
    model, state, force = box(0.0)
    state, _ = run(model, state, 2.0)
    model.realizeDynamics(state)
    values, labels = force.getRecordValues(state), force.getRecordLabels()
    index = [labels.get(i) for i in range(labels.size())].index("contact.box.force.Y")
    out["rest"] = dict(settled_normal_force_n=values.get(index),
                       height_m=model.getCoordinateSet().get(4).getValue(state))
    model, state, force = box(SLIDE_V0)
    state, _ = run(model, state, 2.0)
    coordinate = model.getCoordinateSet().get(3)
    out["slide"] = dict(distance_m=coordinate.getValue(state),
                        final_speed_m_s=abs(coordinate.getSpeedValue(state)),
                        note="regularized Hunt-Crossley friction, transition velocity 0.01 m/s")
    return out


def run_backend(name, dt):
    started = time.perf_counter()
    try:
        module = importlib.import_module({"mujoco": "mujoco", "opensim": "opensim", "drake": "pydrake"}[name])
    except Exception as exc:  # measured availability, recorded verbatim
        return dict(backend=name, available=False, import_error=f"{type(exc).__name__}: {exc}")
    version = getattr(module, "__version__", None)
    if name == "opensim":
        version = module.GetVersionAndDate()
    from importlib import metadata
    dist = metadata.distribution({"mujoco": "mujoco", "opensim": "opensim"}[name])
    footprint = sum(Path(f.locate()).stat().st_size for f in dist.files or () if Path(f.locate()).is_file())
    package = dict(license=dist.metadata.get("License") or dist.metadata.get("License-Expression"),
                   classifiers=[c for c in dist.metadata.get_all("Classifier") or () if "License" in c],
                   installed_bytes=footprint,
                   moco_available=bool(getattr(module, "MocoStudy", None)) if name == "opensim" else None)
    try:
        fixtures = {"mujoco": mujoco_fixtures, "opensim": opensim_fixtures}[name](dt)
        error = None
    except Exception as exc:
        fixtures, error = None, f"{type(exc).__name__}: {exc}"
    return dict(backend=name, available=True, version=version, package=package, fixtures=fixtures, error=error,
                wall_s=time.perf_counter() - started, peak_rss_mb=peak_rss_mb())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    parser.add_argument("--backend", choices=("mujoco", "opensim", "drake"))
    parser.add_argument("--dt", type=float, default=.001)
    args = parser.parse_args(argv)
    if args.backend:      # child process: isolated memory measurement
        print(json.dumps(run_backend(args.backend, args.dt)))
        return 0
    out = Path(args.out)
    if out.exists():
        parser.error("output must be a fresh directory")
    out.mkdir(parents=True)
    results = []
    for name in ("mujoco", "opensim", "drake"):
        proc = subprocess.run([sys.executable, "-m", "scripts.res610_backend_comparison", "--out", str(out),
                               "--backend", name, "--dt", str(args.dt)], capture_output=True, text=True, timeout=900)
        line = proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else "{}"
        result = json.loads(line)
        result["exit_code"] = proc.returncode
        if proc.returncode:
            result["stderr_tail"] = proc.stderr[-2000:]
        results.append(result)
    pip = subprocess.run([sys.executable, "-m", "pip", "download", "--no-deps", "--dest", str(out / "probe"), "drake"],
                         capture_output=True, text=True, timeout=600)
    receipt = dict(issue="RES-610", host=dict(platform=platform.platform(), python=sys.version.split()[0],
                   machine=platform.machine()), dt_s=args.dt, analytic=analytic(), results=results,
                   drake_wheel_probe=dict(exit_code=pip.returncode, tail=(pip.stdout + pip.stderr)[-600:]),
                   source_sha=subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
                   claim_ceilings=["NUMERICAL_VERIFICATION_ONLY"])
    (out / "backend_comparison.json").write_text(json.dumps(receipt, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(receipt, indent=1)[:6000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
