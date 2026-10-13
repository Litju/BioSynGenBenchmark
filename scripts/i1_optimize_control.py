"""Declared control-hypothesis search for the I1 jump scenarios (RES-482/483).

The excitation timing of each task is chosen by penalized Nelder-Mead
maximizing takeoff vertical velocity of the system COM, a declared
maximal-jump objective. A trial gets no credit unless it shows:

* a sustained flight (termination LANDING_CONTACT_NOT_MODELED);
* no passive joint-limit torque at takeoff (no knee-lock artifact);
* |horizontal COM velocity| <= 0.3 m/s at takeoff.

This selects a control hypothesis; it does not fit data. Output:
runs/i1/res482/control-<task>.json (all trials, deterministic seeds).

    python -m scripts.i1_optimize_control --task loaded_cmj --bar-mass 20
    python -m scripts.i1_optimize_control --task sj
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

from biosyngen.i1.control import CountermovementStrategy, SquatJumpStrategy
from biosyngen.i1.params import BarParameters
from biosyngen.i1.scenarios import CMJ_POSTURE, SJ_POSTURE
from biosyngen.i1.simulate import InadmissibleScenario, JumpScenario, simulate_jump


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "runs" / "i1" / "res482"


def build(task: str, x: list[float], bar_mass: float) -> JumpScenario:
    if task == "sj":
        push, kd, ad = x[:3]
        return JumpScenario("sj", SJ_POSTURE, SquatJumpStrategy(push_time_s=push, knee_delay_s=kd, ankle_delay_s=ad))
    push, kd, ad, rf = x
    strategy = CountermovementStrategy(
        release_fraction=min(max(rf, 0.0), 1.0), push_time_s=max(push, 0.35), knee_delay_s=kd, ankle_delay_s=ad
    )
    bar = BarParameters(mass_kg=bar_mass) if task == "loaded_cmj" else None
    return JumpScenario(task, CMJ_POSTURE, strategy, bar=bar)


def score(task: str, x: list[float], bar_mass: float) -> tuple[float, dict]:
    try:
        traj = simulate_jump(build(task, x, bar_mass))
    except (InadmissibleScenario, ArithmeticError, ValueError) as error:
        return -10.0, {"x": x, "error": str(error)}
    takeoff = traj.event_time("TAKEOFF")
    record = {"x": x, "termination": traj.termination}
    if takeoff is None:
        return -5.0, record
    s = traj.sample_at_or_after(takeoff)
    vz, vx = s.system_com_velocity_m_s[1], s.system_com_velocity_m_s[0]
    passive = max(abs(p) for p in s.passive_torque_n_m)
    penalty = 0.0 if traj.termination == "LANDING_CONTACT_NOT_MODELED" else 2.0
    penalty += passive / 100.0 + 2.0 * max(0.0, abs(vx) - 0.3)
    record.update({"takeoff_vz": vz, "takeoff_vx": vx, "passive_n_m": passive, "objective": vz - penalty})
    return vz - penalty, record


def nelder_mead(f, x0, step, iterations, log):
    pts = [x0] + [[x0[j] + (step[j] if j == i else 0.0) for j in range(len(x0))] for i in range(len(x0))]
    vals = []
    for p in pts:
        v, rec = f(p)
        vals.append(v)
        log.append(rec)
    for _ in range(iterations):
        order = sorted(range(len(pts)), key=lambda i: -vals[i])
        pts = [pts[i] for i in order]
        vals = [vals[i] for i in order]
        c = [sum(p[j] for p in pts[:-1]) / (len(pts) - 1) for j in range(len(x0))]

        def ev(p):
            v, rec = f(p)
            log.append(rec)
            return v

        xr = [c[j] + (c[j] - pts[-1][j]) for j in range(len(x0))]
        fr = ev(xr)
        if fr > vals[0]:
            xe = [c[j] + 2 * (c[j] - pts[-1][j]) for j in range(len(x0))]
            fe = ev(xe)
            pts[-1], vals[-1] = (xe, fe) if fe > fr else (xr, fr)
        elif fr > vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = [c[j] + 0.5 * (pts[-1][j] - c[j]) for j in range(len(x0))]
            fc = ev(xc)
            if fc > vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0]] + [[pts[0][j] + 0.5 * (p[j] - pts[0][j]) for j in range(len(x0))] for p in pts[1:]]
                vals = [vals[0]] + [ev(p) for p in pts[1:]]
    best = max(range(len(pts)), key=lambda i: vals[i])
    return pts[best], vals[best]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", choices=("cmj", "sj", "loaded_cmj"), required=True)
    parser.add_argument("--bar-mass", type=float, default=20.0)
    parser.add_argument("--starts", type=int, default=3)
    parser.add_argument("--iterations", type=int, default=25)
    parser.add_argument("--seed", type=int, default=20261012)
    parser.add_argument(
        "--near",
        type=float,
        nargs="+",
        default=None,
        help="start each simplex near this point (+/- 10%% of the step) instead of random",
    )
    args = parser.parse_args()
    rng = random.Random(args.seed)
    log: list[dict] = []
    best = None
    for _ in range(args.starts):
        if args.task == "sj":
            x0 = [rng.uniform(0.35, 0.6), rng.uniform(-0.15, 0.05), rng.uniform(-0.05, 0.15)]
            step = [0.05, 0.05, 0.05]
        else:
            x0 = [rng.uniform(0.6, 0.9), rng.uniform(-0.25, 0.0), rng.uniform(-0.05, 0.15), rng.uniform(0.0, 0.5)]
            step = [0.04, 0.05, 0.05, 0.15]
        if args.near is not None:
            step = [0.5 * v for v in step]
            x0 = [c + rng.uniform(-0.1, 0.1) * st for c, st in zip(args.near, step)]
        x, v = nelder_mead(lambda p: score(args.task, p, args.bar_mass), x0, step, args.iterations, log)
        if best is None or v > best[1]:
            best = (x, v)
        print(f"start best {[round(a, 4) for a in x]} objective {v:.4f}", flush=True)
    OUT.mkdir(parents=True, exist_ok=True)
    suffix = f"{args.task}" + (f"-{args.bar_mass:g}kg" if args.task == "loaded_cmj" else "")
    out = OUT / f"control-{suffix}.json"
    out.write_text(json.dumps({
        "task": args.task, "bar_mass_kg": args.bar_mass if args.task == "loaded_cmj" else None,
        "seed": args.seed, "starts": args.starts, "iterations": args.iterations,
        "objective": "takeoff system-COM vz minus penalties (see module docstring)",
        "best_x": best[0], "best_objective": best[1], "trials": log,
    }, indent=1, sort_keys=True))
    print(f"BEST {best[0]} {best[1]:.4f} -> {out}")


if __name__ == "__main__":
    main()
