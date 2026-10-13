"""One-command I1 integrated run: simulation -> instruments -> phases -> benchmark -> figures.

    python -m scripts.run_i1                 # writes runs/i1/res487/run
    python -m scripts.run_i1 --replicate     # two separate OS processes, digest comparison

All artifacts are deterministic (no timestamps, no machine paths). The
manifest records SHA-256 of every source file and artifact, plus the Python
version. SIMULATED_MEASUREMENT_ONLY; BIOLOGICAL_VALIDATION=NOT_ESTABLISHED;
REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

from biosyngen.i1.body import PlanarBody
from biosyngen.i1.export import force_export_dict, kinematic_export, lpt_export_dict
from biosyngen.i1.instruments import ForcePlateModel, LptModel, lpt_attachment_point, observe_force_plate, observe_lpt
from biosyngen.i1.observability import study
from biosyngen.i1.phases import analyze_phases
from biosyngen.i1.scenarios import reference_cmj, reference_loaded_cmj, reference_sj
from biosyngen.i1.simulate import ContactMode, simulate_jump
from biosyngen_benchmark.i1 import CLAIMS, evaluate_i1
from biosyngen_benchmark.suite import accept_everything, refuse_everything, run_suite
from scripts.i1_benchmark_suite import LOADED_LPT, build_cases, canary_invariance
from scripts.svgplot import PALETTE, PHASE_FILL, Figure

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "runs" / "i1" / "res487" / "run"
CEILING = ["NUMERICAL_VERIFICATION_ONLY", "SIMULATED_MEASUREMENT_ONLY",
           "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED", "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED"]
SOURCES = sorted(
    [p for p in (ROOT / "biosyngen").rglob("*.py")]
    + [p for p in (ROOT / "biosyngen_benchmark").rglob("*.py")]
    + [ROOT / "scripts" / n for n in ("run_i1.py", "svgplot.py", "i1_benchmark_suite.py", "i1_optimize_control.py",
                                      "i1_provenance_sensitivity.py")]
)


def r(v, n=6):
    return None if v is None else round(v, n)


def phase_bands(panel, pa):
    for ph in pa.phases:
        if ph.start_s is not None and ph.end_s is not None:
            panel.band(ph.start_s, ph.end_s, PHASE_FILL.get(ph.name, "#F5F5F5"), ph.name.replace("_", " "))


def stick_panel(fig, traj, pa, x, y, w, h):
    times = [0.0]
    for name in ("peak_downward_velocity", "com_velocity_reversal", "heel_off", "takeoff"):
        t = pa.event(name).time_s
        if t is not None:
            times.append(t)
    onset, vmin = pa.event("kinetic_onset").time_s, pa.event("peak_downward_velocity").time_s
    times.insert(1, 0.5 * (onset + vmin))
    takeoff = pa.event("takeoff").time_s
    times.append(takeoff + 0.6 * (pa.takeoff_velocity_m_s / 9.80665))
    times = sorted(times)
    labels = ["quiet", "unweighting", "peak v-", "reversal", "heel-off", "takeoff", "flight"]
    # Equal aspect: metres per pixel identical on both axes.
    y_range = (-0.05, 1.95)
    x_span = (y_range[1] - y_range[0]) * w / h
    spacing = x_span / len(times)
    p = fig.panel(x, y, w, h, (-0.5 * spacing, -0.5 * spacing + x_span), y_range,
                  "(a) Model postures at latent events (stick figure; HAT = head-arms-trunk)", "", "height (m)")
    body = PlanarBody(traj.scenario.body, traj.scenario.bar)
    p.raw(f'<line x1="{p.px(p.xr[0]):.2f}" y1="{p.py(0):.2f}" x2="{p.px(p.xr[1]):.2f}" '
          f'y2="{p.py(0):.2f}" stroke="{PALETTE["black"]}" stroke-width="1.5"/>')
    for k, t in enumerate(times):
        s = traj.sample_at_or_after(t)
        q = list(s.q)
        off = k * spacing - q[0]
        pts = {name: body.point(q, terms) for name, terms in body.points.items()}
        head = body.point(q, body.points["hip"] + ((5, body.params.hat_length_m + 0.18, 0.0),))
        chain = [pts["toe"], pts["heel"], pts["ankle"], pts["toe"]]
        legs = [pts["ankle"], pts["knee"], pts["hip"], head]
        color = PALETTE["blue"] if s.mode is not ContactMode.FLIGHT else PALETTE["vermillion"]
        p.line([c[0] + off for c in chain], [c[1] for c in chain], PALETTE["grey"], 1.5)
        p.line([c[0] + off for c in legs], [c[1] for c in legs], color, 2.4)
        cx, cz = s.system_com_m
        p.markers([cx + off], [cz], PALETTE["black"], r=3)
        p.text(k * spacing, 1.88, labels[k] if k < len(labels) else "", size=9, anchor="middle")
        p.text(k * spacing, 1.76, f"t={t:.3f}s", size=8, color=PALETTE["grey"], anchor="middle")
    p.legend += [("line", PALETTE["blue"], "in contact"), ("line", PALETTE["vermillion"], "flight"),
                 ("marker", PALETTE["black"], "system COM")]
    return p


def figure_cmj(traj, pa, force, lpt, fdet, ldet, rows) -> str:
    fig = Figure(1240, 1360, "I1 reference CMJ: emergent phases, latent states and simulated devices",
                 "Forward-dynamics countermovement jump of a declared hypothetical adult. Latent model states are "
                 "separated from simulated force-plate and cable-LPT exports and from detector estimates.",
                 "SIMULATED_MEASUREMENT_ONLY · BIOLOGICAL_VALIDATION=NOT_ESTABLISHED · "
                 "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED · devices are SIMULATED_DEVICE_HYPOTHESIS")
    stick_panel(fig, traj, pa, 70, 80, 1130, 250)
    t_end = traj.samples[-1].time_s
    ts = [s.time_s for s in traj.samples]
    W = pa.weight_n
    # (b) vertical GRF
    p = fig.panel(70, 380, 540, 250, (0, t_end), (-100, 2100), "(b) Vertical ground reaction: latent vs plate export",
                  "simulation time (s)", "Fz (N)")
    phase_bands(p, pa)
    p.hline(W, PALETTE["grey"], "body weight")
    p.line(ts, [s.grf_z_n for s in traj.samples], PALETTE["black"], 1.8, label="latent Fz (model contact)")
    off = force.model.clock.offset_s
    fs = force.samples[::4]
    p.markers([s.native_time_s - off for s in fs], [s.fz_n for s in fs], PALETTE["orange"], r=1.3,
              label="plate Fz samples (every 4th; native clock, nominal offset)")
    for name, color, pos in (("kinetic_onset", PALETTE["blue"], 0), ("takeoff", PALETTE["blue"], 1)):
        t = pa.event(name).time_s
        p.vline(t, color, f"latent {name.replace('_', ' ')}", position=pos)
    for name, label, pos in (("onset_threshold", "detected onset", 2), ("takeoff", "detected takeoff", 3)):
        d = fdet[name]
        if d.native_time_s is not None:
            p.vline(d.native_time_s, PALETTE["vermillion"], label, dash="1 2", position=pos)
    # (c) joint angles
    p = fig.panel(670, 380, 530, 250, (0, t_end), (40, 200), "(c) Joint angles (extension-positive)",
                  "simulation time (s)", "angle (deg)")
    phase_bands(p, pa)
    for j, (name, color) in enumerate((("ankle", PALETTE["green"]), ("knee", PALETTE["blue"]), ("hip", PALETTE["purple"]))):
        p.line(ts, [math.degrees(s.joint_angle_rad[j]) for s in traj.samples], color, 1.8, label=name)
    # (d) excitation / activation
    p = fig.panel(70, 680, 540, 250, (0, t_end), (-0.02, 1.05),
                  "(d) Extensor excitation (dashed) and activation (solid): LATENT HYPOTHESIS, not EMG",
                  "simulation time (s)", "normalized")
    phase_bands(p, pa)
    for idx, (name, color) in enumerate(((0, PALETTE["green"]), (2, PALETTE["blue"]), (4, PALETTE["purple"]))):
        label = ("ankle", "knee", "hip")[idx]
        p.line(ts, [s.excitation[name] for s in traj.samples], color, 1.0, dash="4 3")
        p.line(ts, [s.activation[name] for s in traj.samples], color, 1.8, label=f"{label} extensor")
    # (e) COM velocity: latent vs force-integrated
    p = fig.panel(670, 680, 530, 250, (0, t_end), (-2.0, 2.8), "(e) System COM vertical velocity: latent vs force-derived",
                  "simulation time (s)", "v_z (m/s)")
    phase_bands(p, pa)
    p.hline(0.0, PALETTE["grey"])
    p.line(ts, [s.system_com_velocity_m_s[1] for s in traj.samples], PALETTE["black"], 1.8, label="latent (model)")
    bw = fdet["body_weight"].value
    m = bw / 9.80665
    t0 = fdet["onset_backdated"].native_time_s
    v, prev, est_t, est_v = 0.0, None, [], []
    t_to = fdet["takeoff"].native_time_s
    for s in force.samples:
        t = s.native_time_s - off
        if t < t0 or s.fz_n is None or t > t_to:
            continue
        if prev is not None:
            v += 0.5 * ((prev[1] - bw) + (s.fz_n - bw)) / m * (t - prev[0])
        prev = (t, s.fz_n)
        est_t.append(t)
        est_v.append(v)
    p.line(est_t, est_v, PALETTE["vermillion"], 1.4, dash="5 3", label="integrated plate force (detector; v=0 at backdated onset)")
    # (f) LPT
    p = fig.panel(70, 980, 540, 250, (0, t_end), (-0.4, 0.75),
                  "(f) Hip-belt cable LPT: cable displacement is not COM displacement",
                  "simulation time (s)", "displacement (m)")
    phase_bands(p, pa)
    loff = lpt.model.clock.offset_s
    ls = [s for s in lpt.samples if s.displacement_m is not None][::3]
    p.markers([s.native_time_s - loff for s in ls], [s.displacement_m for s in ls], PALETTE["sky"], r=1.3,
              label="LPT cable displacement (native clock, every 3rd)")
    z0 = traj.samples[0].system_com_m[1]
    p.line(ts, [s.system_com_m[1] - z0 for s in traj.samples], PALETTE["black"], 1.6, label="latent system COM vertical displacement")
    a0 = lpt_attachment_point(traj.samples[0], traj, lpt.model)[2]
    p.line(ts, [lpt_attachment_point(s, traj, lpt.model)[2] - a0 for s in traj.samples], PALETTE["green"], 1.4, dash="4 3",
           label="latent belt point vertical displacement")
    rev = ldet["lpt_velocity_reversal"].native_time_s
    if rev is not None:
        p.vline(rev, PALETTE["sky"], "LPT reversal (detector)", dash="1 2", position=0)
    p.vline(pa.event("com_velocity_reversal").time_s, PALETTE["black"], "latent COM reversal", position=1)
    # (g) COP
    p = fig.panel(670, 980, 530, 250, (0, t_end), (-0.1, 0.2), "(g) COP: latent vs plate export (missing when Fz < 20 N)",
                  "simulation time (s)", "x (m, lab)")
    phase_bands(p, pa)
    p.line(ts, [s.heel_m[0] for s in traj.samples], PALETTE["grey"], 1.0, dash="2 2", label="heel x")
    p.line(ts, [s.toe_m[0] for s in traj.samples], PALETTE["grey"], 1.0, label="toe x")
    p.line(ts, [s.cop_x_m for s in traj.samples], PALETTE["black"], 1.6, label="latent COP")
    cs = force.samples[::4]
    p.markers([s.native_time_s - off for s in cs], [s.cop_x_m for s in cs], PALETTE["orange"], r=1.2, label="plate COP export")
    missing = [s.native_time_s - off for s in force.samples if s.cop_x_m is None][::10]
    p.missing(missing, PALETTE["vermillion"], "COP missing (not zero)")
    obs = "; ".join(f"{r_.latent_event}->{r_.observed_event.split(':')[1]}: "
                    + (f"{r_.error_ms:+.1f} ms" if r_.error_ms is not None else r_.status)
                    for r_ in rows)
    fig.note(70, 1272, "Detector minus latent event times (simulated devices, default noise):", 10, weight="bold")
    words, line, y = obs.split("; "), "", 1290
    for wd in words:
        if len(line) + len(wd) > 190:
            fig.note(70, y, line, 9)
            y += 14
            line = ""
        line += wd + "; "
    fig.note(70, y, line, 9)
    fig.note(70, y + 20, "Shaded bands = latent phases from model state predicates (not scheduled). Landing is not modeled: "
             "runs end at foot-ground contact.", 9, PALETTE["grey"])
    return fig.render()


def figure_loaded(traj, pa, lpt) -> str:
    fig = Figure(1240, 600, "I1 loaded CMJ (20 kg gripped bar): body, bar and system COM are distinct",
                 "Bar is a separate point mass on a compliant grip coupling; the LPT is attached to the bar collar.",
                 "SIMULATED_MEASUREMENT_ONLY · coupling stiffness/damping ASSUMED_UNCALIBRATED · not a back-squat or validated loaded-jump model")
    ts = [s.time_s for s in traj.samples]
    t_end = ts[-1]
    p = fig.panel(70, 80, 540, 220, (0, t_end), (-2.2, 2.2), "(a) Vertical velocity", "simulation time (s)", "v_z (m/s)")
    phase_bands(p, pa)
    p.line(ts, [s.body_com_velocity_m_s[1] for s in traj.samples], PALETTE["blue"], 1.6, label="body COM")
    p.line(ts, [s.bar_velocity_m_s[1] for s in traj.samples], PALETTE["vermillion"], 1.4, dash="4 3", label="bar")
    p.line(ts, [s.system_com_velocity_m_s[1] for s in traj.samples], PALETTE["black"], 1.6, label="system COM (body + bar)")
    p = fig.panel(670, 80, 530, 220, (0, t_end), (-100, 800), "(b) Grip coupling force on bar (vertical)",
                  "simulation time (s)", "N")
    phase_bands(p, pa)
    p.hline(traj.scenario.bar.mass_kg * 9.80665, PALETTE["grey"], "bar weight")
    p.line(ts, [s.coupling_force_on_bar_n[1] for s in traj.samples], PALETTE["vermillion"], 1.6, label="coupling force")
    p = fig.panel(70, 350, 540, 220, (0, t_end), (-0.35, 0.45), "(c) Bar-collar cable LPT vs bar vertical displacement",
                  "simulation time (s)", "displacement (m)")
    phase_bands(p, pa)
    off = lpt.model.clock.offset_s
    ls = [s for s in lpt.samples if s.displacement_m is not None][::3]
    p.markers([s.native_time_s - off for s in ls], [s.displacement_m for s in ls], PALETTE["sky"], r=1.3, label="LPT cable displacement")
    b0 = traj.samples[0].bar_m[1]
    p.line(ts, [s.bar_m[1] - b0 for s in traj.samples], PALETTE["vermillion"], 1.4, dash="4 3", label="latent bar vertical displacement")
    p = fig.panel(670, 350, 530, 220, (0, t_end), (-100, 2400), "(d) Vertical ground reaction (system)", "simulation time (s)", "Fz (N)")
    phase_bands(p, pa)
    p.hline(pa.weight_n, PALETTE["grey"], "system weight")
    p.line(ts, [s.grf_z_n for s in traj.samples], PALETTE["black"], 1.6, label="latent Fz")
    return fig.render()


def figure_vv(trajs, suite, sens) -> str:
    fig = Figure(1240, 700, "I1 verification evidence: numerical residuals, evaluator margins and sensitivity",
                 "Energy-identity and constraint residuals of the reference runs, the evaluator's residual-to-tolerance "
                 "ratios on positive controls and mutations, and one-at-a-time takeoff-velocity sensitivity.",
                 "Verification of declared mechanics and software only (KNOWN_ANSWER_VERIFICATION); no empirical validation")
    p = fig.panel(70, 80, 540, 240, (0, 1.6), (-8, -1), "(a) Work-energy identity residual |dE - W| (log10 J)",
                  "simulation time (s)", "log10 J")
    colors = (PALETTE["black"], PALETTE["vermillion"], PALETTE["blue"])
    for (name, traj), color in zip(trajs.items(), colors):
        e0 = traj.samples[0].kinetic_energy_j + traj.samples[0].potential_energy_j
        ys = []
        for s in traj.samples:
            res = abs(s.kinetic_energy_j + s.potential_energy_j - e0 - (s.work_joint_j + s.work_coupling_damping_j + s.work_contact_j))
            ys.append(math.log10(max(res, 1e-12)))
        p.line([s.time_s for s in traj.samples], ys, color, 1.2, label=name)
    p = fig.panel(670, 80, 530, 240, (0, 1.6), (-17, -8), "(b) Contact constraint residual (log10 m)",
                  "simulation time (s)", "log10 m")
    for (name, traj), color in zip(trajs.items(), colors):
        p.line([s.time_s for s in traj.samples],
               [math.log10(max(s.constraint_residual_m, 1e-17)) if s.mode is not ContactMode.FLIGHT else None
                for s in traj.samples], color, 1.2, label=name)
    # (c) evaluator ratios
    fams = suite["families"]
    p = fig.panel(70, 380, 470, 260, (-0.5, len(fams) - 0.5), (0, 2), "(c) Adjudicated outcomes per case family (count)",
                  "case family", "claim instances")
    family_labels = {"benchmark_closed_form": "closed form", "i0_prescribed_force": "I0 1-D",
                     "i1_multisegment": "I1 multisegment", "unseen_model_family": "unseen 2-body",
                     "mutation": "negatives"}
    p.xtick_labels = {i: family_labels.get(family, family[:12]) for i, family in enumerate(fams)}
    agg = {f: {"PASS": 0, "REJECTED": 0, "NOT_EVALUABLE": 0} for f in fams}
    for rw in suite["rows"]:
        agg[rw["family"]][rw["observed"] if rw["observed"] in ("PASS", "REJECTED") else "NOT_EVALUABLE"] += 1
    top = max(sum(v.values()) for v in agg.values())
    p.yr = (0, top * 1.6)
    p.legend_top = True
    for k, fam in enumerate(fams):
        base = 0
        for outcome, color in (("PASS", PALETTE["green"]), ("REJECTED", PALETTE["vermillion"]), ("NOT_EVALUABLE", PALETTE["grey"])):
            n = agg[fam][outcome]
            if n:
                p.raw(f'<rect x="{p.px(k - 0.3):.2f}" y="{p.py(base + n):.2f}" width="{p.px(k + 0.3) - p.px(k - 0.3):.2f}" '
                      f'height="{p.py(base) - p.py(base + n):.2f}" fill="{color}"/>')
                p.text(k, base + n / 2, str(n), size=9, color="#FFFFFF", anchor="middle")
            base += n
    p.legend += [("marker", PALETTE["green"], "PASS (expected for positives)"),
                 ("marker", PALETTE["vermillion"], "REJECTED (expected for negatives)"),
                 ("marker", PALETTE["grey"], "NOT_EVALUABLE (expected abstentions)")]
    t = suite["totals"]
    p.text(len(fams) - 0.1, top * 1.22, f"{t['agreements']}/{t['adjudicated_claim_instances']} agree; gate {suite['gate']}; "
           f"refuse-all & accept-all FAIL", size=9, anchor="end")
    # (d) sensitivity tornado
    ref = next(s for s in sens if s["parameter"] == "reference")["takeoff_velocity_m_s"]
    params = []
    for s in sens:
        if s["parameter"] == "reference":
            continue
        params.append((s["parameter"], s["value"], s.get("takeoff_velocity_m_s"), s["termination"]))
    names = sorted({pn for pn, _, _, _ in params}, key=lambda n: [pn for pn, _, _, _ in params].index(n))
    p = fig.panel(790, 380, 410, 260, (1.5, 3.0), (-0.5, len(names) - 0.5),
                  "(d) Takeoff velocity sensitivity (one at a time)", "takeoff velocity (m/s); dashed = reference", "")
    p.show_yticks = False
    p.raw(f'<line x1="{p.px(ref):.2f}" y1="{p.y:.2f}" x2="{p.px(ref):.2f}" y2="{p.y + p.h:.2f}" stroke="#222" stroke-dasharray="4 3"/>')
    for k, name in enumerate(names):
        vals = [(v, vt, term) for pn, v, vt, term in params if pn == name]
        yk = len(names) - 1 - k
        failures = []
        for v, vt, term in vals:
            if vt is None or term != "LANDING_CONTACT_NOT_MODELED":
                failures.append(f"{v:+g}: {term}")
                continue
            p.raw(f'<line x1="{p.px(ref):.2f}" y1="{p.py(yk):.2f}" x2="{p.px(vt):.2f}" y2="{p.py(yk):.2f}" '
                  f'stroke="{PALETTE["blue"]}" stroke-width="6"/>')
            p.text(vt, yk + 0.28, f"{v:g}", size=7.5, anchor="middle")
        if failures:
            p.text(1.52, yk - 0.12, "; ".join(failures), size=7.5, color=PALETTE["vermillion"])
        fig.note(575, p.py(yk) + 3, name.replace("_", " "), 8.5)
    return fig.render()


def build(out: Path) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    artifacts: dict[str, bytes] = {}
    cmj, sj, loaded = simulate_jump(reference_cmj()), simulate_jump(reference_sj()), simulate_jump(reference_loaded_cmj())
    pa_cmj, pa_sj, pa_loaded = analyze_phases(cmj), analyze_phases(sj, "SJ"), analyze_phases(loaded)
    force = observe_force_plate(cmj, ForcePlateModel())
    lpt = observe_lpt(cmj, LptModel())
    rows, fdet, ldet = study(pa_cmj, force, lpt)
    lpt_loaded = observe_lpt(loaded, LOADED_LPT)
    cases, base = build_cases()
    suite = run_suite(cases)
    refuse, accept = run_suite(cases, refuse_everything), run_suite(cases, accept_everything)
    canary = canary_invariance(base["cmj"])
    sens_path = ROOT / "runs" / "i1" / "res486" / "sensitivity.json"
    sens = json.loads(sens_path.read_text(encoding="utf-8"))

    def phases_dict(pa):
        return {
            "events": [{"name": e.name, "time_s": r(e.time_s), "uncertainty_s": r(e.uncertainty_s, 12),
                        "evidence_class": e.evidence_class, "definition": e.definition} for e in pa.events],
            "phases": [{"name": p.name, "start_s": r(p.start_s), "end_s": r(p.end_s), "definition": p.definition} for p in pa.phases],
            "takeoff_velocity_m_s": r(pa.takeoff_velocity_m_s), "apex_rise_m": r(pa.flight_apex_rise_m),
            "countermovement_depth_m": r(pa.countermovement_depth_m),
            "peak_downward_velocity_m_s": r(pa.peak_downward_velocity_m_s), "refusals": list(pa.refusals),
        }

    def run_dict(traj):
        e0 = traj.samples[0].kinetic_energy_j + traj.samples[0].potential_energy_j
        return {
            "termination": traj.termination, "rhs_evaluations": traj.rhs_evaluations, "samples": len(traj.samples),
            "events": [{"kind": e.kind, "time_s": r(e.time_s, 9), "detail": e.detail} for e in traj.events],
            "max_energy_identity_residual_j": max(abs(s.kinetic_energy_j + s.potential_energy_j - e0
                                                      - (s.work_joint_j + s.work_coupling_damping_j + s.work_contact_j))
                                                  for s in traj.samples),
            "max_constraint_residual_m": max(s.constraint_residual_m for s in traj.samples),
            "peak_grf_bw": max(s.grf_z_n for s in traj.samples) / traj.samples[0].grf_z_n,
        }

    results = {
        "claim_ceiling": CEILING,
        "runs": {"cmj": run_dict(cmj), "sj": run_dict(sj), "loaded_cmj_20kg": run_dict(loaded)},
        "phases": {"cmj": phases_dict(pa_cmj), "sj": phases_dict(pa_sj), "loaded_cmj_20kg": phases_dict(pa_loaded)},
        "observability_cmj": [{"latent": x.latent_event, "observed": x.observed_event, "status": x.status,
                               "error_ms": r(x.error_ms, 4), "resolution_ms": x.resolution_ms, "note": x.note} for x in rows],
        "benchmark_suite": {"gate": suite["gate"], "gate_failures": suite["gate_failures"],
                            "agreements": suite["totals"]["agreements"],
                            "instances": suite["totals"]["adjudicated_claim_instances"],
                            "cases": suite["totals"]["cases"],
                            "refuse_everything_gate": refuse["gate"], "accept_everything_gate": accept["gate"],
                            "canary": canary,
                            "per_claim": {c: {k: v for k, v in suite["per_claim"][c].items()} for c in CLAIMS}},
    }
    artifacts["results.json"] = (json.dumps(results, indent=1, sort_keys=True, default=str) + "\n").encode()
    artifacts["evaluator-input-cmj.json"] = (json.dumps({
        "candidate_id": "i1_cmj", "claims": list(CLAIMS),
        "kinematics": kinematic_export(cmj, LptModel()), "force": force_export_dict(force), "lpt": lpt_export_dict(lpt),
    }, sort_keys=True) + "\n").encode()
    artifacts["figure-cmj.svg"] = figure_cmj(cmj, pa_cmj, force, lpt, fdet, ldet, rows).encode()
    artifacts["figure-loaded-cmj.svg"] = figure_loaded(loaded, pa_loaded, lpt_loaded).encode()
    artifacts["figure-vv.svg"] = figure_vv({"cmj": cmj, "loaded cmj": loaded, "sj": sj}, suite, sens).encode()
    manifest = {
        "claim_ceiling": CEILING,
        "python": sys.version.split()[0],
        "external_dependencies": [],
        "source_sha256": {str(p.relative_to(ROOT)).replace("\\", "/"): hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES},
        "input_sha256": {"runs/i1/res486/sensitivity.json": hashlib.sha256(sens_path.read_bytes()).hexdigest()},
        "artifact_sha256": {k: hashlib.sha256(v).hexdigest() for k, v in sorted(artifacts.items())},
        "determinism": "no timestamps, machine paths or run identifiers in artifacts",
    }
    artifacts["manifest.json"] = (json.dumps(manifest, indent=1, sort_keys=True) + "\n").encode()
    for name, data in artifacts.items():
        (out / name).write_bytes(data)
    digest = hashlib.sha256("\n".join(f"{k}:{hashlib.sha256(v).hexdigest()}" for k, v in sorted(artifacts.items())).encode()).hexdigest()
    return {"digest": digest, "files": sorted(artifacts), "gate": suite["gate"]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--replicate", action="store_true", help="run twice in separate processes and compare digests")
    args = parser.parse_args()
    if args.replicate:
        base = args.out.parent
        digests = {}
        for name in ("replica-a", "replica-b"):
            proc = subprocess.run([sys.executable, "-m", "scripts.run_i1", "--out", str(base / name)],
                                  cwd=ROOT, capture_output=True, text=True, check=True)
            digests[name] = proc.stdout.strip().splitlines()[-1].split()[-1]
        files = sorted(p.name for p in (base / "replica-a").iterdir())
        same = all((base / "replica-a" / f).read_bytes() == (base / "replica-b" / f).read_bytes() for f in files)
        record = {"replicas": digests, "byte_identical": same, "files": files,
                  "method": "two separate OS processes of `python -m scripts.run_i1`; every file compared byte-for-byte"}
        (base / "reproducibility.json").write_text(json.dumps(record, indent=1, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(record, indent=1))
        if not same or len(set(digests.values())) != 1:
            raise SystemExit("replicas differ")
        return
    info = build(args.out)
    print(f"gate={info['gate']} files={len(info['files'])} out={args.out}")
    print(f"digest {info['digest']}")


if __name__ == "__main__":
    main()
