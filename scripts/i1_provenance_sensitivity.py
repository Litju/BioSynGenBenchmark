"""RES-486: parameter provenance ledger, sensitivity sweeps and counterexamples.

    python -m scripts.i1_provenance_sensitivity

Writes:
* docs/i1/parameter_provenance.json: every model value with its provenance
  status and sensitivity range;
* runs/i1/res486/sensitivity.json: one-at-a-time sweeps across the declared
  ranges, with control timing held fixed and separate strategy perturbations;
* runs/i1/res486/sensor_noise_spread.json: detector spread over 10 plate noise
  seeds (stochastic sensor uncertainty, separate from model parameters);
* runs/i1/res486/counterexamples.json: quantified counterexamples to
  mistaken equivalences (bar vs system COM; provider velocity vs true cable
  rate; cable vs vertical displacement).

No dataset is read. All numbers are SIMULATED_MEASUREMENT_ONLY.
"""

from __future__ import annotations

from dataclasses import asdict, replace
import json
import math
import statistics
from pathlib import Path

from biosyngen.i1.control import CountermovementStrategy
from biosyngen.i1.instruments import (
    ForcePlateModel,
    LptModel,
    cable_length,
    lpt_attachment_point,
    observe_force_plate,
    observe_lpt,
)
from biosyngen.i1.observability import study
from biosyngen.i1.params import PROVENANCE, BarParameters, BodyParameters, NeuromuscularParameters
from biosyngen.i1.phases import analyze_phases
from biosyngen.i1.scenarios import LOADED_STRATEGY, reference_cmj, reference_loaded_cmj
from biosyngen.i1.simulate import ContactMode, InadmissibleScenario, simulate_jump


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "runs" / "i1" / "res486"
LEDGER = ROOT / "docs" / "i1" / "parameter_provenance.json"

LITERATURE_ANCHORS = [
    {
        "citation": "Bobbert MF, Gerritsen KG, Litjens MC, Van Soest AJ (1996) Med Sci Sports Exerc 28:1402-1412",
        "doi": "10.1097/00005768-199611000-00009",
        "read_scope": "PubMed abstract only (inspected 2026-10-12)",
        "inspected_values": "6 male volleyball players; CMJ height on average 3.4 cm greater than SJ from the same start position; "
                            "simulation ruled out elastic energy; attributed to active-state build-up before shortening",
        "use": "qualitative mechanism/direction anchor for CMJ-SJ difference; not a validation target",
    },
    {
        "citation": "Bobbert MF, Casius LJR (2005) Med Sci Sports Exerc 37:440-446",
        "doi": "10.1249/01.mss.0000155389.34538.97",
        "read_scope": "PubMed abstract only (inspected 2026-10-12)",
        "inspected_values": "4-segment, 6-muscle simulation; CMJ-SJ height difference 0.4 cm (infinite dSTIM/dt) to ~2.5 cm (lowest rate studied)",
        "use": "model-to-model comparison of the active-state mechanism; not human validation",
    },
    {
        "citation": "Ferreira ARP, Macedo VOC, Boullosa D, Vieira A (2023) Int J Exerc Sci 16:1038-1051",
        "doi": "10.70252/LBRW9042",
        "read_scope": "PubMed abstract only (inspected 2026-10-12)",
        "inspected_values": "phase-duration metrics less consistent between days than force/impulse metrics (n=42); no numeric durations in abstract",
        "use": "supports treating phase durations as strategy-dependent, not as fixed human norms",
    },
]


def summarize(traj) -> dict:
    pa = analyze_phases(traj)
    weight = pa.weight_n
    stance = [s for s in traj.samples if s.mode is not ContactMode.FLIGHT]

    def dur(name):
        p = pa.phase(name)
        return None if p.start_s is None or p.end_s is None else p.end_s - p.start_s

    return {
        "termination": traj.termination,
        "takeoff_velocity_m_s": pa.takeoff_velocity_m_s,
        "apex_rise_m": pa.flight_apex_rise_m,
        "countermovement_depth_m": pa.countermovement_depth_m,
        "unweighting_s": dur("unweighting"),
        "braking_s": dur("braking"),
        "propulsion_s": dur("propulsion"),
        "peak_force_bw": max(s.grf_z_n for s in stance) / weight,
        "min_force_bw": min(s.grf_z_n for s in stance) / weight,
        "refusals": list(pa.refusals),
    }


def ledger() -> dict:
    body, nm, bar = BodyParameters(), NeuromuscularParameters(), BarParameters()
    return {
        "claim_ceiling": "No value is SOURCE_VERIFIED; recalled values were not inspected in this run.",
        "values": {"body": asdict(body), "neuromuscular": asdict(nm), "bar": asdict(bar)},
        "provenance": {k: asdict(v) for k, v in PROVENANCE.items()},
        "literature_anchors_inspected": LITERATURE_ANCHORS,
    }


def sweeps() -> list[dict]:
    base = reference_cmj()
    nm = base.nm

    def ext(nm_, field, factor=None, value=None):
        def f(p):
            return replace(p, **{field: value if value is not None else getattr(p, field) * factor})
        return replace(nm_, ankle_extensor=f(nm_.ankle_extensor), knee_extensor=f(nm_.knee_extensor),
                       hip_extensor=f(nm_.hip_extensor))

    variants = [("reference", "none", base)]
    for label, lo, hi in (("strength_factor", 0.85, 1.15),):
        variants += [(label, lo, replace(base, nm=nm.scaled_strength(lo))), (label, hi, replace(base, nm=nm.scaled_strength(hi)))]
    for f in (0.85, 1.15):
        variants.append(("extensor_omega_max_factor", f, replace(base, nm=ext(nm, "max_shortening_velocity_rad_s", f))))
    for v in (0.15, 0.40):
        variants.append(("extensor_hill_curvature", v, replace(base, nm=ext(nm, "hill_curvature", value=v))))
    for v in (0.010, 0.020):
        variants.append(("activation_time_constant_s", v, replace(base, nm=replace(nm, activation_time_constant_s=v))))
    for v in (0.040, 0.060):
        variants.append(("deactivation_time_constant_s", v, replace(base, nm=replace(nm, deactivation_time_constant_s=v))))
    for v in (65.0, 85.0):
        variants.append(("body_mass_kg_strength_fixed", v, replace(base, body=replace(base.body, body_mass_kg=v))))
    for v in (0.5, 1.0):
        variants.append(("friction_coefficient", v, replace(base, body=replace(base.body, friction_coefficient=v))))
    strat = base.strategy
    for dv in (-0.02, 0.02):
        variants.append(("strategy_push_time_shift_s", dv, replace(base, strategy=replace(strat, push_time_s=strat.push_time_s + dv))))
    for v in (0.2, 0.4):
        variants.append(("strategy_release_fraction", v, replace(base, strategy=replace(strat, release_fraction=v))))
    out = []
    for label, value, scenario in variants:
        try:
            row = summarize(simulate_jump(scenario))
        except InadmissibleScenario as error:
            row = {"termination": "INADMISSIBLE", "detail": str(error)}
        out.append({"parameter": label, "value": value, **row})
        print(f"{label:34s} {str(value):6s} {row.get('termination')} vto={row.get('takeoff_velocity_m_s')}", flush=True)
    return out


def noise_spread(traj) -> dict:
    pa = analyze_phases(traj)
    lpt = observe_lpt(traj, LptModel())
    errors: dict[str, list[float]] = {}
    vto = []
    for seed in range(10):
        rows, fdet, _ = study(pa, observe_force_plate(traj, ForcePlateModel(seed=100 + seed)), lpt)
        for r in rows:
            if r.error_ms is not None and r.observed_event.startswith("force:"):
                errors.setdefault(f"{r.latent_event}->{r.observed_event}", []).append(r.error_ms)
        if fdet["takeoff_velocity"].value is not None:
            vto.append(fdet["takeoff_velocity"].value)
    return {
        "seeds": 10,
        "detector_error_ms": {k: {"mean": statistics.mean(v), "sd": statistics.stdev(v), "min": min(v), "max": max(v)}
                              for k, v in errors.items()},
        "takeoff_velocity_estimate_m_s": {"latent": pa.takeoff_velocity_m_s, "mean": statistics.mean(vto),
                                          "sd": statistics.stdev(vto)},
        "note": "spread from plate noise only; calibration bias is fixed across seeds and appears in the mean",
    }


def counterexamples() -> dict:
    loaded = simulate_jump(reference_loaded_cmj())
    takeoff = loaded.event_time("TAKEOFF")
    stance = [s for s in loaded.samples if s.time_s <= takeoff]
    bar_vs_system = max(abs(s.bar_velocity_m_s[1] - s.system_com_velocity_m_s[1]) for s in stance)
    bar_vs_body = max(abs(s.bar_velocity_m_s[1] - s.body_com_velocity_m_s[1]) for s in stance)
    cmj = simulate_jump(reference_cmj())
    model = LptModel(anchor_m=(-0.6, 0.3, 0.0))
    export = observe_lpt(cmj, model)
    worst_rate = worst_vertical = 0.0
    p0 = lpt_attachment_point(cmj.samples[0], cmj, model)
    for s in export.samples[5:-5]:
        latent = cmj.sample_at_or_after(s.native_time_s - model.clock.offset_s)
        p = lpt_attachment_point(latent, cmj, model)
        worst_vertical = max(worst_vertical, abs(s.displacement_m - (p[2] - p0[2])))
    # True cable rate by central difference of exact geometric length on the latent samples.
    samples = cmj.samples
    true_rate = []
    for a, b in zip(samples, samples[1:]):
        if b.time_s > a.time_s:
            la = cable_length(lpt_attachment_point(a, cmj, model), model.anchor_m)
            lb = cable_length(lpt_attachment_point(b, cmj, model), model.anchor_m)
            true_rate.append((0.5 * (a.time_s + b.time_s), (lb - la) / (b.time_s - a.time_s)))
    for s in export.samples:
        if s.provider_velocity_m_s is None:
            continue
        t = s.native_time_s - model.clock.offset_s
        ref = min(true_rate, key=lambda p: abs(p[0] - t))[1]
        worst_rate = max(worst_rate, abs(s.provider_velocity_m_s - ref))
    return {
        "loaded_cmj_bar_vs_system_com_vz_max_m_s": bar_vs_system,
        "loaded_cmj_bar_vs_body_com_vz_max_m_s": bar_vs_body,
        "oblique_lpt_cable_vs_vertical_displacement_max_m": worst_vertical,
        "provider_velocity_vs_true_cable_rate_max_m_s": worst_rate,
        "interpretation": [
            "bar velocity is not system-COM velocity (bar is a separately coupled body)",
            "an oblique cable displacement is not vertical displacement",
            "provider velocity differs from the true cable rate (finite difference of quantized, jittered samples)",
        ],
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    LEDGER.write_text(json.dumps(ledger(), indent=1, sort_keys=True), encoding="utf-8")
    sens = sweeps()
    (OUT / "sensitivity.json").write_text(json.dumps(sens, indent=1, sort_keys=True), encoding="utf-8")
    spread = noise_spread(simulate_jump(reference_cmj()))
    (OUT / "sensor_noise_spread.json").write_text(json.dumps(spread, indent=1, sort_keys=True), encoding="utf-8")
    ce = counterexamples()
    (OUT / "counterexamples.json").write_text(json.dumps(ce, indent=1, sort_keys=True), encoding="utf-8")
    print(json.dumps(spread, indent=1))
    print(json.dumps(ce, indent=1))


if __name__ == "__main__":
    main()
