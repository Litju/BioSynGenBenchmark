"""Latent-versus-observed event study (RES-483/484). Uses latent truth by design.

This module is an OBSERVABILITY_STUDY, not an evaluator. It simulates the
instrument chains, runs the independent sensor-only detectors
(biosyngen_benchmark.detectors), and compares their outputs with the latent
model events. The comparison maps native device times to simulation time
with the analyst's NOMINAL clock knowledge (offset only). Drift and jitter
therefore remain in the residuals, as they would for an analyst without a
shared trigger.
"""

from __future__ import annotations

from dataclasses import dataclass

from biosyngen_benchmark.detectors import DetectorConvention, detect_force_events, detect_lpt_events

from .instruments import ForceExport, LptExport
from .phases import PhaseAnalysis


@dataclass(frozen=True, slots=True)
class ObservabilityRow:
    latent_event: str
    latent_time_s: float | None
    observed_event: str
    observed_time_s: float | None
    error_ms: float | None
    resolution_ms: float
    status: str
    note: str = ""


PAIRS = (
    ("kinetic_onset", "force", "onset_threshold", "threshold crossing lags true onset"),
    ("kinetic_onset", "force", "onset_backdated", "conventional 30 ms backdating"),
    ("peak_downward_velocity", "force", "peak_downward_velocity", "integrated force"),
    ("com_velocity_reversal", "force", "velocity_reversal", "integrated force"),
    ("takeoff", "force", "takeoff", "force threshold"),
    ("com_velocity_reversal", "lpt", "lpt_velocity_reversal", "attachment point, not COM"),
    ("com_velocity_reversal", "lpt", "lpt_lowest_point", "attachment point, not COM"),
    ("takeoff", "lpt", "lpt_takeoff", "cable cannot observe contact"),
    ("touchdown", "force", "landing", "landing not in export"),
)


def study(
    analysis: PhaseAnalysis,
    force: ForceExport,
    lpt: LptExport,
    convention: DetectorConvention = DetectorConvention(),
) -> tuple[list[ObservabilityRow], dict, dict]:
    f_times = [s.native_time_s - force.model.clock.offset_s for s in force.samples]
    f_detect = detect_force_events(f_times, [s.fz_n for s in force.samples], convention)
    l_times = [s.native_time_s - lpt.model.clock.offset_s for s in lpt.samples]
    l_detect = detect_lpt_events(
        l_times, [s.displacement_m for s in lpt.samples], [s.provider_velocity_m_s for s in lpt.samples], convention
    )
    rows = []
    for latent_name, device, observed_name, note in PAIRS:
        latent = analysis.event(latent_name).time_s
        detected = (f_detect if device == "force" else l_detect)[observed_name]
        resolution = 1000.0 * (force.model.clock.sample_period_s if device == "force" else lpt.model.clock.sample_period_s)
        if detected.refusal is not None:
            rows.append(ObservabilityRow(latent_name, latent, f"{device}:{observed_name}", None, None,
                                         resolution, "NOT_EVALUABLE", detected.refusal))
            continue
        if latent is None:
            rows.append(ObservabilityRow(latent_name, None, f"{device}:{observed_name}", detected.native_time_s,
                                         None, resolution, "NO_LATENT_EVENT", note))
            continue
        error = 1000.0 * (detected.native_time_s - latent)
        rows.append(ObservabilityRow(latent_name, latent, f"{device}:{observed_name}", detected.native_time_s,
                                     error, resolution, "COMPARED", note))
    return rows, f_detect, l_detect
