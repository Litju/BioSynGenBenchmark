"""Bounded computational domains, not an athlete population distribution."""

import math

SCHEMA = "BSMB-CORPUS-CONFIG-1.0"
PROTOCOLS = ("guided_vertical", "cmj", "sj", "loaded_cmj", "loaded_jump_squat",
             "back_squat", "deadlift", "bench_press", "clean_pull", "snatch_pull", "power_clean_pull")
REFUSED = {"drop_jump": "COMPLIANT_IMPACT_NOT_IMPLEMENTED",
           "rebound_jump": "COMPLIANT_IMPACT_NOT_IMPLEMENTED",
           "power_clean": "CATCH_NOT_IMPLEMENTED", "snatch": "CATCH_NOT_IMPLEMENTED",
           "hang_pull": "HANG_START_NOT_IMPLEMENTED", "block_pull": "BLOCK_START_NOT_IMPLEMENTED"}

# Each accepted scalar is actually consumed by a builder or acquisition stage.
DOMAINS = {
    "body_mass_kg": (50, 100), "stature_m": (1.5, 2.0), "load_mass_kg": (0, 120),
    "baseline_s": (1.0, 2.0), "tempo_scale": (0.8, 1.2), "effort": (0.5, 1.0),
    "friction": (0.1, 1.0), "encoder_resolution_m": (1e-6, 1e-3),
    "retraction_speed_m_s": (0.01, 10), "angle_error_bound_rad": (0, 0.01),
    "angle_resolution_rad": (1e-6, 0.001), "force_noise_scale": (0, 10),
    "force_gain_scale": (0.9, 1.1), "force_adc_range_n": (50, 20000),
    "force_cutoff_hz": (20, 400), "force_offset_s": (-10, 10), "lpt_offset_s": (-10, 10),
    "force_drift_ppm": (-500, 500), "lpt_drift_ppm": (-500, 500),
    "jitter_sd_s": (0, 1e-5), "force_start_s": (0, 10), "lpt_start_s": (0, 10),
    "force_end_s": (0.1, 30), "lpt_end_s": (0.1, 30),
}
OPTIONS = {"acquisition_profile", "contact_configuration", "force_drop_indices", "lpt_drop_indices"}


class ConfigurationRefused(ValueError):
    pass


def validate_parameters(parameters, protocol):
    if protocol in REFUSED:
        raise ConfigurationRefused(REFUSED[protocol])
    if protocol not in PROTOCOLS:
        raise ConfigurationRefused("PROTOCOL_NOT_IMPLEMENTED")
    if not isinstance(parameters, dict) or set(parameters) - (DOMAINS.keys() | OPTIONS):
        raise ConfigurationRefused("UNKNOWN_SCIENTIFIC_PARAMETER")
    for key, value in parameters.items():
        if key in DOMAINS:
            lo, hi = DOMAINS[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
                raise ConfigurationRefused(f"PARAMETER_DOMAIN:{key}")
        elif key.endswith("drop_indices"):
            if not isinstance(value, list) or len(value) > 100000 or any(type(i) is not int or i < 0 for i in value) or len(set(value)) != len(value):
                raise ConfigurationRefused(f"INVALID_DROP_INDICES:{key}")
        elif key == "acquisition_profile" and value not in ("dual_1khz", "legacy_mixed"):
            raise ConfigurationRefused("ACQUISITION_PROFILE_UNSUPPORTED")
        elif key == "contact_configuration" and value != "NATIVE_FIXED_TOPOLOGY":
            raise ConfigurationRefused("CONTACT_TOPOLOGY_UNSUPPORTED")
    allowed_motion = {
        "guided_vertical": {"body_mass_kg", "load_mass_kg", "baseline_s", "tempo_scale"},
        "cmj": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "effort"},
        "sj": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "effort"},
        "loaded_cmj": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "effort"},
        "loaded_jump_squat": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "effort"},
        "back_squat": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "effort", "friction"},
        "deadlift": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "friction"},
        "bench_press": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale"},
        "clean_pull": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "effort"},
        "snatch_pull": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "effort"},
        "power_clean_pull": {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "effort"},
    }
    motion_keys = {"body_mass_kg", "stature_m", "load_mass_kg", "baseline_s", "tempo_scale", "effort", "friction"}
    if (set(parameters) & motion_keys) - allowed_motion[protocol]:
        raise ConfigurationRefused("PARAMETER_NOT_APPLICABLE_TO_PROTOCOL")
    if protocol in ("cmj", "sj") and parameters.get("load_mass_kg", 0) != 0:
        raise ConfigurationRefused("UNLOADED_PROTOCOL_WITH_LOAD")
    if protocol not in ("cmj", "sj", "guided_vertical") and parameters.get("load_mass_kg", 1) <= 0:
        raise ConfigurationRefused("POSITIVE_LOAD_REQUIRED")
    return dict(parameters)
