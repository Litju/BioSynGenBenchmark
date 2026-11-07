"""Read and validate the pinned RES-490 protocol capability inventory.

This module validates the committed inventory snapshot offline. It does not
import Plate2bar, fetch external source, execute external tests, or authorize
scientific computations.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping


INVENTORY_RELATIVE_PATH = Path("docs") / "i2" / "RES490_PROTOCOL_TEST_CAPABILITY_INVENTORY_V1.json"
SOURCE_COMMIT = "74ec9520b005fb1096c1afe89e14043455725636"
CLAIM_CEILINGS = frozenset(
    {
        "NUMERICAL_VERIFICATION_ONLY",
        "SIMULATED_MEASUREMENT_ONLY",
        "BIOLOGICAL_VALIDATION=NOT_ESTABLISHED",
        "REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED",
    }
)
EXPECTED_MOVEMENT_FAMILIES = frozenset(
    {
        "CONTINUOUS_LOADED_SQUAT",
        "BALLISTIC_VERTICAL",
        "GROUND_CONSTRAINED_PULL",
        "WEIGHTLIFTING_PULL_CATCH",
        "SUPPORTED_UPPER_BODY_PRESS",
    }
)
EXPECTED_MOVEMENT_EXERCISES = frozenset(
    {
        "BACK_SQUAT",
        "COUNTERMOVEMENT_JUMP",
        "SQUAT_JUMP",
        "CONVENTIONAL_DEADLIFT",
        "POWER_CLEAN",
        "BENCH_PRESS",
    }
)
EXPECTED_STRENGTH_ROUTES = frozenset(
    {
        "BACK_SQUAT",
        "LOADED_JUMP_SQUAT",
        "DEADLIFT",
        "BENCH_PRESS",
        "POWER_CLEAN",
        "OLYMPIC_PULL_DERIVATIVE",
        "CLEAN_SNATCH_VARIANTS",
    }
)
EXPECTED_EXERCISE_ROWS = frozenset(
    {
        "BACK_SQUAT",
        "COUNTERMOVEMENT_JUMP",
        "SQUAT_JUMP",
        "LOADED_CMJ",
        "LOADED_JUMP_SQUAT",
        "DEADLIFT",
        "BENCH_PRESS",
        "OLYMPIC_PULL_DERIVATIVE",
        "CLEAN_PULL",
        "SNATCH_PULL",
        "POWER_CLEAN",
    }
)
EXPECTED_DEFERRED = frozenset(
    {
        "FULL_CLEAN",
        "FULL_SNATCH",
        "HANG_CLEAN",
        "HANG_SNATCH",
        "HANG_POWER_CLEAN",
        "HANG_POWER_SNATCH",
    }
)
EXPECTED_GOVERNANCE_TEST_IDS = frozenset(
    f"TST-ML51-LV-{number:02d}" for number in range(1, 13)
)
EXPECTED_GOVERNANCE_MISSING = frozenset({"TST-ML51-LV-06"})
EXPECTED_GOVERNANCE_RENAMED = frozenset({"TST-ML51-LV-05", "TST-ML51-LV-12"})
EXPECTED_TEST_GROUP_COUNTS = {
    "protocol_and_cmj": 66,
    "movement_semantics": 29,
    "sensor_time_and_context": 128,
}
EXPECTED_TEST_GROUP_FILES = {
    "protocol_and_cmj": frozenset(
        {
            "tests/back_squat/test_ml415_back_squat_dual_v1.py",
            "tests/protocol/test_back_squat_presentation.py",
            "tests/loaded_jump_squat/test_ml416_loaded_jump_squat.py",
            "tests/deadlift/test_ml511_deadlift_dual_v1.py",
            "tests/bench_press/test_ml513_bench_press_dual_v1.py",
            "tests/olympic_pull/test_ml417_olympic_pull_derivative.py",
            "tests/olympic_pull/test_ml417_clean_snatch_variants.py",
            "tests/power_clean/test_ml417_power_clean.py",
            "tests/measurement/test_ml579_cmj_domain.py",
            "tests/protocol/test_ml579_cmj_protocol_contract.py",
            "tests/api/test_ml579_cmj_domain_model_api.py",
        }
    ),
    "movement_semantics": frozenset(
        {
            "tests/reviewer/ml47/test_movement_domain_v1.py",
            "tests/reviewer/ml47/test_movement_truth_authority_wave1_v1.py",
            "tests/reviewer/ml47/test_movement_truth_hardening_v2.py",
        }
    ),
    "sensor_time_and_context": frozenset(
        {
            "tests/native/test_ml416_contact_state.py",
            "tests/measurement/test_ml412_coupled_trial.py",
            "tests/measurement/test_ml540_time_normalization.py",
            "tests/measurement/test_ml410_ml411_component_pipeline.py",
            "tests/measurement/test_ml411_lpt_component_pipeline.py",
            "tests/native/test_ml313_lpt_kinematics.py",
            "tests/measurement/test_ml413_event01.py",
            "tests/native/test_ml314_processing_profiles.py",
            "tests/measurement/test_ml541_raw_source_evidence.py",
            "tests/measurement/test_measurement_context.py",
            "tests/protocol/test_shared_load_value.py",
            "tests/governance/test_ml51_lightweight_acceptance.py",
        }
    ),
}
ROW_REQUIRED_FIELDS = frozenset(
    {
        "exercise_id",
        "context_of_use",
        "degrees_of_freedom",
        "body_load_support_topology",
        "impact_and_contact",
        "actuator_hypothesis",
        "primary_latent_event_roles",
        "observable_sensor_events",
        "event_ids",
        "required_phase_ids",
        "optional_phase_ids",
        "event_phase_catalog_state",
        "force_stage",
        "lpt_stage",
        "clock_contract",
        "authorized_metric_ids",
        "candidate_metric_ids",
        "metric_boundary",
        "positive_controls",
        "negative_controls",
        "source_identities",
        "physics_owner_or_acceptance_criterion",
        "implementation_status",
        "claim_ceilings",
    }
)


class InventoryValidationError(ValueError):
    """The inventory is malformed or fails its frozen completeness contract."""


def _reject_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON constant is forbidden: {value}")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def load_inventory(path: str | Path | None = None) -> dict[str, Any]:
    """Load and validate the committed inventory without network access."""
    source = Path(path) if path is not None else Path(__file__).resolve().parents[1] / INVENTORY_RELATIVE_PATH
    with source.open("r", encoding="utf-8") as stream:
        data = json.load(
            stream,
            object_pairs_hook=_unique_object,
            parse_constant=_reject_constant,
        )
    errors = validate_inventory(data)
    if errors:
        raise InventoryValidationError("; ".join(errors))
    return data


def validate_inventory(data: object) -> list[str]:
    """Return all completeness/schema violations for an inventory mapping."""
    errors: list[str] = []
    if not isinstance(data, Mapping):
        return ["root must be a JSON object"]

    def require(condition: bool, message: str) -> None:
        if not condition:
            errors.append(message)

    require(data.get("schema_version") == "1.0.0", "unsupported schema_version")
    require(
        data.get("inventory_id") == "BIOSYNGEN-RES490-PLATE2BAR-CAPABILITY-INVENTORY",
        "inventory_id mismatch",
    )
    snapshot = data.get("source_snapshot")
    require(isinstance(snapshot, Mapping), "source_snapshot must be an object")
    if isinstance(snapshot, Mapping):
        require(snapshot.get("repository") == "Litju/plate2bar-instrument", "source repository mismatch")
        require(snapshot.get("commit") == SOURCE_COMMIT, "source commit mismatch")
        require(
            snapshot.get("tree_sha") == "eb604b88cebb3bee097bda0b5e8f7489427ad2e0",
            "source tree mismatch",
        )

    access = data.get("source_access")
    require(isinstance(access, Mapping), "source_access must be an object")
    evidence_paths: set[str] = set()
    if isinstance(access, Mapping):
        require(access.get("mode") == "READ_ONLY_GITHUB_GET", "source access must remain read-only")
        require(access.get("source_mutations") == 0, "Plate2bar source mutation was recorded")
        require(access.get("source_tests_executed") == 0, "Plate2bar tests must not be executed")
        require(access.get("local_clone_created") is False, "source clone is outside this audit contract")
        evidence = access.get("evidence_files")
        require(isinstance(evidence, list) and bool(evidence), "source evidence file list is required")
        if isinstance(evidence, list):
            for item in evidence:
                if not isinstance(item, Mapping):
                    errors.append("source evidence entries must be objects")
                    continue
                path = item.get("path")
                blob_sha = item.get("blob_sha")
                if isinstance(path, str):
                    if path in evidence_paths:
                        errors.append(f"duplicate source evidence path: {path}")
                    evidence_paths.add(path)
                else:
                    errors.append("source evidence path must be text")
                require(
                    isinstance(blob_sha, str)
                    and len(blob_sha) == 40
                    and all(ch in "0123456789abcdef" for ch in blob_sha),
                    f"invalid source blob SHA for {path!r}",
                )
                require(
                    isinstance(item.get("bytes"), int) and item.get("bytes", -1) >= 0,
                    f"invalid source byte length for {path!r}",
                )

    claims = data.get("claim_ceilings")
    require(isinstance(claims, list), "claim_ceilings must be an array")
    if isinstance(claims, list):
        require(all(isinstance(item, str) for item in claims), "claim ceilings must be text identities")
        if all(isinstance(item, str) for item in claims):
            require(set(claims) == CLAIM_CEILINGS, "claim ceiling set changed or contains an unapproved claim")

    catalogs = data.get("catalog_layers")
    require(isinstance(catalogs, Mapping), "catalog_layers must be an object")
    routes: list[Mapping[str, Any]] = []
    if isinstance(catalogs, Mapping):
        movement = catalogs.get("movement_registry")
        require(isinstance(movement, Mapping), "movement_registry layer missing")
        if isinstance(movement, Mapping):
            families = movement.get("families")
            require(isinstance(families, list), "movement families must be an array")
            if isinstance(families, list):
                family_ids = [x.get("family_id") for x in families if isinstance(x, Mapping)]
                require(all(isinstance(item, str) for item in family_ids), "movement family IDs must be text")
                if all(isinstance(item, str) for item in family_ids):
                    require(set(family_ids) == EXPECTED_MOVEMENT_FAMILIES, "movement family inventory is incomplete")
                    require(len(family_ids) == len(set(family_ids)), "duplicate movement family")
            movement_ids = movement.get("exercise_ids")
            require(isinstance(movement_ids, list), "movement exercise IDs must be an array")
            if isinstance(movement_ids, list) and all(isinstance(item, str) for item in movement_ids):
                require(set(movement_ids) == EXPECTED_MOVEMENT_EXERCISES, "movement exercise inventory is incomplete")
            require(movement.get("family_count") == 5, "movement family count mismatch")
            require(movement.get("exercise_count") == 6, "movement exercise count mismatch")

        router = catalogs.get("strength_protocol_router")
        require(isinstance(router, Mapping), "strength protocol router layer missing")
        if isinstance(router, Mapping):
            routes_value = router.get("routes")
            require(isinstance(routes_value, list), "routes must be an array")
            if isinstance(routes_value, list):
                routes = [x for x in routes_value if isinstance(x, Mapping)]
                route_keys = [x.get("route_key") for x in routes]
                require(all(isinstance(item, str) for item in route_keys), "route keys must be text")
                if all(isinstance(item, str) for item in route_keys):
                    require(set(route_keys) == EXPECTED_STRENGTH_ROUTES, "strength route keys are incomplete")
                    require(len(route_keys) == len(set(route_keys)), "duplicate strength route key")
                protocol_ids = [x.get("protocol_id") for x in routes]
                require(all(isinstance(x, str) and x for x in protocol_ids), "route lacks protocol_id")
                require(router.get("route_key_count") == 7, "strength route count mismatch")
                require(router.get("unique_protocol_id_count") == len(set(protocol_ids)) == 6, "unique protocol ID count mismatch")

        cmj = catalogs.get("cmj_domain")
        require(isinstance(cmj, Mapping), "CMJ domain layer missing")
        if isinstance(cmj, Mapping):
            require(cmj.get("conditions") == ["UNLOADED", "LOADED"], "CMJ condition identities changed")
            require(cmj.get("acquisition_modes") == ["FORCE_ONLY", "LPT_ONLY", "PAIRED"], "CMJ acquisition modes changed")
            require(cmj.get("canonical_variant_catalog_present") is False, "CMJ fixture variants were promoted to canonical")
            require(cmj.get("metric_graph") == "CMJ_METRIC_PREREQUISITES@1.0.0", "CMJ metric graph identity mismatch")

        protocol_registry = catalogs.get("generic_protocol_registry")
        require(isinstance(protocol_registry, Mapping), "generic protocol registry layer missing")

        governance = catalogs.get("governance_test_registry")
        require(isinstance(governance, Mapping), "governance test registry layer missing")
        if isinstance(governance, Mapping):
            tests = governance.get("tests")
            require(isinstance(tests, list), "governance tests must be an array")
            if isinstance(tests, list):
                test_ids = [x.get("test_id") for x in tests if isinstance(x, Mapping)]
                require(all(isinstance(item, str) for item in test_ids), "governance test IDs must be text")
                if all(isinstance(item, str) for item in test_ids):
                    require(set(test_ids) == EXPECTED_GOVERNANCE_TEST_IDS, "registered governance test IDs are incomplete")
                    require(len(test_ids) == len(set(test_ids)), "duplicate governance test ID")
                missing = {
                    x.get("test_id")
                    for x in tests
                    if isinstance(x, Mapping)
                    and isinstance(x.get("test_id"), str)
                    and x.get("identity_status") == "MISSING_CURRENT_SOURCE_FUNCTION"
                }
                renamed = {
                    x.get("test_id")
                    for x in tests
                    if isinstance(x, Mapping)
                    and isinstance(x.get("test_id"), str)
                    and x.get("identity_status") == "REGISTERED_FUNCTION_RENAMED"
                }
                require(missing == EXPECTED_GOVERNANCE_MISSING, "governance missing-function discrepancy changed")
                require(renamed == EXPECTED_GOVERNANCE_RENAMED, "governance renamed-function discrepancy changed")
                require(governance.get("unregistered_current_functions") == ["test_ml51_claim_requirement_trace_is_reciprocal_and_audited"], "unregistered governance function discrepancy changed")
                require(governance.get("record_count") == 12, "governance registry record count mismatch")
                require(governance.get("status") == "CONTROLLED_DRAFT", "governance registry status mismatch")

    rows = data.get("exercise_matrix")
    require(isinstance(rows, list), "exercise_matrix must be an array")
    row_by_id: dict[str, Mapping[str, Any]] = {}
    if isinstance(rows, list):
        for row in rows:
            if not isinstance(row, Mapping):
                errors.append("exercise rows must be objects")
                continue
            exercise_id = row.get("exercise_id")
            if not isinstance(exercise_id, str) or not exercise_id:
                errors.append("exercise_id is required")
                continue
            if exercise_id in row_by_id:
                errors.append(f"duplicate exercise row: {exercise_id}")
            row_by_id[exercise_id] = row
            missing_fields = ROW_REQUIRED_FIELDS - set(row)
            require(not missing_fields, f"{exercise_id} missing fields: {sorted(missing_fields)}")
            require(row.get("degrees_of_freedom") == "NOT_DECLARED_BY_PLATE2BAR_MEASUREMENT_SYSTEM", f"{exercise_id} invents source DOFs")
            require(row.get("actuator_hypothesis") == "NOT_IMPLEMENTED_BY_PLATE2BAR; no biological or device actuator parameters are defined.", f"{exercise_id} invents actuator parameters")
            require(isinstance(row.get("context_of_use"), str) and bool(row.get("context_of_use")), f"{exercise_id} lacks COU")
            require(isinstance(row.get("physics_owner_or_acceptance_criterion"), str) and bool(row.get("physics_owner_or_acceptance_criterion")), f"{exercise_id} lacks a physics owner/criterion")
            row_claims = row.get("claim_ceilings")
            require(isinstance(row_claims, list), f"{exercise_id} claim ceilings must be an array")
            if isinstance(row_claims, list) and all(isinstance(item, str) for item in row_claims):
                require(set(row_claims) == CLAIM_CEILINGS, f"{exercise_id} claim ceiling changed")
            elif isinstance(row_claims, list):
                errors.append(f"{exercise_id} claim ceilings must be text identities")
            require(isinstance(row.get("event_phase_catalog_state"), str) and bool(row.get("event_phase_catalog_state")), f"{exercise_id} lacks event/phase catalog status")
            for field in (
                "aliases", "route_keys", "movement_registry_ids", "protocol_ids", "condition_ids",
                "variant_ids", "primary_latent_event_roles", "observable_sensor_events", "event_ids",
                "required_phase_ids", "optional_phase_ids", "authorized_metric_ids", "candidate_metric_ids",
                "positive_controls", "negative_controls", "source_identities",
            ):
                value = row.get(field)
                require(isinstance(value, list), f"{exercise_id}.{field} must be an array")
                if isinstance(value, list):
                    require(all(isinstance(item, str) and item for item in value), f"{exercise_id}.{field} contains an empty/non-text identity")
            source_ids = row.get("source_identities")
            for source_id in source_ids if isinstance(source_ids, list) else []:
                if isinstance(source_id, str):
                    require(source_id in evidence_paths, f"{exercise_id} references unpinned source {source_id!r}")
            row_routes = row.get("route_keys")
            for route_key in row_routes if isinstance(row_routes, list) else []:
                if isinstance(route_key, str):
                    require(route_key in {route.get("route_key") for route in routes}, f"{exercise_id} references unknown route {route_key!r}")
    require(set(row_by_id) == EXPECTED_EXERCISE_ROWS, "exercise matrix rows are incomplete or contain unreviewed rows")

    deferred = data.get("deferred_protocol_candidates")
    require(isinstance(deferred, list), "deferred_protocol_candidates must be an array")
    if isinstance(deferred, list):
        deferred_ids = [x.get("exercise_id") for x in deferred if isinstance(x, Mapping)]
        require(all(isinstance(item, str) for item in deferred_ids), "deferred exercise IDs must be text")
        if all(isinstance(item, str) for item in deferred_ids):
            require(set(deferred_ids) == EXPECTED_DEFERRED, "deferred protocol candidates are incomplete")
            require(len(deferred_ids) == len(set(deferred_ids)), "duplicate deferred candidate")
        for item in deferred:
            if isinstance(item, Mapping):
                require(item.get("implementation_status") == "NOT_IMPLEMENTED", "deferred candidate must remain NOT_IMPLEMENTED")
                require(bool(item.get("physical_requirement")), "deferred candidate lacks a physical observability requirement")

    sensors = data.get("sensor_capabilities")
    require(isinstance(sensors, list) and len(sensors) == 4, "sensor capability inventory is incomplete")
    if isinstance(sensors, list):
        sensor_ids = [x.get("sensor_id") for x in sensors if isinstance(x, Mapping)]
        require(all(isinstance(item, str) for item in sensor_ids), "sensor IDs must be text")
        if all(isinstance(item, str) for item in sensor_ids):
            require(set(sensor_ids) == {"FORCE_PIPE_01", "LPT_PIPE_01", "COUPLED_TRIAL_TIME", "CONTACT_STATE"}, "sensor identities changed")

    test_inventory = data.get("test_inventory")
    require(isinstance(test_inventory, Mapping), "test_inventory missing")
    if isinstance(test_inventory, Mapping):
        focused = test_inventory.get("focused_source_test_groups")
        require(isinstance(focused, Mapping), "focused source test groups missing")
        if isinstance(focused, Mapping):
            require(set(focused) == set(EXPECTED_TEST_GROUP_COUNTS), "focused test group set changed")
            total = 0
            for group_name, expected_count in EXPECTED_TEST_GROUP_COUNTS.items():
                group = focused.get(group_name)
                require(isinstance(group, Mapping), f"test group {group_name} missing")
                if not isinstance(group, Mapping):
                    continue
                files = group.get("files")
                require(isinstance(files, list), f"test group {group_name} files missing")
                if not isinstance(files, list):
                    continue
                paths = [x.get("path") for x in files if isinstance(x, Mapping)]
                require(set(paths) == EXPECTED_TEST_GROUP_FILES[group_name], f"test module set incomplete for {group_name}")
                require(len(paths) == len(set(paths)), f"duplicate test module in {group_name}")
                group_count = 0
                for file in files:
                    if not isinstance(file, Mapping):
                        continue
                    methods = file.get("methods")
                    require(isinstance(methods, list) and bool(methods), f"test method inventory missing for {file.get('path')}")
                    if isinstance(methods, list):
                        require(all(isinstance(name, str) and name.startswith("test_") for name in methods), f"invalid test function name in {file.get('path')}")
                        require(file.get("test_count") == len(methods), f"test count mismatch for {file.get('path')}")
                        group_count += len(methods)
                require(group_count == expected_count, f"test method inventory count mismatch for {group_name}")
                require(group.get("test_count") == expected_count, f"test group count mismatch for {group_name}")
                total += group_count
            summary = data.get("count_summary")
            require(isinstance(summary, Mapping), "count_summary missing")
            if isinstance(summary, Mapping):
                require(summary.get("focused_source_test_function_count") == total == 223, "focused source test total mismatch")
                require(summary.get("exercise_row_count") == len(EXPECTED_EXERCISE_ROWS), "exercise row summary mismatch")
        gov_inventory = test_inventory.get("governance_registry")
        require(isinstance(gov_inventory, Mapping), "governance test inventory missing")
        if isinstance(gov_inventory, Mapping):
            gov_tests = gov_inventory.get("tests")
            require(isinstance(gov_tests, list), "governance test inventory rows missing")
            ids = [x.get("test_id") for x in gov_tests if isinstance(x, Mapping)] if isinstance(gov_tests, list) else []
            require(all(isinstance(item, str) for item in ids), "governance inventory test IDs must be text")
            if all(isinstance(item, str) for item in ids):
                require(set(ids) == EXPECTED_GOVERNANCE_TEST_IDS, "governance test inventory lost a registered test")

    discrepancies = data.get("catalog_discrepancies")
    require(isinstance(discrepancies, list) and len(discrepancies) >= 10, "catalog discrepancy inventory is incomplete")
    for field in ("protocol_control_dimensions", "reproduction", "authority_separation"):
        require(isinstance(data.get(field), Mapping), f"{field} is required")
    return errors
