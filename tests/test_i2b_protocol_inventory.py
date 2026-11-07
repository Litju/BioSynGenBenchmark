from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from biosyngen_benchmark.protocol_inventory import (
    CLAIM_CEILINGS,
    load_inventory,
    validate_inventory,
)


class ProtocolInventoryTests(unittest.TestCase):
    def test_pinned_inventory_loads_and_source_access_is_read_only(self) -> None:
        inventory = load_inventory()
        self.assertEqual(inventory["source_snapshot"]["commit"], "74ec9520b005fb1096c1afe89e14043455725636")
        self.assertEqual(inventory["source_access"]["source_mutations"], 0)
        self.assertEqual(inventory["source_access"]["source_tests_executed"], 0)
        self.assertEqual(len(inventory["source_access"]["evidence_files"]), 56)
        self.assertEqual(set(inventory["claim_ceilings"]), CLAIM_CEILINGS)

    def test_catalog_layers_keep_routes_movement_and_cmj_variants_distinct(self) -> None:
        inventory = load_inventory()
        catalogs = inventory["catalog_layers"]
        routes = catalogs["strength_protocol_router"]["routes"]
        self.assertEqual(len(routes), 7)
        self.assertEqual(len({route["protocol_id"] for route in routes}), 6)
        self.assertEqual(
            catalogs["movement_registry"]["exercise_ids"],
            [
                "BACK_SQUAT",
                "COUNTERMOVEMENT_JUMP",
                "SQUAT_JUMP",
                "CONVENTIONAL_DEADLIFT",
                "POWER_CLEAN",
                "BENCH_PRESS",
            ],
        )
        self.assertFalse(catalogs["cmj_domain"]["canonical_variant_catalog_present"])
        self.assertTrue(catalogs["cmj_domain"]["test_fixture_variant_ids"])

    def test_each_exercise_row_has_physics_and_sensor_boundaries(self) -> None:
        inventory = load_inventory()
        self.assertEqual(len(inventory["exercise_matrix"]), 11)
        for row in inventory["exercise_matrix"]:
            with self.subTest(exercise=row["exercise_id"]):
                self.assertEqual(row["degrees_of_freedom"], "NOT_DECLARED_BY_PLATE2BAR_MEASUREMENT_SYSTEM")
                self.assertTrue(row["clock_contract"])
                self.assertTrue(row["physics_owner_or_acceptance_criterion"])
                self.assertTrue(row["source_identities"])
                self.assertEqual(set(row["claim_ceilings"]), CLAIM_CEILINGS)
        self.assertEqual(len(inventory["deferred_protocol_candidates"]), 6)
        self.assertTrue(
            all(
                row["implementation_status"] == "NOT_IMPLEMENTED" and row["physical_requirement"]
                for row in inventory["deferred_protocol_candidates"]
            )
        )

    def test_protocol_event_phase_and_metric_identities_are_pinned(self) -> None:
        inventory = load_inventory()
        by_id = {row["exercise_id"]: row for row in inventory["exercise_matrix"]}
        jump_squat = by_id["LOADED_JUMP_SQUAT"]
        self.assertEqual(len(jump_squat["event_ids"]), 6)
        self.assertEqual(len(jump_squat["required_phase_ids"]), 3)
        self.assertEqual(len(jump_squat["optional_phase_ids"]), 2)
        self.assertEqual(len(jump_squat["authorized_metric_ids"]), 12)
        self.assertIn("BENCH_PRESS_FOOT_FORCE_TOTAL_MEAN_V1", by_id["BENCH_PRESS"]["authorized_metric_ids"])
        self.assertNotIn("BENCH_PRESS_FORCE_TOTAL_MEAN_V1", by_id["BENCH_PRESS"]["authorized_metric_ids"])
        self.assertTrue(by_id["COUNTERMOVEMENT_JUMP"]["event_phase_catalog_state"].startswith("OPEN_VERSIONED"))
        self.assertEqual(len(by_id["COUNTERMOVEMENT_JUMP"]["movement_registry_event_ids"]), 8)
        self.assertEqual(len(by_id["SQUAT_JUMP"]["movement_registry_phase_ids"]), 7)

    def test_governance_registry_name_gaps_are_explicit(self) -> None:
        inventory = load_inventory()
        registry = inventory["catalog_layers"]["governance_test_registry"]
        self.assertEqual(registry["record_count"], 12)
        self.assertEqual(registry["missing_registered_functions"], ["TST-ML51-LV-06"])
        self.assertEqual(registry["registered_function_renames"], ["TST-ML51-LV-05", "TST-ML51-LV-12"])
        self.assertEqual(
            registry["unregistered_current_functions"],
            ["test_ml51_claim_requirement_trace_is_reciprocal_and_audited"],
        )

    def test_completeness_and_claim_mutations_fail_closed(self) -> None:
        mutations = [
            (
                lambda doc: doc["catalog_layers"]["strength_protocol_router"]["routes"].pop(),
                "strength route keys are incomplete",
            ),
            (
                lambda doc: doc["catalog_layers"]["governance_test_registry"]["tests"].pop(),
                "registered governance test IDs are incomplete",
            ),
            (
                lambda doc: doc["exercise_matrix"].pop(),
                "exercise matrix rows are incomplete",
            ),
            (
                lambda doc: doc["exercise_matrix"][0]["claim_ceilings"].append("BIOLOGICALLY_VALIDATED"),
                "claim ceiling changed",
            ),
            (
                lambda doc: doc["source_access"].__setitem__("source_tests_executed", 1),
                "Plate2bar tests must not be executed",
            ),
            (
                lambda doc: doc["test_inventory"]["focused_source_test_groups"]["protocol_and_cmj"]["files"][0]["methods"].pop(),
                "test count mismatch",
            ),
        ]
        for mutate, expected in mutations:
            with self.subTest(expected=expected):
                inventory = deepcopy(load_inventory())
                mutate(inventory)
                errors = validate_inventory(inventory)
                self.assertTrue(any(expected in error for error in errors), errors)

    def test_malformed_claim_arrays_fail_without_crashing(self) -> None:
        inventory = deepcopy(load_inventory())
        inventory["exercise_matrix"][0]["claim_ceilings"].append([])
        errors = validate_inventory(inventory)
        self.assertIn("BACK_SQUAT claim ceilings must be text identities", errors)

    def test_loader_rejects_duplicate_json_keys_and_nonfinite_constants(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            duplicate = Path(directory) / "duplicate.json"
            duplicate.write_text('{"schema_version":"1.0.0","schema_version":"1.0.1"}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
                load_inventory(duplicate)
            nonfinite = Path(directory) / "nonfinite.json"
            nonfinite.write_text('{"value":NaN}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-finite"):
                load_inventory(nonfinite)

    def test_known_aliases_do_not_turn_into_unobserved_physics(self) -> None:
        inventory = load_inventory()
        rows = {row["exercise_id"]: row for row in inventory["exercise_matrix"]}
        self.assertIn("front-rack", rows["POWER_CLEAN"]["impact_and_contact"].lower())
        self.assertTrue(any("chest" in text.lower() for text in rows["BENCH_PRESS"]["negative_controls"]))
        self.assertEqual(rows["CLEAN_PULL"]["protocol_ids"], ["OLYMPIC_PULL_DERIVATIVE_DUAL_V1"])
        self.assertEqual(rows["SNATCH_PULL"]["protocol_ids"], ["OLYMPIC_PULL_DERIVATIVE_DUAL_V1"])
        self.assertEqual(rows["LOADED_CMJ"]["route_keys"], [])


if __name__ == "__main__":
    unittest.main()
