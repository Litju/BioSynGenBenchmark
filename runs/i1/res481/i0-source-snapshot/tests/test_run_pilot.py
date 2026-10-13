import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree
import unittest

from scripts.run_pilot import build_run_artifacts


class IntegratedPilotTests(unittest.TestCase):
    def test_same_seed_builds_identical_artifact_bytes(self) -> None:
        first = build_run_artifacts()
        second = build_run_artifacts()
        self.assertEqual(first, second)

    def test_manifest_hashes_and_svg_accessibility_are_valid(self) -> None:
        artifacts = build_run_artifacts()
        manifest = json.loads(artifacts["manifest.json"])
        for name, expected_hash in manifest["artifact_sha256"].items():
            self.assertEqual(
                hashlib.sha256(artifacts[name]).hexdigest(),
                expected_hash,
            )
        source_hashes = manifest["source_sha256"]
        project_root = Path(__file__).resolve().parents[1]
        self.assertEqual(
            source_hashes["biosyngen_benchmark/core.py"],
            hashlib.sha256(
                (project_root / "biosyngen_benchmark" / "core.py").read_bytes()
            ).hexdigest(),
        )

        svg = ElementTree.fromstring(artifacts["pilot-figure.svg"])
        namespace = "{http://www.w3.org/2000/svg}"
        self.assertEqual(svg.tag, namespace + "svg")
        self.assertTrue(svg.findtext(namespace + "title"))
        description = svg.findtext(namespace + "desc")
        self.assertIn("missing samples", description)
        self.assertIn("not empirical validation", description)
        _, _, canvas_width, canvas_height = map(
            float, svg.attrib["viewBox"].split()
        )
        for text in svg.iter(namespace + "text"):
            self.assertGreaterEqual(float(text.attrib["x"]), 0.0)
            self.assertLessEqual(float(text.attrib["x"]), canvas_width)
            self.assertGreaterEqual(float(text.attrib["y"]), 0.0)
            self.assertLessEqual(float(text.attrib["y"]), canvas_height)
        visible_text = " ".join(
            element.text or ""
            for element in svg.iter(namespace + "text")
        )
        for label in ("Latent positions", "Total vertical force", "Native clock offset"):
            self.assertIn(label, visible_text)
        self.assertGreaterEqual(visible_text.count("Missing sample"), 3)

    def test_independent_report_covers_pass_reject_and_not_evaluable(self) -> None:
        artifacts = build_run_artifacts()
        report = json.loads(artifacts["evaluation-report.json"])["reports"]
        narrative = artifacts["pilot-report.md"].decode("utf-8")
        known = report["positive_known_answer"]["counts"]
        adversarial = report["inconsistent_impulse_control"]["findings"][0]
        missing = report["incomplete_evidence"]["counts"]
        self.assertEqual(known["admitted"], 3)
        self.assertEqual(adversarial["outcome"], "REJECTED")
        self.assertEqual(adversarial["code"], "INCONSISTENT_IMPULSE")
        self.assertEqual(missing["not_evaluable"], 5)
        self.assertIn("INCONSISTENT_IMPULSE: REJECTED", narrative)
        self.assertIn("Gaussian noise SD 15 N", narrative)

    def test_public_evaluator_input_does_not_include_latent_state_fields(self) -> None:
        artifacts = build_run_artifacts()
        public_input = json.loads(artifacts["evaluator-input.json"])
        serialized = json.dumps(public_input)
        self.assertNotIn("latent_mechanics", serialized)
        self.assertNotIn("hidden_truth", serialized)
        self.assertNotIn("latent_truth", serialized)


if __name__ == "__main__":
    unittest.main()
