"""Independent canaries for publication screening; no real credentials/data."""
import unittest
from scripts.audit_i3_provenance import restricted, screen


class PublicationScreenTests(unittest.TestCase):
    def test_restricted_paths_remain_unread(self):
        for path in ("research/report.md", "tests/corpus/a.json", "data/holdouts/a.csv", ".env.local"):
            with self.subTest(path=path):
                self.assertTrue(restricted(path))
        self.assertFalse(restricted("runs/i1/res481/i0-source-snapshot/biosyngen/mechanics.py"))

    def test_independent_secret_canaries(self):
        cases = [(b"ghp_" + b"A" * 36, "github_token"),
                 (b"sk-proj-" + b"B" * 40, "provider_key"),
                 (b"AKIA" + b"C" * 16, "aws_key"),
                 (b"-----BEGIN " + b"PRIVATE KEY-----", "private_key")]
        for data, expected in cases:
            with self.subTest(expected=expected):
                self.assertIn(expected, screen(data))
        self.assertEqual(screen(b"ordinary source code without credentials"), [])

    def test_binary_and_machine_metadata_are_flagged(self):
        self.assertIn("binary_manual_review", screen(b"PNG\x00data"))
        self.assertIn("machine_path", screen(b"D:\\projects\\example"))


if __name__ == "__main__":
    unittest.main()
