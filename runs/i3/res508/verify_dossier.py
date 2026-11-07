"""Check final dossier metadata against actual Git, retained CI and CFF schema.

Run with the isolated metadata environment; this is document/provenance
verification, not a new scientific oracle or core runtime requirement.
"""
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import re
import subprocess
import sys
import urllib.request

import jsonschema
import yaml


ROOT = Path(__file__).resolve().parents[3]
NUMERICAL_SOURCE = "57484146cd12859dad34a004d21606cfb67bcd5f"
SCHEMA_URL = "https://raw.githubusercontent.com/citation-file-format/citation-file-format/1.2.0/schema.json"
SCHEMA_SHA = "0b8d22140da702d766df318dcff3a91af2f39521298dcf36d76315fd99cc169b"


def git(*arguments):
    return subprocess.check_output(["git", *arguments], cwd=ROOT)


def main():
    ledger = json.loads((ROOT / "runs/i3/res508/qualification-ledger.json").read_text())
    for row in ledger["commits"]:
        assert git("cat-file", "-t", row["sha"]).strip() == b"commit"
        subject = git("show", "-s", "--format=%s", row["sha"]).decode().strip()
        assert subject == row["subject"]
    assert not git("diff", NUMERICAL_SOURCE, "--", "biosyngen", "biosyngen_benchmark", "scripts", "tests", ".github")
    assert git("rev-parse", "claude/res-488-roadmap-r1").decode().strip() == "3322d89499050eea0004f9e8882666e2a40d5a9a"
    assert git("rev-parse", "main").decode().strip() == "6115d702ba3ab814982353945b0753f4846aae88"
    assert {r["id"] for r in ledger["i2_live_snapshot"]} == {f"RES-{i}" for i in range(489, 499)}
    assert all(r["status"] == "In Review" for r in ledger["i2_live_snapshot"])

    hosted = json.loads((ROOT / "runs/i3/res506/hosted-run-38506388914.json").read_text())
    assert hosted["headSha"] == NUMERICAL_SOURCE
    assert hosted["status"] == "completed" and hosted["conclusion"] == "success"
    assert len(hosted["jobs"]) == 5 and all(j["conclusion"] == "success" for j in hosted["jobs"])
    for filename in ("linux-qualified-job-115665088742.txt", "windows-qualified-job-115665088693.txt"):
        log = (ROOT / "runs/i3/res506" / filename).read_text()
        assert "Ran 289 tests" in log and "OK (skipped=2)" in log
    inspection = json.loads((ROOT / "runs/i3/res506/downloaded-evidence-inspection.json").read_text())
    for platform in inspection["platform_results"].values():
        assert platform["worker_processes"] == 12 and platform["all_worker_exits"] == 0
        assert platform["positive_controls"] == 10 and platform["fault_detections"] == 76
    assert inspection["recomputed_comparison"]["passed"]

    with urllib.request.urlopen(SCHEMA_URL, timeout=30) as response:
        schema_bytes = response.read()
    assert hashlib.sha256(schema_bytes).hexdigest() == SCHEMA_SHA
    citation = yaml.safe_load((ROOT / "CITATION.cff").read_text())
    jsonschema.validate(citation, json.loads(schema_bytes))
    assert citation["license"] == "Apache-2.0"
    assert hashlib.sha256((ROOT / "LICENSE").read_bytes()).hexdigest() == "cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30"

    documents = [ROOT / name for name in ("README.md", "AGENTS.md", "docs/ROADMAP_CONVERGENCE_STATUS.md", "docs/i3/RES508_OWNER_DOSSIER.md")]
    links = []
    for path in documents:
        content = path.read_text()
        for target in re.findall(r"\[[^\]]+\]\(([^)]+)\)", content):
            if target.startswith(("https://", "http://", "#")):
                continue
            destination = (path.parent / target.split("#", 1)[0]).resolve()
            assert destination.exists(), (str(path), target)
            links.append({"document": path.relative_to(ROOT).as_posix(), "target": target})
    dossier = documents[-1].read_text()
    for ceiling in ledger["claim_ceilings"]:
        assert ceiling in dossier
    assert len(ledger["scientific_matrix"]) == 10
    result = {
        "issue": "RES-508", "passed": True, "numerical_source": NUMERICAL_SOURCE,
        "checks": ["17 immutable issue commit subjects/types", "numerical source/workflow unchanged", "original I2/main refs preserved", "10 authenticated snapshot statuses", "five actual hosted successes and exact suite counts", "retained reproduction/benchmark denominators", "official pinned CFF schema", "selected Apache license digest", "all local Markdown links", "all claim ceilings and 10 scientific matrix rows"],
        "cff_schema_url": SCHEMA_URL, "cff_schema_sha256": SCHEMA_SHA,
        "PyYAML": yaml.__version__, "jsonschema": version("jsonschema"),
        "python": sys.version, "command": [sys.executable, *sys.argv], "exit": 0,
        "local_links": links,
        "scope": "document/provenance checks only; no new empirical/scientific validity inferred",
    }
    (ROOT / "runs/i3/res508/dossier-verification.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8", newline="\n")
    print("PASS: dossier Git/CI/licensing/citation/link checks; numerical source unchanged")


if __name__ == "__main__":
    main()
