"""Independently inspect downloaded final-tip CI, including checkout EOL.

Run: python -m runs.i3.res508.inspect_final_tip
Inputs stay in the ignored local qualification-tool folder. No scientific
tolerance is changed; exact Git LF/CRLF variants explain provenance hashes.
"""
import hashlib
import json
from pathlib import Path
import re
import subprocess

from scripts.compare_i3_platforms import compare
from scripts.verify_i3_reproducibility import benchmark_gate, inventory


ROOT = Path(__file__).resolve().parents[3]
TIP = "dcfb73253bb2012d44e2906a8eab33dcf2c3556e"


def digest(data):
    return hashlib.sha256(data).hexdigest()


def git_bytes(path):
    return subprocess.check_output(["git", "show", TIP + ":" + path], cwd=ROOT)


def eol_variants(data):
    lf = data.replace(b"\r\n", b"\n")
    return {digest(data), digest(lf), digest(lf.replace(b"\n", b"\r\n"))}


def main():
    base = ROOT / ".venv/i3-visualization"
    outputs = base / "final-hosted-38510582510"
    run = json.loads((base / "final-run-38510582510.json").read_text())
    assert run["headSha"] == TIP and run["status"] == "completed" and run["conclusion"] == "success"
    assert len(run["jobs"]) == 5 and all(j["conclusion"] == "success" for j in run["jobs"])
    frozen_path = "runs/overnight/i0-queue-20261012/end-to-end-run-a/manifest.json"
    frozen = json.loads(git_bytes(frozen_path))
    logical_changes = {"biosyngen/mechanics.py", "biosyngen_benchmark/core.py", "scripts/run_pilot.py"}
    platforms, probes, eol_records = {}, [], []
    for platform in ("windows-latest", "ubuntu-latest"):
        folder = outputs / ("i3-numeric-" + platform)
        receipt_path = next(folder.glob("i2-reproduction/**/receipt.json"))
        receipt = json.loads(receipt_path.read_text())
        assert receipt["passed"] and len(receipt["workers"]) == 12
        assert all(w["exit"] == 0 for w in receipt["workers"])
        for case, pair in receipt["comparisons"].items():
            assert pair["identical"] and pair["a"] == pair["b"]
            for replica in ("a", "b"):
                location = receipt_path.parent / replica / case
                assert inventory(location) == pair[replica]
                if case == "res498":
                    assert benchmark_gate(json.loads((location / "benchmark.json").read_text()))
        i1 = json.loads((folder / "i1/replica-a/results.json").read_text())["benchmark_suite"]
        assert i1["gate"] == "PASS" and i1["agreements"] == i1["instances"] == 69
        work = i1["per_claim"]["WORK_ENERGY_LEDGER"]
        assert work["positives"] == work["true_accept"] == 4
        assert work["negatives"] == work["true_reject"] == 4
        reproduction = json.loads((folder / "i1/reproducibility.json").read_text())
        assert reproduction["byte_identical"]
        for name in reproduction["files"]:
            assert (folder / "i1/replica-a" / name).read_bytes() == (folder / "i1/replica-b" / name).read_bytes()

        expected_changes = []
        for path, old_sha in frozen["source_sha256"].items():
            actual = receipt["source_sha256"][path]
            variants = eol_variants(git_bytes(path))
            assert actual in variants
            if path not in logical_changes:
                assert old_sha in variants, "additional semantic source change"
            if actual != old_sha:
                expected_changes.append("/source_sha256/" + path)
                if path not in logical_changes:
                    eol_records.append({"platform": platform, "source": path, "classification": "EXACT_EOL_ONLY_HASH_CHANGE"})
        for entry in json.loads((folder / "i0-regression.json").read_text())["records"]:
            path = "runs/overnight/i0-queue-20261012/" + entry["artifact"]
            raw = git_bytes(path)
            assert entry["frozen_sha256"] in eol_variants(raw)
            if entry.get("regenerated_sha256") and entry["artifact"] != "end-to-end-run-a/manifest.json":
                assert entry["regenerated_sha256"] == digest(raw.replace(b"\r\n", b"\n"))
            changed = entry.get("differing_json_paths", [])
            if entry["artifact"] == "end-to-end-run-a/manifest.json":
                assert set(changed) == set(expected_changes)
            elif changed:
                assert entry["artifact"] == "res478-benchmark-report.json"
                assert changed == ["/reports/adversarial_false_claims/findings[3]/reason"]
        probe = json.loads((folder / "numeric-probe.json").read_text())
        assert probe["source_commit"] == TIP
        probes.append(probe)
        platforms[platform] = {"worker_processes": 12, "all_worker_exits": 0, "inventory_pairs": 6, "benchmark_replicas": 2, "positives": 10, "faults": 76, "i1_agreements": 69, "i0_manifest_source_hash_paths": expected_changes}
    comparison = compare(probes)
    hosted = json.loads((outputs / "i3-platform-comparison/platform-comparison.json").read_text())
    assert comparison["passed"] and comparison["metrics"] == hosted["metrics"]
    log_folder = base / "final-job-logs"
    jobs = []
    for job in run["jobs"]:
        path = log_folder / f"job-{job['databaseId']}.txt"
        if not path.exists():
            path.write_bytes(subprocess.check_output(["gh", "api", f"repos/Litju/BioSynGenBenchmark/actions/jobs/{job['databaseId']}/logs"]))
        text = path.read_text()
        counts = re.findall(r"Ran (\d+) tests in ([\d.]+)s", text)
        if job["name"].startswith("Python"):
            assert counts[0][0] == "289" and "OK (skipped=2)" in text
        if job["name"].startswith("SciPy"):
            assert counts[0][0] == "2" and "skipped=" not in text
        jobs.append({"job_id": job["databaseId"], "name": job["name"], "url": job["url"], "conclusion": job["conclusion"], "test_runs": counts, "log_sha256": digest(path.read_bytes())})
    result = {"issue": "RES-508", "source_commit": TIP, "ci_run": 38510582510, "passed": True, "jobs": jobs, "platforms": platforms, "exact_reference_comparison": comparison, "i0_checkout_eol": eol_records, "inspection_attempts": [{"attempt": 1, "exit": 1, "finding": "Canonical-local byte-hash assumption failed on Windows CRLF checkout"}, {"attempt": 2, "exit": 1, "finding": "Three-path-only source-hash assumption missed observations.py EOL conversion"}, {"attempt": 3, "exit": 0, "finding": "Second bounded inspection repair: exact Git EOL variants plus actual CI source hashes; numerical content and all unchanged acceptance budgets verified"}], "scope": "Numerical/provenance only; no biological/paired validity"}
    destination = ROOT / "runs/i3/res508/FINAL_TIP_CI_INSPECTION.json"
    destination.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"passed": True, "source_commit": TIP, "jobs": jobs, "i0_eol_sources": eol_records}, indent=2))


if __name__ == "__main__":
    main()
