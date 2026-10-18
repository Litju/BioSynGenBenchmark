"""Fresh-process generator replay; compare every SOURCE/TRUTH/diagnostic byte."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

# Versioned expected denominators of docs/i3/RES524_CORPUS_CONFIG.json at 16
# cases: every modeled reference qualifies, but one guide case deliberately
# refuses synchronization (unbounded sample jitter). Not 16 full-case trials.
EXPECTED_16 = {"model_reference_qualified_count": 16, "bounded_synchronization_count": 15,
    "synchronization_refused_count": 1, "full_case_qualified_count": 15, "evidence_refused_count": 0,
    "missing_evidence_count": 0, "not_evaluable_count": 0}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--count", type=int, default=16)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=False)
    inventories, denominators = [], []
    for hash_seed, replica in enumerate(("a", "b")):
        root = args.out / replica
        subprocess.run([sys.executable, "-m", "scripts.generate_paired_corpus", "--config",
            "docs/i3/RES524_CORPUS_CONFIG.json", "--protocol", "all", "--count", str(args.count),
            "--seed", "52416", "--max-bytes", "1073741824", "--out", str(root)], check=True,
            env={**os.environ, "PYTHONHASHSEED": str(hash_seed)})
        subprocess.run([sys.executable, "-m", "scripts.verify_paired_corpus", "--root", str(root),
            "--out", str(args.out / f"verification-{replica}.json")], check=True)
        inventories.append(json.loads((root / "inventory.json").read_text()))
        report = json.loads((args.out / f"verification-{replica}.json").read_text())
        denominators.append({k: v for k, v in report.items() if k.endswith("_count")})
    if inventories[0] != inventories[1]:
        raise ValueError("FRESH_PROCESS_INVENTORY_OR_SHA_MISMATCH")
    if denominators[0] != denominators[1] or (args.count == 16 and any(
            denominators[0][k] != v for k, v in EXPECTED_16.items())):
        raise ValueError("QUALIFICATION_DENOMINATORS_CHANGED")
    receipt = {"matching_files": len(inventories[0]), "replicas": 2, "cases_per_replica": args.count,
               "python_hash_seeds": [0, 1],
               "fresh_process_reproduction": "PASS", "qualification_denominators": denominators[0],
               "inventory": inventories[0]}
    with (args.out / "replay.json").open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(receipt, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({k: v for k, v in receipt.items() if k != "inventory"}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
