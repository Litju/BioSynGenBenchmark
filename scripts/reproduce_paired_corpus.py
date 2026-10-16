"""Fresh-process generator replay; compare every SOURCE/TRUTH/diagnostic byte."""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--count", type=int, default=16)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=False)
    inventories = []
    for replica in ("a", "b"):
        root = args.out / replica
        subprocess.run([sys.executable, "-m", "scripts.generate_paired_corpus", "--config",
            "docs/i3/RES524_CORPUS_CONFIG.json", "--protocol", "all", "--count", str(args.count),
            "--seed", "52416", "--max-bytes", "1073741824", "--out", str(root)], check=True)
        subprocess.run([sys.executable, "-m", "scripts.verify_paired_corpus", "--root", str(root),
            "--out", str(args.out / f"verification-{replica}.json")], check=True)
        inventories.append(json.loads((root / "inventory.json").read_text()))
    if inventories[0] != inventories[1]:
        raise ValueError("FRESH_PROCESS_INVENTORY_OR_SHA_MISMATCH")
    receipt = {"matching_files": len(inventories[0]), "replicas": 2, "cases_per_replica": args.count,
               "fresh_process_reproduction": "PASS", "inventory": inventories[0]}
    with (args.out / "replay.json").open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(receipt, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({k: v for k, v in receipt.items() if k != "inventory"}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
