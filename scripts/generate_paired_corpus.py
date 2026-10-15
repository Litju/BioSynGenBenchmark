"""Bounded development pilot: same-world native dual-1kHz sensor packages.

Bulk generation remains gated on production qualification. SOURCE is the only
directory to supply to the product reader; TRUTH is exclusively evaluator input.
"""

import argparse
import json
from pathlib import Path
import time
import tracemalloc

from biosyngen.i2.paired_export import export_case, write_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--seed", type=int, default=524)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--protocol", choices=("back_squat", "deadlift", "guided_vertical"), required=True)
    args = parser.parse_args(argv)
    config = {} if args.config is None else json.loads(args.config.read_text(encoding="utf-8"))
    if set(config) - {"schema_id", "purpose"} or (config and config.get("schema_id") != "BSMB-PILOT-CONFIG-1.0"):
        parser.error("unsupported configuration; no silent scientific parameter overrides")
    if not 1 <= args.count <= 3:
        parser.error("pilot count must be 1..3; bulk generation is not qualified")
    args.out.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    tracemalloc.start()
    cases = []
    for i in range(args.count):
        case_id = f"trial-{args.seed:08d}-{i:04d}"
        try:
            cases.append({"status": "GENERATED_NOT_QUALIFIED", **export_case(
                args.out, protocol=args.protocol, seed=args.seed + i, case_id=case_id)})
        except (ValueError, ArithmeticError) as exc:
            cases.append({"case_id": case_id, "status": "FAILED", "error": str(exc)})
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    elapsed = time.perf_counter() - started
    report = {"schema_id": "BSMB-PILOT-RUN-1.0", "config": config,
        "protocol": args.protocol, "seed": args.seed, "requested_count": args.count,
        "generated_count": sum(c["status"] == "GENERATED_NOT_QUALIFIED" for c in cases),
        "failed_count": sum(c["status"] == "FAILED" for c in cases), "cases": cases,
        "elapsed_s": elapsed, "python_peak_traced_bytes": peak,
        "memory_scope": "tracemalloc Python allocations; not process peak RSS or native allocations",
        "generated_bytes_before_run_report": sum(p.stat().st_size for p in args.out.rglob("*") if p.is_file()),
        "cases_per_s": len(cases) / elapsed, "production_qualification": "NOT_ESTABLISHED"}
    write_json(args.out / "run.json", report)
    print(json.dumps(report, sort_keys=True))
    return 1 if report["failed_count"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
