"""Deterministic sequential manufactured corpus. SOURCE/TRUTH remain separate."""
import argparse
import ctypes
import json
from pathlib import Path
import sys
import time

from biosyngen.i2.corpus_config import SCHEMA, PROTOCOLS, REFUSED, validate_parameters
from biosyngen.i2.paired_export import export_case, sha, write_json


def peak_rss_bytes():
    if sys.platform == "win32":
        from ctypes import wintypes
        class Counters(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD)] + [
                (name, ctypes.c_size_t) for name in ("PeakWorkingSetSize", "WorkingSetSize",
                "QuotaPeakPagedPoolUsage", "QuotaPagedPoolUsage", "QuotaPeakNonPagedPoolUsage",
                "QuotaNonPagedPoolUsage", "PagefileUsage", "PeakPagefileUsage")]
        counters = Counters()
        counters.cb = ctypes.sizeof(counters)
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.GetCurrentProcess.restype = wintypes.HANDLE
        psapi = ctypes.WinDLL("psapi", use_last_error=True)
        psapi.GetProcessMemoryInfo.argtypes = (wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD)
        if not psapi.GetProcessMemoryInfo(kernel.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
            raise ctypes.WinError(ctypes.get_last_error())
        return counters.PeakWorkingSetSize
    import resource
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * (1 if sys.platform == "darwin" else 1024)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--seed", type=int, default=524)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--protocol", choices=(*PROTOCOLS, *REFUSED, "all"), required=True)
    parser.add_argument("--max-seconds", type=float, default=1800)
    parser.add_argument("--max-bytes", type=int, default=512 * 1024 * 1024)
    args = parser.parse_args(argv)
    config = {} if args.config is None else json.loads(args.config.read_text(encoding="utf-8"))
    if config and (config.get("schema_id") != SCHEMA or set(config) - {"schema_id", "parameters", "cases"}):
        parser.error("unsupported configuration schema/fields")
    parameters, cases = config.get("parameters", {}), config.get("cases", [{}])
    if not isinstance(parameters, dict) or not isinstance(cases, list) or not cases:
        parser.error("parameters must be an object and cases a nonempty list")
    if any(not isinstance(c, dict) or set(c) - {"protocol", "parameters"} or
           not isinstance(c.get("parameters", {}), dict) for c in cases):
        parser.error("case fields must be protocol and parameters")
    if not 1 <= args.count <= 10000 or not 0 <= args.seed < 2**32:
        parser.error("count domain 1..10000; seed domain 0..2**32-1")
    if not 0 < args.max_seconds <= 21600 or not 0 < args.max_bytes <= 10 * 1024**3:
        parser.error("resource limits: 0 < seconds <= 21600; 0 < bytes <= 10 GiB")
    args.out.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    results, consumed_bytes = [], 0
    for i in range(args.count):
        entry = cases[i % len(cases)]
        protocol = entry.get("protocol", PROTOCOLS[i % len(PROTOCOLS)] if args.protocol == "all" else args.protocol)
        p = {**parameters, **entry.get("parameters", {})}
        case_id = f"trial-{args.seed:08d}-{i:04d}"
        before = time.perf_counter()
        result = {"case_id": case_id, "protocol": protocol, "parameters": p}
        try:
            validate_parameters(p, protocol)
            if before - started >= args.max_seconds or (i and consumed_bytes + max(
                    c.get("case_bytes", 0) for c in results) > args.max_bytes):
                raise ValueError("RESOURCE_BUDGET_EXHAUSTED")
            result.update(export_case(args.out, protocol=protocol, seed=args.seed + i,
                                      case_id=case_id, parameters=p))
            result["status"] = "GENERATED_NOT_EMPIRICALLY_QUALIFIED"
        except (ValueError, ArithmeticError) as exc:
            result.update(status="REFUSED", reason=f"{type(exc).__name__}:{exc}")
        except OSError as exc:
            # Partial bytes are retained as failure evidence. The receipt marks
            # the case refused, so no verifier can qualify the partial bundle.
            result.update(status="REFUSED", reason=f"OS_WRITE_FAILURE:{type(exc).__name__}:{exc}",
                partial_artifacts=sorted(path.relative_to(args.out).as_posix() for namespace in
                    ("SOURCE", "TRUTH", "DIAGNOSTICS") for path in (args.out / namespace / case_id).rglob("*")
                    if path.is_file()))
        result["elapsed_s"] = time.perf_counter() - before
        result["case_bytes"] = sum(path.stat().st_size for namespace in ("SOURCE", "TRUTH", "DIAGNOSTICS")
            for path in (args.out / namespace / case_id).rglob("*") if path.is_file())
        consumed_bytes += result["case_bytes"]
        results.append(result)
    elapsed = time.perf_counter() - started
    generated = sum(c["status"] == "GENERATED_NOT_EMPIRICALLY_QUALIFIED" for c in results)
    report = {"schema_id": "BSMB-CORPUS-RUN-1.0", "config": config, "seed": args.seed,
        "requested_count": args.count, "generated_count": generated, "refused_count": args.count - generated,
        "cases": results, "elapsed_s": elapsed, "peak_rss_bytes": peak_rss_bytes(),
        "resource_scope": "sequential process lifetime peak resident memory; no retained trajectory cache",
        "corpus_bytes": consumed_bytes, "generated_cases_per_s": generated / elapsed,
        "forecast_1000_cases_s": 1000 * elapsed / generated if generated else None,
        "forecast_1000_cases_bytes": 1000 * consumed_bytes / generated if generated else None,
        "forecast_scope": "linear exploratory extrapolation; not a population estimate or bound",
        "limits": {"seconds": args.max_seconds, "bytes": args.max_bytes},
        "empirical_validity": "NOT_ESTABLISHED", "plate2bar_integration": "DEFERRED"}
    inventory = {p.relative_to(args.out).as_posix(): sha(p) for p in sorted(args.out.rglob("*")) if p.is_file()}
    write_json(args.out / "inventory.json", inventory)
    write_json(args.out / "run.json", report)
    print(json.dumps(report, sort_keys=True))
    return 1 if report["refused_count"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
