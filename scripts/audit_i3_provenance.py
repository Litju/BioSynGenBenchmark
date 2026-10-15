"""Inventory a frozen Git revision without traversing ignored/private workspaces.

Pattern screening is triage, not proof of copyright or secret clearance.
No finding includes matched content; restricted paths are listed but never read.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess

BASELINE = "3322d89499050eea0004f9e8882666e2a40d5a9a"
SECRET_PATTERNS = {
    "private_key": re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    "github_token": re.compile(rb"\b(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,})\b"),
    "provider_key": re.compile(rb"\bsk-(?:proj-)?[A-Za-z0-9_-]{32,}\b"),
    "aws_key": re.compile(rb"\bAKIA[A-Z0-9]{16}\b"),
}
RESTRICTED_PARTS = {"research", "corpus", "corpora", "holdout", "holdouts", "participants", ".env"}


def restricted(path: str) -> bool:
    parts = set(PurePosixPath(path.lower()).parts)
    return bool(parts & RESTRICTED_PARTS) or PurePosixPath(path).name.lower().startswith(".env.")


def screen(data: bytes) -> list[str]:
    findings = [key for key, regex in SECRET_PATTERNS.items() if regex.search(data)]
    if re.search(rb"[A-Za-z]:[\\/](?:projects|Users)[\\/]", data):
        findings.append("machine_path")
    if b"\x00" in data:
        findings.append("binary_manual_review")
    if len(data) > 5_000_000:
        findings.append("large_artifact")
    return findings


def git(*args: str) -> bytes:
    return subprocess.check_output(["git", *args])


def tree(ref: str) -> list[tuple[str, str]]:
    entries = []
    for record in git("ls-tree", "-rz", "--full-tree", ref).split(b"\0"):
        if not record:
            continue
        metadata, path = record.split(b"\t", 1)
        mode, kind, oid = metadata.decode().split()
        if kind != "blob" or mode not in {"100644", "100755"}:
            raise ValueError("nonregular tree entry requires manual clearance")
        entries.append((path.decode("utf-8"), oid))
    return entries


def audit(ref: str = BASELINE) -> dict:
    resolved = git("rev-parse", ref).decode().strip()
    commits = git("rev-list", "--reverse", resolved).decode().splitlines()
    history_paths: dict[str, set[str]] = {}
    ledger = []
    for sha in commits:
        for path, oid in tree(sha):
            history_paths.setdefault(oid, set()).add(path)
        subject = git("show", "-s", "--format=%s", sha).decode().strip()
        ledger.append({"sha": sha, "subject": subject,
                       "paths": git("diff-tree", "--root", "--no-commit-id", "--name-only", "-r", sha).decode().splitlines()})
    blobs = {}
    for oid, paths in sorted(history_paths.items()):
        size = int(git("cat-file", "-s", oid))
        blocked = any(restricted(p) for p in paths)
        data = None if blocked else git("cat-file", "blob", oid)
        findings = ["restricted_path_not_read"] if blocked else screen(data)
        blobs[oid] = {"git_blob": oid, "paths": sorted(paths), "size_bytes": size,
                      "sha256": None if data is None else hashlib.sha256(data).hexdigest(),
                      "findings": findings}
    files = []
    for path, oid in tree(resolved):
        item = dict(blobs[oid], path=path)
        item.pop("paths")
        item["license"] = "NO_LICENSE_OWNER_CHOICE_PENDING"
        item["provenance"] = ("MANUFACTURED_RUN_ARTIFACT" if path.startswith("runs/") else
                              "PROJECT_AUTHORED_OR_DECLARED_REFERENCE_UNVERIFIED")
        item["disposition"] = ("EXCLUDE" if restricted(path) else
                               "RIGHTS_REVIEW_REQUIRED" if "RES490" in path else
                               "CANDIDATE_PENDING_OWNER_LICENSE_AND_CONTENT_REVIEW")
        files.append(item)
    fingerprint = hashlib.sha256("".join(f"{p}\0{o}\n" for p, o in sorted(tree(resolved))).encode()).hexdigest()
    return {"issue": "RES-503", "revision": resolved, "tree": git("rev-parse", resolved + "^{tree}").decode().strip(),
            "path_blob_inventory_sha256": fingerprint, "files": files, "commit_ledger": ledger,
            "history_blobs": list(blobs.values()),
            "limitations": ["Pattern screening cannot establish ownership, absence of PII or copied code.",
                            "Ignored local files and excluded research/corpora/holdouts are not read.",
                            "Historical source snapshots are software regression fixtures, not external corpora.",
                            "Publication remains gated on owner license and rights/content clearance."]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", default=BASELINE)
    parser.add_argument("--out", default="runs/i3/res503/provenance.json")
    args = parser.parse_args()
    result = audit(args.ref)
    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"revision": result["revision"], "files": len(result["files"]),
                      "commits": len(result["commit_ledger"]), "history_blobs": len(result["history_blobs"]),
                      "finding_counts": {key: sum(key in b["findings"] for b in result["history_blobs"])
                                         for key in sorted({f for b in result["history_blobs"] for f in b["findings"]})}}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
