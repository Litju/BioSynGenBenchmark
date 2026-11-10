"""RES-624 RED canary: the frozen RES-618 falsifier must still reproduce its failure.

    python -m scripts.res624_red_canary [--out runs/i3/res624/ci/red_canary.json]

Calls the unmodified ``scripts.res618_fullbody_qualification.run()`` (which
always reports FAIL) and checks that the *physics* of the RED is still
present: the bar loses all hand support and drops. Exit 0 iff the RED is
reproduced; exit 1 if the frozen witness no longer fails physically (which
would mean the regression baseline changed). Never writes runs/i3/res618/*.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from scripts.res618_fullbody_qualification import run as frozen_run

# Historical drops: Windows 0.3760531196714312 m, Linux 0.376222 m. The canary
# asserts the qualitative failure (no hand contact, drop > 0.3 m), not a value.
MIN_DROP_M = .3


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/i3/res624/ci/red_canary.json")
    a = ap.parse_args(argv)
    r = frozen_run()
    hand = [c for c in r["final_contacts"] if "bar_shaft" in (c["geom1"], c["geom2"])]
    reproduced = r["outcome"] == "FAIL" and r["bar_drop_m"] > MIN_DROP_M and not hand
    out = dict(issue="RES-624", canary="RES618_FROZEN_RED", reproduced=reproduced, frozen_outcome=r["outcome"],
               frozen_code=r["code"], bar_drop_m=r["bar_drop_m"], final_hand_contacts=len(hand),
               final_contacts=[(c["geom1"], c["geom2"]) for c in r["final_contacts"]], source_sha=r["source_sha"])
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(out, indent=2, sort_keys=True)+"\n", encoding="utf-8")
    print(json.dumps(out, indent=2))
    return 0 if reproduced else 1


if __name__ == "__main__":
    raise SystemExit(main())
