"""Plot measured scientific residuals from a fresh credibility report, without rerunning physics."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def main(argv=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    report_path, output = Path(args.report), Path(args.out)
    if output.exists():
        parser.error("output must be fresh")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["denominators"]["total"] == 120
    cases = sorted(report["bounded_drop_cases"], key=lambda c: c["conditions"]["dt_s"])
    steps = [c["conditions"]["dt_s"]*1000 for c in cases]
    errors = [abs(c["energy"]["raw_energy_residual_j"]) for c in cases]
    assert len(steps) >= 3 and all(math.isfinite(x) and x > 0 for x in steps+errors)
    families = report["baseline"]["families"]
    names = list(families)
    ratios = [families[name]["angular_momentum"]["max_prefix_residual_kg_m2_s"]/
              families[name]["angular_momentum"]["tolerance_kg_m2_s"] for name in names]
    assert all(math.isfinite(x) and x >= 0 for x in ratios)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5), constrained_layout=True)
    axes[0].loglog(steps, errors, "o-", label="Measured energy residual")
    axes[0].loglog(steps, [errors[0]*h/steps[0] for h in steps], "--", color="gray",
                   label="First-order guide, anchored at finest step")
    axes[0].set(xlabel="Integration step (ms)", ylabel="Absolute residual (J)",
                title="First rebound only: convergence is not energy admission")
    axes[0].legend(fontsize=8)
    colors = ["#b54738" if ratio > 1 else "#36786d" for ratio in ratios]
    axes[1].bar(names, ratios, color=colors)
    axes[1].set_yscale("symlog", linthresh=1)
    axes[1].axhline(1, color="black", linestyle="--", label="Declared numerical budget")
    axes[1].set(ylabel="Maximum angular residual / budget", title="Independent planar angular balance")
    axes[1].tick_params(axis="x", labelrotation=65, labelsize=8)
    axes[1].legend(fontsize=8, loc="upper left")
    for axis in axes:
        axis.grid(alpha=.2, axis="y")
    fig.suptitle("NUMERICAL_VERIFICATION_ONLY | source "+report["source_sha"][:12]+
                 "\nFull movement and biological/device qualification remain incomplete", fontsize=11)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)
    metadata = dict(source_sha=report["source_sha"], matplotlib=matplotlib.__version__,
                    report_sha256=hashlib.sha256(report_path.read_bytes()).hexdigest(),
                    figure_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                    visual_inspection="PENDING", scope="Synthetic residual diagnostics only")
    output.with_suffix(".json").write_text(json.dumps(metadata, indent=2)+"\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
