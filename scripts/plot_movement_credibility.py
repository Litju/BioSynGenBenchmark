"""Publication-safe synthetic traces; requires optional matplotlib.

Run in the isolated scientific environment. Plots use actual native device
timestamps independently, and label latent states separately from observations.
"""
import argparse
import hashlib
import json
from pathlib import Path


def main(argv=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scripts.run_i2_benchmark import families
    from biosyngen.i2.acquisition import DUAL_1KHZ_V1
    from biosyngen.i2.instruments import default_lpt, observe_dual_plates, observe_lpt
    from biosyngen.i2.drop_jump import build_drop_jump_candidate
    from biosyngen.i2.bench import bench_lpt
    from biosyngen.i2.deadlift import deadlift_lpt
    from biosyngen.i2.squat import squat_lpt
    from biosyngen.i2.olympic import pull_lpt
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    out = Path(parser.parse_args(argv).out)
    if out.exists():
        parser.error("output must be fresh")
    out.mkdir(parents=True)
    manifest = {}
    cases = list(families()) + [("DROP_REBOUND_CANDIDATE", build_drop_jump_candidate())]
    for name, scenario in cases:
        trial = scenario.run()
        cable_builder = {"BENCH_PRESS": bench_lpt, "DEADLIFT": deadlift_lpt, "BACK_SQUAT": squat_lpt,
                         "CLEAN_PULL": pull_lpt, "SNATCH_PULL": pull_lpt,
                         "POWER_CLEAN_PULL_ONLY": pull_lpt}.get(name, default_lpt)
        force_model, lpt_model = DUAL_1KHZ_V1.instruments(cable_builder(trial))
        force, lpt = observe_dual_plates(trial, force_model), observe_lpt(trial, lpt_model)
        times = [f.t_end_s for f in trial.frames]
        mass = trial.model.total_mass_kg()
        com = [trial.model.system.center_of_mass(f.state)[1] for f in trial.frames]
        velocity = [trial.model.system.linear_momentum(f.state)[1]/mass for f in trial.frames]
        fig, axes = plt.subplots(3, 2, figsize=(12, 10), constrained_layout=True)
        ax = axes.flat
        ax[0].plot([s.native_time_s for s in force.samples], [s.fz_left_n for s in force.samples], label="Left plate")
        ax[0].plot([s.native_time_s for s in force.samples], [s.fz_right_n for s in force.samples], linestyle="--", label="Right plate")
        ax[0].set(title="Simulated FORCE: feet-only support", ylabel="Vertical force (N)", xlabel="FORCE native time (s)")
        ax[0].legend()
        ax[1].plot([s.native_time_s for s in lpt.samples], [s.displacement_m for s in lpt.samples])
        ax[1].set(title="Simulated LPT: "+lpt_model.attachment.label, ylabel="Cable displacement (m)", xlabel="LPT native time (s)")
        ax[2].plot(times, com)
        ax[2].set(title="Latent whole-system COM", ylabel="Height (m)", xlabel="Model time (s)")
        ax[3].plot(times, velocity)
        ax[3].axhline(0, color="black", linewidth=.7)
        ax[3].set(title="Latent whole-system COM velocity", ylabel="Vertical velocity (m/s)", xlabel="Model time (s)")
        for side, style in (("left", "-"), ("right", "--")):
            ax[4].plot(times, [trial.model.system.joint_state(f.state, side+"_knee")[0] for f in trial.frames],
                       linestyle=style, label=side+" knee")
            ax[5].step(times, [sum(f.state.contacts[c].mode.value != "OPEN" for c in trial.model.contact_ids[side]) for f in trial.frames],
                       where="post", linestyle=style, label=side)
        ax[4].set(title="Latent knee configuration", ylabel="Angle (rad)", xlabel="Model time (s)")
        ax[5].set(title="Latent foot contacts engaged", ylabel="Contact count", xlabel="Model time (s)", yticks=[0, 1, 2])
        ax[4].legend(); ax[5].legend()
        for a in ax:
            a.grid(alpha=.2)
        fig.suptitle(name+" | SIMULATED_MEASUREMENT_ONLY\nTermination: "+trial.termination, fontsize=12)
        path = out/(name.lower()+".png")
        fig.savefig(path, dpi=140)
        plt.close(fig)
        manifest[name] = dict(figure=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            termination=trial.termination, force_samples=len(force.samples), lpt_samples=len(lpt.samples),
            acquisition="native dual-1kHz independent clocks", visual_inspection="PENDING")
    (out/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
