"""Reproduce failed landing controls; no completed-drop-jump admission."""
import json
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from biosyngen.i2.drop_jump import build_drop_jump_candidate
from biosyngen.i2.jumps import _gains


def main():
    results = []
    for name in ("baseline", "ankle_balance", "ankle_and_hip_balance"):
        candidate = build_drop_jump_candidate(duration_s=3)
        changes = {}
        if name != "baseline":
            changes = dict(balance_gain_n_m_per_m=3000,
                           balance_damping_n_m_s_per_m=1200)
        if name == "ankle_and_hip_balance":
            changes.update(kp_n_m_per_rad=_gains(150, 1500, 1500),
                           kd_n_m_s_per_rad=_gains(30, 100, 100),
                           hip_balance_gain_n_m_per_m=1000,
                           hip_balance_damping_n_m_s_per_m=300)
        candidate.controller.stages = [
            replace(stage, **changes) if stage.name == "FINAL_LANDING" else stage
            for stage in candidate.controller.stages]
        trial = candidate.run(.0004)
        final = trial.frames[-1]
        assert trial.termination.startswith("BODY_GROUND_CONTACT_NOT_MODELED")
        assert final.t_end_s < 3
        results.append(dict(control=name, changes=changes, dt_s=.0004,
                            requested_duration_s=3, terminal_time_s=final.t_end_s,
                            termination=trial.termination,
                            terminal_momentum_kg_m_s=candidate.model.system.linear_momentum(final.state),
                            commands=trial.control_commands))
    print(json.dumps(dict(claim="NUMERICAL_VERIFICATION_ONLY", completed=0,
                          attempted=3, trials=results), indent=2))


if __name__ == "__main__":
    main()
