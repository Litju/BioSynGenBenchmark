"""Render the manufactured contact fixture against the distinct RK4 reference."""
import argparse
import hashlib
import json
from pathlib import Path


def main(argv=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from tests.test_res597_compliant_contact import drop
    from tests.test_res598_contact_credibility import independent_reference
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    out = Path(parser.parse_args(argv).out)
    if out.exists():
        parser.error("output must be fresh")
    out.mkdir(parents=True)
    ref_z, ref_v = independent_reference()
    rows = []
    for h in (.0004, .0002, .0001):
        v, impulse, energy, dissipation, peak = drop(h, .3)
        rows.append(dict(dt_s=h, velocity_m_s=v, impulse_n_s=impulse,
                         energy_j=energy, dissipation_j=dissipation, peak_force_n=peak,
                         velocity_error_m_s=abs(v-ref_v),
                         energy_closure_error_j=abs(energy+dissipation-.981)))
    assert rows[2]["energy_closure_error_j"] < rows[1]["energy_closure_error_j"] < rows[0]["energy_closure_error_j"]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), constrained_layout=True)
    fields = [("energy_closure_error_j", "Energy closure error (J)", True),
              ("velocity_error_m_s", "Velocity error versus RK4 (m/s)", True),
              ("peak_force_n", "Finite contact peak (N)", False)]
    for ax, (field, label, log_y) in zip(axes, fields):
        ax.plot([r["dt_s"]*1000 for r in rows], [r[field] for r in rows], marker="o")
        ax.set(xlabel="Timestep (ms)", ylabel=label, xscale="log")
        if log_y:
            ax.set_yscale("log")
        ax.grid(alpha=.25)
    fig.suptitle("Manufactured 1 kg drop: numerical verification only; no human/device validation")
    path = out/"contact_convergence.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    (out/"contact_convergence.json").write_text(json.dumps(dict(
        context="1 kg, 0.1 m release, 9.81 m/s², k=10000 N/m, damping=0.3 s/m, 0.3 s duration",
        reference_method="Distinct scalar RK4 at 10 microseconds", reference_z_m=ref_z,
        reference_v_m_s=ref_v, rows=rows, figure_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        scope="Numerical contact fixture only; not full articulated energy qualification"), indent=2)+"\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
