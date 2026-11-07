"""Render retained RES-507 raw CSVs; no smoothing, capping or journal claim.

Requires optional Matplotlib. Raw interval forces, velocities and instantaneous
impulses occupy separate panels with their own units and full data ranges.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",default="runs/i3/res507/artifacts")
    parser.add_argument("--out",default="runs/i3/res507/rendered")
    args=parser.parse_args(argv)
    project=Path(__file__).resolve().parents[1]
    source=(project/args.source).resolve()
    out=(project/args.out).resolve()
    if not all(p.is_relative_to(project) and p!=project for p in (source,out)):
        parser.error("source/output must remain in project subdirectories")
    if out.exists() and any(out.iterdir()):
        parser.error("use a fresh output directory")
    out.mkdir(parents=True,exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names=("LOADED_JUMP_SQUAT","BACK_SQUAT","DEADLIFT","BENCH_PRESS","CLEAN_PULL","SNATCH_PULL")
    manifest={"issue":"RES-507","Matplotlib":matplotlib.__version__,"transforms":"none; full raw data ranges",
              "scope":"numerical interval forces and impulses, not physical tissue peak/RFD or empirical physiology",
              "sources":{},"outputs":{}}
    with plt.rc_context({"svg.fonttype":"none","font.size":9,"axes.grid":True,"grid.alpha":0.25}):
        for name in names:
            path=source/(name+".csv")
            manifest["sources"][path.relative_to(project).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
            rows=list(csv.DictReader(path.open(encoding="utf-8")))
            def col(key): return [float(r[key]) if r[key] else float("nan") for r in rows]
            times=col("t_end_s")
            fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True,layout="constrained")
            fig.get_layout_engine().set(rect=(0,0.06,1,0.92))
            fig.suptitle(name+" — raw numerical research fixture")
            for key,label,style in (("foot_interval_force_z_n","feet","-"),("bench_force_z_n","bench","--"),
                                    ("rack_force_z_n","rack",":"),("bar_floor_force_z_n","bar floor","-.")):
                values=col(key)
                if any(v!=0 for v in values): axes[0].plot(times,values,style,label=label,linewidth=0.9)
            axes[0].set_ylabel("interval-average force (N)")
            axes[0].legend(loc="upper left",bbox_to_anchor=(1.01,1),frameon=False)
            axes[1].plot(times,col("body_com_vz_m_s"),"-",label="body COM")
            axes[1].plot(times,col("system_com_vz_m_s"),"--",label="system COM")
            if any(r['bar_vz_m_s'] for r in rows): axes[1].plot(times,col("bar_vz_m_s"),":",label="bar")
            axes[1].set_ylabel("modeled vertical velocity (m/s)")
            axes[1].legend(loc="upper left",bbox_to_anchor=(1.01,1),frameon=False)
            impulses=source/(name+"-impulses.csv")
            manifest["sources"][impulses.relative_to(project).as_posix()]=hashlib.sha256(impulses.read_bytes()).hexdigest()
            hits=list(csv.DictReader(impulses.open(encoding="utf-8")))
            groups={r['contact_id'] for r in hits}
            for group in sorted(groups):
                selected=[r for r in hits if r['contact_id']==group]
                axes[2].scatter([float(r['time_s']) for r in selected],[float(r['impulse_z_n_s']) for r in selected],
                                s=20,label=group,marker="x")
            axes[2].set(xlabel="model time (s)",ylabel="instantaneous impulse z (N s)")
            if groups: axes[2].legend(loc="upper left",bbox_to_anchor=(1.01,1),frameon=False)
            fig.text(0.01,0.003,"Synthetic only. Rigid impacts and assumed control/measurement stages: physical peak/RFD NOT_EVALUABLE.",fontsize=8)
            for suffix in ("png","svg"):
                destination=out/(name+"."+suffix)
                fig.savefig(destination,dpi=150,facecolor="white",metadata={"Date":None} if suffix=="svg" else None)
                if suffix=="svg":
                    content=destination.read_text(encoding="utf-8")
                    destination.write_text("\n".join(line.rstrip() for line in content.splitlines())+"\n",
                                           encoding="utf-8",newline="\n")
                manifest['outputs'][destination.relative_to(project).as_posix()]=hashlib.sha256(destination.read_bytes()).hexdigest()
            plt.close(fig)
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n",encoding="utf-8",newline="\n")
    print("Rendered",len(names),"full-range raw figures")
    return 0


if __name__=="__main__":
    raise SystemExit(main())
