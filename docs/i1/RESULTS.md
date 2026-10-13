# I1 results: commands, versions, counts, digests

Environment: Windows 11 Home 10.0.22631. CPython 3.12.10, standard library only (canonical). CPython 3.11.9 with SciPy 1.17.1, used only for the optional solver cross-check. No packages were installed. PowerShell 7 is absent.

| Command | Exit | Result |
|---|---|---|
| `python -m unittest discover -s tests -v` | 0 | 96 tests: 94 OK, 2 skipped (SciPy unavailable on 3.12) → `runs/i1/res487/final-test-suite.txt` |
| `py -3.11 -m unittest tests.test_i1_solver_crosscheck -v` | 0 | 2 OK → `runs/i1/res487/scipy-crosscheck-py311.txt` |
| `python -m scripts.run_i1 --replicate` | 0 | Two separate OS processes, all 6 artifacts byte-identical. Bundle digest `ddbd7282e3b1fbf1675921ecc3507b3f2c9c4a2ef9e089cc63333c7aebdec53d` → `runs/i1/res487/reproducibility.json`; `runs/i1/res487/run/manifest.json` lists source and artifact SHA-256 |
| `python -m scripts.i1_benchmark_suite` | 0 | gate PASS, 81/81 adjudicated instances, 0 false accepts; refuse-all and accept-all FAIL; canary refused → `runs/i1/res485/benchmark-suite.json` |
| `python -m scripts.i1_provenance_sensitivity` | 0 | ledger, 19 sensitivity runs, 10-seed sensor spread, counterexamples → `runs/i1/res486/` |
| `python -m scripts.i1_i0_regression` | 0 | I0 artifacts byte-identical except manifest source hashes and one reason string → `runs/i1/res481/i0-regression.json` |
| `python -m scripts.i1_optimize_control --task {cmj,sj,loaded_cmj}` | 0 | declared control searches → `runs/i1/res482/` |
| `pwsh -File tests\test_overnight_controller.ps1` | not run | PowerShell 7 not installed |

Source hashes at completion: `runs/i1/res487/source-sha256.txt` (42 files).

## Reference scenario outcomes (simulated; hypothetical adult)

| Scenario | Takeoff v (m/s) | Apex rise (m) | Depth (m) | Phases (s) | Flight end |
|---|---|---|---|---|---|
| CMJ | 2.395 | 0.292 | 0.305 | quiet 0–0.301, unweighting –0.642, braking –0.796, propulsion –1.045, flight –1.533 | foot contact (landing not modeled) |
| SJ | 2.128 | 0.231 | 0 (no countermovement) | static hold 0–0.329, propulsion –0.530, flight –0.965 | foot contact |
| Loaded CMJ, 20 kg | 1.725 | 0.152 | 0.246 | quiet 0–0.301, unweighting –0.599, braking –0.741, propulsion –1.002, flight –1.326 | foot contact |

Figures (inspected rendered in a browser):
* `runs/i1/res487/run/figure-cmj.svg`: postures, GRF, joint angles, activation, COM velocity, LPT, COP.
* `figure-loaded-cmj.svg`
* `figure-vv.svg`: residuals, evaluator outcomes, sensitivity.
