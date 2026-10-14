# Contributing to BioSynGenBenchmark

Owned project contributions are governed by Apache-2.0; preserve the LICENSE and applicable NOTICE/attribution. Contribution rights must cover all submitted material. Governance and the owner decision are recorded in `docs/i3/RES505_LICENSE_AND_RIGHTS.md`. Public review and scientific qualification remain separate gates.

This is a numerical research prototype. Contributions must distinguish numerical verification, modeled biomechanics, simulated measurements and empirical validation. Keep the four scientific ceilings in source and artifacts: NUMERICAL_VERIFICATION_ONLY; SIMULATED_MEASUREMENT_ONLY; BIOLOGICAL_VALIDATION=NOT_ESTABLISHED; REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED.

Submit one logical change with a clear problem, independent falsifier, true test exit codes, expected/observed results and limitations. Keep generator latent truth separate from evaluator inputs and oracle construction. Preserve existing unsupported-mechanics refusals; new implementations require explicit numerical and scientific acceptance criteria. Do not relax thresholds or hide failed runs to obtain a PASS.

Use manufactured data and independently authored analytical fixtures. Include provenance and licenses for new dependencies or assets. Participant data, proprietary sensor exports, external corpora, locked/consumed holdouts, paid source scans and undisclosed copied code are outside the software contribution scope. Gitignore does not remove private bytes from existing history.

Run relevant tests plus the complete suite before proposing a review commit:

    python -m unittest discover -s tests -v
    python -m scripts.verify_i3_reproducibility
    git diff --check

Optional solver qualification requires SciPy in a separate Python 3.11 environment; record exact Python/SciPy/NumPy versions and run `python -m unittest tests.test_i1_solver_crosscheck -v`. These checks share the I1 derivative and are not an independent biomechanical oracle.

For Windows, capture `$LASTEXITCODE` immediately after each native command and stop on nonzero; do not determine success from a formatting/output pipe. Stage explicit paths, inspect the complete staged diff and inventory, and keep issue-scoped atomic commits. Scientific issue acceptance and main merges remain owner actions. Preserve original authorship and failure/correction history.
