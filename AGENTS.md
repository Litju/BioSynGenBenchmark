# BioSynGenBenchmark agent rules

This workspace is the canonical project root: D:\projects\BioSynGenBenchmark. Read the current I0 contract and the live Linear issue before each queue item. The only owner-authorized implementation queue is RES-475 → RES-476 → RES-477 → RES-478 → RES-479, in that order.

For each issue, implement the specified software, run its fresh tests and falsifying controls, inspect the outputs, and record exact commands and evidence. Move a passing issue to In Review; never move it to Done. Continue to the next I0 issue only after the preceding evidence is reviewable. Stop at a blocker, after two repair attempts, or at six elapsed hours.

Keep code, tests, synthetic-only outputs, and run records inside this project. Preserve existing files after inspection. Do not access or modify Plate2bar, DynamisBench, historical corpora, participant data, locked or consumed holdouts, the sibling evidence folder, or unrelated research issues. Do not launch a nested Codex process from an active Codex agent.

Use only the claim ceilings NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, and REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. Synthetic results do not establish human or device validity. Keep generator truth separate from the independent evaluator.

For a separately started local runner, read README.md and scripts/overnight.ps1 first. The runner requires a fresh issue snapshot from an authenticated Linear connection and, for live execution, authenticated Linear access in the Codex CLI itself. Use Windows PowerShell; no WSL setup is required.
