# BioSynGenBenchmark agent rules

This workspace is the canonical project root: D:\projects\BioSynGenBenchmark. Read the active project contract and the live Linear issue before each queue item. The current owner-authorized follow-on queue under RES-488 is RES-489 → RES-490 → RES-491 → RES-492, in that order. The earlier I0 and I1 queues are historical work; preserve their source and run artifacts.

For each issue, implement the specified software, run its fresh tests and falsifying controls, inspect the outputs, and record exact commands and evidence. Move a passing issue to In Review; never move it to Done. Continue to the next authorized issue only after the preceding evidence is reviewable. Stop at a blocker, after two repair attempts, or at six elapsed hours.

Before moving any issue to In Review, run its relevant tests and falsifiers, inspect `git diff --check` and the complete staged diff, stage only that issue's files, and verify the staged inventory contains no other issue's edits or private/source/holdout data. Create an issue-scoped atomic commit named `RES-<id>: <specific verified outcome>`, record its full SHA, staged file inventory, commands/counts/results, evidence limitations, and branch/PR reference in Linear, and leave no uncommitted issue work before continuing. Never amend/force-push or merge to `main`; keep owner-only Done transitions for the owner. If a public branch or draft PR is not safe, keep the branch local and record why.

Keep code, tests, synthetic-only outputs, and run records inside this project. Preserve existing files after inspection. RES-490 authorizes read-only inspection of Plate2bar's actual exercise/test registry for inventory only; do not modify Plate2bar or use its output as a force/phase oracle. Do not access DynamisBench, historical corpora, participant data, locked or consumed holdouts, the sibling evidence folder, or unrelated research issues. Do not launch a nested Codex process from an active Codex agent.

Use only the claim ceilings NUMERICAL_VERIFICATION_ONLY, SIMULATED_MEASUREMENT_ONLY, BIOLOGICAL_VALIDATION=NOT_ESTABLISHED, and REAL_PAIRED_FORCE_LPT_VALIDATION=NOT_ESTABLISHED. Synthetic results do not establish human or device validity. Keep generator truth separate from the independent evaluator.

For a separately started local runner, read README.md and scripts/overnight.ps1 first. The runner requires a fresh issue snapshot from an authenticated Linear connection and, for live execution, authenticated Linear access in the Codex CLI itself. Use Windows PowerShell; no WSL setup is required.
