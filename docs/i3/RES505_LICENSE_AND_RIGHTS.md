# RES-505 proposed license and rights disposition

Status: **BLOCKED_LICENSE_OWNER_DECISION** and **RIGHTS_CONFIRMATION_PENDING**. Open-source intent is approved. No actual code license has been selected or granted. The root README's all-rights-reserved statement still applies. The exact Apache text in `legal/APACHE-2.0.proposed.txt` is an option for review, not a license grant over this project.

## Concrete license choice

| Option | Redistribution and attribution | Patent and compatibility considerations |
| --- | --- | --- |
| Apache-2.0 (recommended) | Permissive; retain notices and license, mark modifications, preserve applicable NOTICE content | Express contributor patent grant with patent-litigation termination; no trademark grant. Apache code can enter GPLv3 works, not GPLv2-only works. |
| MIT | Permissive; retain the copyright and permission notice | Short terms; no express patent grant equivalent to Apache section 3. Allows proprietary derivative distribution. |
| GPL-3.0-only or GPL-3.0-or-later | Copyleft on conveyed covered works; provide corresponding source and preserve notices | Patent provisions and installation-information requirements can apply. The owner must choose "only" or "or later" explicitly. Mere network use is not AGPL network copyleft. |

The recommendation is Apache-2.0 for rights-cleared project source, documentation and manufactured fixtures/figures, with separate exclusions for third-party or restricted material. The rationale is permissive scientific reuse with an explicit patent grant. It does not resolve actual patent ownership or employer/source obligations.

Authoritative texts: [Apache terms](https://www.apache.org/licenses/LICENSE-2.0), [MIT text](https://choosealicense.com/licenses/mit/), [GPLv3 text/identifiers](https://spdx.org/licenses/GPL-3.0-only.html), [Apache/GPL compatibility](https://www.apache.org/licenses/GPL-compatibility.html). The license comparison is a release decision brief; it is not an assertion that this repository already meets those terms.

Downloaded exact proposed Apache text: source `https://www.apache.org/licenses/LICENSE-2.0.txt`, SHA-256 `cfc7749b96f63bd31c3c42b5c471bf756814053e847c10f3eb003417bc523d30`. No copied textbook, article, vendor manual or experimental dataset was downloaded.

## Ownership and attribution evidence

All 29 commits in the frozen I2 ancestry identify Julio Rodriguez and the same GitHub noreply address. This establishes recorded Git attribution, not proof of copyright ownership or employment clearance. Opus 5.5 implementation and GPT-6.1 Solo I3 assistance are workflow provenance; neither is treated as a human scientific author. The owner must confirm the rights holder and authorization to grant the selected license, including any employment/contract obligations.

The RES-503 manifest is the exact per-file/per-blob ledger. Code and scientific prose were inspected for references, source fragments, external imports and declared provenance. Winter/Thelen/Hill references are recalled numerical/form hypotheses labeled ASSUMED_UNCALIBRATED; no underlying book tables or paper pages were copied into this campaign. Those sources remain citations, not material granted a new license. Lack of a detected copy is not an independent chain-of-title certificate.

The RES-490 registry inventory carries external identifiers, test names and descriptions. Its recorded no-source-code-copy assertion is retained. Confirm authorization to publish the existing inventory and registry-derived constants, or produce a replacement containing only independently authored descriptions and necessary factual identities. Until then the inventory and dependent `protocol_inventory.py`/coverage tests are a rights dependency, not a distributable isolated data bundle.

## File and history disposition

| Material | Present status | Proposed release treatment |
| --- | --- | --- |
| Project Python/PowerShell source and tests, CI definitions | Candidate original project work; title not independently certified | Selected code license after owner rights confirmation |
| Project-authored documentation and synthetic JSON/SVG fixtures | Numerical/model artifacts; no admitted real sensor or participant records | Same selected license if owner approves that scope; retain scientific ceilings and provenance |
| Historical source snapshots under runs/i1 | Project software regression snapshots | Retain software history and attribution; do not describe them as external empirical corpora |
| Registry-derived inventory/constants | Rights review required | Explicit owner clearance or independently authored replacement before publication |
| Existing failure/optimization logs and new process receipts | Machine metadata in some paths/commands | Preserve local evidence; public history requires a reviewed metadata disposition or sanitized new review branch |
| `.claude/launch.json` | Local launch configuration, originally untracked | Exclude from publication/staging |
| Ignored research, external corpora, participant records, proprietary exports, locked/consumed holdouts, sibling evidence | Not accessed in I3; not part of intended code release | Exclude; a code license does not authorize access or redistribution |
| Proposed Apache legal text | Official reusable license document, exact source/hash above | Draft only until owner selects terms; no implication of project licensing |

The history audit screened 229 distinct baseline blobs and recorded 20 machine-path flags, with zero matches for its four credential-pattern classes, zero NUL-bearing binaries and zero blobs above 5 MB. Screening and source review remain bounded evidence, not proof against every secret/PII/copy format. Later I3 commits must be rescanned as part of release qualification.

No public branch has been pushed. A clean tip alone cannot clear ancestor bytes. If a sanitized candidate history is needed, preserve the original local branches and issue commits, construct separate issue-scoped review commits from the cleared public base, and record original→public SHA mappings. Do not amend, force-push, erase original negative evidence or silently combine issues. The owner must review the final legal/metadata content before publication under the binding I3 contract.

## Dependency ledger

| Dependency | Actual role/version | License evidence / distribution scope |
| --- | --- | --- |
| CPython | Primary Windows 3.12.10; independent-solver environment 3.11.9 | [PSF license and incorporated notices](https://docs.python.org/3/license.html); interpreter not vendored |
| Python standard library | Generator/evaluator/test runtime; evaluator imports only stdlib | Covered by interpreter's own license; no bundled interpreter distribution proposed |
| SciPy 1.17.1 | Optional independent I1 DOP853 verification; Python >=3.11 | [SciPy license](https://github.com/scipy/scipy/blob/v1.17.1/LICENSE.txt), BSD-3-Clause plus bundled component notices; not vendored |
| NumPy 2.4.4 | Installed transitive SciPy dependency in qualification environment | [NumPy license](https://github.com/numpy/numpy/blob/v2.4.4/LICENSE.txt); metadata expression BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0; not vendored |
| actions/checkout v4 | Existing remote CI action reference | [MIT](https://github.com/actions/checkout/blob/v4/LICENSE); action not copied into repository |
| actions/setup-python v5 | Existing remote CI action reference | [MIT](https://github.com/actions/setup-python/blob/v5/LICENSE); action not copied into repository |
| Windows PowerShell / PowerShell 7 | Local historical-controller tests | Host tooling, not vendored; runtime not included in release |

No third-party runtime Python dependency is needed for the core generator or evaluator. If future wheels, executables or containers bundle these dependencies, collect their exact binary component notices and distributions; this source-only ledger cannot certify a future binary bundle. Future CI should pin the qualification dependencies and action revisions under RES-506.

## Required owner decisions

1. Select one actual license, including whether owned docs and synthetic fixtures/figures use the same terms.
2. Confirm the copyright owner and authority to license the project and publish the registry-derived identity inventory/constants; identify any exclusions or obligations.
3. Review the final legal/metadata changes and intended history disposition before a public push. Merge approval remains a separate later decision.

On confirmation: record the exact owner wording in Linear, promote the selected draft to root LICENSE, add the selected SPDX/copyright metadata and any required notices, update CITATION/README, rescan the full candidate history and continue RES-506 only after scientific qualification. No engineering operations are required from the owner.
