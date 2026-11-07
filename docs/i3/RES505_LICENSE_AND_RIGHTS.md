# RES-505 proposed license and rights disposition

Status: **LICENSE_CHOSEN_BY_OWNER=Apache-2.0**. The owner explicitly selected Apache-2.0 and stated "decide whats best is my personal research" in response to the copyright/authority/inventory question. This is the recorded basis for personal-project ownership and delegated conservative publication dispositions, not a court-issued title opinion. LICENSE and NOTICE now adopt the selected terms for owned work. External source implementations and restricted data remain excluded. The exact decision and dispositions are in `RES505_OWNER_DECISION.json`; the earlier proposed text remains preserved.

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

All 29 commits in the frozen I2 ancestry identify Julio Rodriguez and the same GitHub noreply address. This establishes recorded Git attribution, not proof of copyright ownership or employment clearance. Opus 5.5 implementation and GPT-6.1 Solo I3 assistance are workflow provenance; neither is treated as a human scientific author. The owner's personal-research reply supplies the ownership/authority declaration used for release preparation; no employment restriction was stated. Git attribution is consistent with the declaration. Future contributors must separately establish submission rights.

The RES-503 manifest is the exact per-file/per-blob ledger. Code and scientific prose were inspected for references, source fragments, external imports and declared provenance. Winter/Thelen/Hill references are recalled numerical/form hypotheses labeled ASSUMED_UNCALIBRATED; no underlying book tables or paper pages were copied into this campaign. Those sources remain citations, not material granted a new license. Lack of a detected copy is not an independent chain-of-title certificate.

The RES-490 registry inventory carries external identifiers, test names and descriptions. Its recorded no-source-code-copy assertion is retained. Under the owner's delegated disposition, retain the independently authored research summary and necessary factual identifiers/constants, with explicit attribution and a boundary excluding external registry source implementation. No Plate2bar source license is granted or changed. The source-access record and inspected tree contain no copied registry implementation; future additions cannot inherit this clearance automatically.

## File and history disposition

| Material | Present status | Proposed release treatment |
| --- | --- | --- |
| Project Python/PowerShell source and tests, CI definitions | Owned personal research per direct owner declaration | Apache-2.0 adopted; no independent legal title certificate claimed |
| Project-authored documentation and synthetic JSON/SVG fixtures | Numerical/model artifacts; no admitted real sensor or participant records | Apache-2.0 adopted for owned material; retain scientific ceilings and provenance |
| Historical source snapshots under runs/i1 | Project software regression snapshots | Retain software history and attribution; do not describe them as external empirical corpora |
| Registry-derived inventory/constants | Owner-delegated factual-summary disposition | Retain owned research summary/identities, exclude external registry implementation |
| Existing failure/optimization logs and new process receipts | Machine metadata in some paths/commands | Preserve local evidence; canonical/generic nonsecret paths reviewed and retained; private data remains excluded |
| `.claude/launch.json` | Local launch configuration, originally untracked | Exclude from publication/staging |
| Ignored research, external corpora, participant records, proprietary exports, locked/consumed holdouts, sibling evidence | Not accessed in I3; not part of intended code release | Exclude; a code license does not authorize access or redistribution |
| Proposed Apache legal text | Official reusable license document, exact source/hash above | Historical proposed copy preserved; identical root LICENSE now grants the owner-selected terms |

The history audit screened 229 distinct baseline blobs and recorded 20 machine-path flags, with zero matches for its four credential-pattern classes, zero NUL-bearing binaries and zero blobs above 5 MB. Screening and source review remain bounded evidence, not proof against every secret/PII/copy format. Later I3 commits must be rescanned as part of release qualification.

No public branch has been pushed. A clean tip alone cannot clear ancestor bytes. If a sanitized candidate history is needed, preserve the original local branches and issue commits, construct separate issue-scoped review commits from the cleared public base, and record original→public SHA mappings. Do not amend, force-push, erase original negative evidence or silently combine issues. The owner reviewed the concrete license/rights proposal and selected its Apache option, delegating the personal-research dispositions. Actual publication still requires the complete candidate tree/history scan and scientific qualification; main merge remains owner-only.

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

## Recorded decision and remaining publication work

The direct owner license selection and personal-research authority reply are recorded verbatim in Linear and `RES505_OWNER_DECISION.json`. Root LICENSE is byte-identical to the reviewed Apache option. NOTICE identifies the copyright holder and scope; CITATION includes the SPDX license identifier. Code/docs/manufactured artifacts and retained owned historical versions share the selected terms. External data and source implementations do not.

Canonical project paths and generic operator interpreter paths were inspected and classified as nonsecret reproducibility metadata. Baseline path-bearing records contain only canonical project/test/optimization locations; new worker receipts add generic local interpreter locations. Preserve them as evidence rather than rewriting historical failure logs. The originally untracked `.claude/` is now ignored and remains local. No participant identity, credential, proprietary source or third-party book/article payload was found in the intended tree/history. Pattern screening remains bounded assurance, not a guarantee against every unknown secret format.

Before public push, RES-506 must rescan the final revision, apply the explicit exclusions and inspect all new issue files. A reviewed source branch/PR is authorized by the I3 mission after these gates; merging main remains a separate owner decision. No engineering operations are required from the owner.
