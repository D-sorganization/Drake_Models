# Development Log — Drake_Models

State table for every feature in flight in this repository. Update
entries **in place**; never append dated sections. One entry per
feature, from proposal to ship. See the `development-logs` section of
`AGENTS.md` for the binding rules and
`shared_scripts/development_log.py` for the validator.

- **Portfolio:** work
- **WIP limit:** 5
- **Last audited:** 2026-08-28 by bootstrap

## States

`proposed` → `in_progress` → `in_review` → `shipped`, with `parked`
reachable from any live state and `abandoned` from `parked`.
`shipped` never returns to `in_progress`; open a new entry instead.

## Active

### DL-#384 · SECURITY: Guard Fork PRs Off the Self-Hosted Fleet (RM#1989); Vendor Fork_Pr_Runner_Guard and Wire It Into CI

- **State:** in_review
- **Owner:** unassigned
- **Issue:** #384
- **Branch:** fix/fork-pr-runner-guard
- **PR:** #385
- **Paths:** see #385
- **Started:** 2026-10-07
- **Last verified:** 2026-10-07 (`d57f0159`; collated from changes/384-security-guard-fork-prs-off-the-self-hos.md)
- **Summary:** SECURITY: guard fork PRs off the self-hosted fleet (RM#1989); vendor fork_pr_runner_guard and wire it into CI
- **Next step:** Merge the PR.

### DL-#373 · Canonical Axes and Sides

- **State:** in_review
- **Owner:** unassigned
- **Issue:** #373
- **Branch:** fix/issue-373-canonical-axes
- **PR:** #380
- **Paths:** see #380
- **Started:** 2026-10-06
- **Last verified:** 2026-10-06 (`4f019ef4`; collated from changes/373-drake-bodies-now-use-the-canonical-frame.md)
- **Summary:** Drake bodies now use the canonical frame: left limbs at +Y, per-coordinate axes from the parity standard, barbell left sleeve at +Y, exercise poses re-signed, feet grounded in the initial pose, and the fingerprint reports measured coordinate axes (Repository_Management#2011).
- **Next step:** Merge the PR.

### DL-#1598 · Adopt Mermaid C4 Architecture Map Contract

- **State:** in_progress
- **Owner:** local
- **Issue:** #1598 (https://github.com/D-sorganization/Repository_Management/issues/1598)
- **Branch:** docs/1598-c4-architecture-map
- **PR:** not created
- **Paths:** `docs/architecture/C4.md`, `scripts/architecture_map_contract.py`, `tests/scripts/test_architecture_map_contract.py`, `.github/workflows/architecture-map-contract.yml`
- **Started:** 2026-09-10
- **Last verified:** 2026-09-10 (`b872fa3`)
- **Next step:** Run linters, open PR, and enable auto-merge.
- **Summary:** Establish canonical Mermaid C4 architecture maps (C4Context, C4Container, Feature Map, Architecture Change Log) with automated CI contract enforcement per Epic #1594.

### DL-0001 · Temp Pr 244

- **State:** parked
- **Owner:** unassigned
- **PR:** not created
- **Paths:** `.` — scope not yet narrowed; set real globs when
  this entry is reactivated.
- **Started:** 2026-08-28
- **Last verified:** 2026-08-28 (`ca934b7`)
- **Summary:** Seeded from local branch `temp-pr-244`, which is
  2 commit(s) ahead of the default branch with no
  development-log entry.
- **Parked:** 2026-08-28 — seeded during fleet rollout. Assign a
  governing issue and set `Paths` before moving this to a live
  state; a live entry without a real issue is orphaned by
  definition.

### DL-0002 · Temp Pr 245

- **State:** parked
- **Owner:** unassigned
- **PR:** not created
- **Paths:** `.` — scope not yet narrowed; set real globs when
  this entry is reactivated.
- **Started:** 2026-08-28
- **Last verified:** 2026-08-28 (`c00febd`)
- **Summary:** Seeded from local branch `temp-pr-245`, which is
  3 commit(s) ahead of the default branch with no
  development-log entry.
- **Parked:** 2026-08-28 — seeded during fleet rollout. Assign a
  governing issue and set `Paths` before moving this to a live
  state; a live entry without a real issue is orphaned by
  definition.

## Shipped (Last 90 Days)

### DL-#2011 · Fingerprint Reports Test-Pose Origins in the Pelvis Frame

- **State:** shipped
- **Owner:** unassigned
- **Issue:** #2011
- **Branch:** feat/issue-2011-test-pose-origins
- **PR:** #383
- **Paths:** see #383
- **Started:** 2026-10-06
- **Last verified:** 2026-10-06 (`093e36fb`; collated from changes/2011-test-pose-origins.md)
- **Summary:** Re-vendored parity bundle (standard 1.2.0, topology.py, Repository_Management#2011 slice 2). The Drake fingerprint reports the pelvis rotation and segment origins at the standard's three test poses; conformance checks them against the reference forward kinematics with zero origin and pose divergences for every exercise.
- **Next step:** Shipped in PR #383.

### DL-#2019 · Vendor RM-5 Change-Fragment Tooling and Test Suite

- **State:** shipped
- **Owner:** unassigned
- **Issue:** #2019
- **Branch:** feat/2019-vendor-rm-5-change-fragment-tooling
- **PR:** #377, #378
- **Paths:** see #377
- **Started:** 2026-10-05
- **Last verified:** 2026-10-05 (`52ee5b65`; collated from changes/2019-wire-collate-changes-workflow-and-check.md)
- **Summary:** vendor RM-5 change-fragment tooling and test suite
- **Next step:** Shipped in PR #377.

Entries stay here for 90 days after merge, then move to the archive.

## Archive

Older entries live in `DEVELOPMENT_LOG_ARCHIVE_<year>.md`.
