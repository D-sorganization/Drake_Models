# Drake_Models Specification

## Purpose

`Drake_Models` provides Drake-compatible multibody model generators for classical barbell exercises. The project emits SDFormat 1.8 XML for barbell and human-body configurations that can be loaded into Drake with `Parser().AddModelsFromString()` or equivalent file-based loading paths.

The maintained public surface is Python-first. A small optional Rust core exists under `rust_core/` for accelerator work, but the canonical model definitions live in `src/drake_models/`.

## Repository Structure

### Python package

- `src/drake_models/__main__.py` provides the CLI entry point exposed by `drake-models`.
- `src/drake_models/exercises/` contains exercise-specific builders.
- `src/drake_models/shared/` contains reusable geometry, barbell, body, contract, parity, and XML helper code.
- `src/drake_models/optimization/` contains exercise objectives, inverse-kinematics helpers, and trajectory optimization utilities.

### Exercise builders

The supported exercise builders currently include:

- `squat`
- `deadlift`
- `bench_press`
- `snatch`
- `clean_and_jerk`
- `gait`
- `sit_to_stand`

Each exercise module is expected to build on the shared body and barbell primitives rather than re-implementing geometry or XML assembly.

### Shared model layer

The shared layer is the single source of truth for geometry, inertia, SDF XML assembly, and contract checks:

- `shared/body/` implements the full-body anthropometric model and staged SDF construction.
- `shared/barbell/` implements the Olympic barbell model.
- `shared/utils/geometry.py` and `shared/utils/sdf_helpers.py` provide reusable math and XML helpers.
- `shared/contracts/` provides precondition and postcondition helpers used by the builders.

The body model uses the repo’s Z-up convention, with gravity aligned to `(0, 0, -9.80665)`. Drake's world frame is the canonical frame of the fleet parity standard: X forward, Y left, Z up. Left bilateral segments (and the barbell's left sleeve) sit at +Y, right ones at -Y. Joint frames are aligned with their parent body at q=0, so each joint's `<axis><xyz>` is the canonical rotation axis of its segment relative to the pelvis; `shared/body/joint_axes.py` is the single table (limb flexion about -Y, adduction/deviation/inversion about X and long-axis rotation about Z, both mirrored on the left; trunk and neck flexion about +Y, lumbar lateral bend about -X, lumbar rotation about +Z). Positive hip/shoulder/knee/elbow/wrist/ankle flexion swings the distal segment forward (knee flexion is negative), positive adduction is toward the midline and positive rotation is internal. The loader places a free pelvis so the lowest foot sole point rests on the ground in the initial pose. The full-body model is assembled from a staged builder that creates pelvis, spine/head, upper-limb, lower-limb, and foot-contact elements in sequence.

## Model Generation Contract

All maintained model generators must produce valid SDFormat 1.8 XML. The generated XML should preserve Drake compatibility without requiring `pydrake` at test time.

The core contract is:

- validate user inputs with explicit preconditions
- build SDF XML through shared helpers
- keep exercise modules thin and exercise-specific
- prefer explicit imports and package-safe execution
- keep behavior deterministic enough for XML-structure tests

The barbell model is represented as a three-link assembly with fixed joints. The human body model is represented as a segmented multibody tree with compound joint chains implemented via virtual links where needed to satisfy SDF tree constraints.

## Optional Drake Integration

`pydrake` is an optional runtime dependency, not a test requirement.

- The package may be installed with the `drake` optional extra when Drake integration is needed.
- Tests verify XML structure and model generation without importing or requiring `pydrake`.
- The user-facing documentation and examples may show Drake loading code, but the package’s correctness is judged by SDF generation and XML validity.

## Command-Line Interface

`python3 -m drake_models` and the `drake-models` console script are the supported entry points.

The CLI accepts:

- the exercise name
- body mass
- body height
- barbell plate mass per side
- an optional output path
- a verbose logging flag

The CLI should remain a thin wrapper around the exercise builder modules.

## Testing Strategy

The repository uses `pytest` for unit and integration coverage.

- Unit tests live under `tests/unit/`.
- Integration tests live under `tests/integration/`.
- Test coverage should exercise the XML structure and builder behavior without requiring Drake.
- `pydrake`-dependent behavior, if present, should be isolated so the default test suite still runs in a plain Python environment.
- Biomechanics-specific validation: 43 comprehensive design-by-contract tests covering model loading, joint limits, link properties, kinematic integrity, and contact geometry (PR #220).

The current validation targets are:

- `python3 -m pytest tests/ -v`
- `ruff check src scripts tests examples`
- `ruff format --check src scripts tests examples`
- `mypy src`

## CI Expectations

Continuous integration is expected to enforce:

- linting with Ruff
- formatting checks with Ruff
- type checking with mypy
- tests on supported Python versions
- artifact and placeholder hygiene

CI should stay compatible with documentation-only changes and should not require `pydrake` for the base suite.

## Architecture Map Contract

The repository maintains an automated architecture contract per Epic #1594:

- **Canonical Map**: `docs/architecture/C4.md` containing `C4Context` and `C4Container` Mermaid diagrams.
- **Traceability**: Feature Map table linking key capabilities to component paths, public interfaces, and test evidence.
- **Contract Enforcement**: `scripts/architecture_map_contract.py` validates required sections and table structures via CI (`.github/workflows/architecture-map-contract.yml`).

## Engine Parity Contract

Cross-engine parameters come from the fleet parity standard vendored at
`src/drake_models/shared/parity/_canonical/` (`biomech_parity_standard.json`,
`conformance.py`, `assemble.py`, `MANIFEST.json`). The canonical source is
`Repository_Management/shared_scripts/model_parity/`; vendored files are never
edited here and `tests/parity/` verifies their hashes against `MANIFEST.json`.

- `shared/parity/standard.py` and the body segment table are computed from the
  bundle; no constants are duplicated.
- `shared/parity/fingerprint.py` loads every exercise's generated SDFormat model in
  the real drake engine and reports a `model-fingerprint/v1`
  (`python -m drake_models.shared.parity.fingerprint --all --out DIR`).
- `tests/parity/test_engine_conformance.py` runs in default CI with the engine
  installed and fails on any divergence from the standard that is not listed,
  with an issue reference, in `shared/parity/parity_divergences.json`. Ledger
  entries that no longer diverge fail as stale, so the ledger only shrinks.
- `shared/parity/axes_probe.py` measures every coordinate's rotation axis in the
  real engine (all joints at zero, one coordinate rotated by the standard's probe
  angle) so the standard's `kinematics` checks (`axis.*`, `side.*`) run against
  Drake itself; none is ledgered.
- `model_pack.yaml` declares honest `capabilities` levels (`none`, `partial`,
  `full`); `full` requires a public API and a real-engine test as evidence.

## Change Log

| Date       | PR   | Changes                                                                                                                                                                                                                                                                       |
| ---------- | ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2026-10-10 | #392 | Add standalone CalcInverseDynamics API and clean up solver comments |
| 2026-10-10 | #397 | Silence pydrake warning on biomech:initial_pose by stripping it before parsing |
| 2026-10-10 | #393 | Weld the right hand to the barbell via a Drake weld constraint, verified on real pydrake |
| 2026-10-07 | #388 | CI: isolate RUSTUP_HOME/CARGO_HOME per workspace in rust-ci (RM#2021) |
| 2026-10-07 | #385 | SECURITY: guard fork PRs off the self-hosted fleet (RM#1989); vendor fork_pr_runner_guard and wire it into CI |
| 2026-10-06 | #380 | Drake bodies now use the canonical frame: left limbs at +Y, per-coordinate axes from the parity standard, barbell left sleeve at +Y, exercise poses re-signed, feet grounded in the initial pose, and the fingerprint reports measured coordinate axes (Repository_Management#2011). |
| 2026-10-06 | #383 | Re-vendored parity bundle (standard 1.2.0, topology.py, Repository_Management#2011 slice 2). The Drake fingerprint reports the pelvis rotation and segment origins at the standard's three test poses; conformance checks them against the reference forward kinematics with zero origin and pose divergences for every exercise. |
| 2026-10-05 | #378 | wire collate-changes workflow and check_spec_freshness into spec-check |
| 2026-10-05 | #377 | vendor RM-5 change-fragment tooling and test suite |
| 2026-10-05 | #361 | chore(changes): vendor RM-5 change-fragment tooling and test suite (ref Repository_Management#2019) |
| 2026-10-05 | #360 | Generated SDF now parses in pydrake (namespaced initial pose, no floating joint type, unique filter-group names, bilateral parent links, poses relative_to parent); new loader applies pose and canonical gravity; real-engine parity conformance against the fleet standard. |
| 2026-09-14 | #336 | Fix expression collision in local-only-runner-guard workflow (#335). |
| 2026-09-14 | #334 | Downgraded non-existent workflow action versions to @v4/@v5 across workflows (#333). |
| 2026-09-10 | #330 | Adopt maintainable Mermaid C4 architecture-map contract (#1598) |
