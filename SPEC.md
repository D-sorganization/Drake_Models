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

The body model uses the repo’s Z-up convention, with gravity aligned to `(0, 0, -9.80665)`. The full-body model is assembled from a staged builder that creates pelvis, spine/head, upper-limb, lower-limb, and foot-contact elements in sequence.

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

## Maintenance Rules

- Keep geometry, inertia, and XML generation in `shared/`.
- Do not duplicate anthropometric tables or barbell constants in exercise modules.
- Keep public APIs stable when possible; prefer compatibility facades over breaking import paths.
- Update this spec when the maintained package layout, supported exercises, or validation expectations change.

