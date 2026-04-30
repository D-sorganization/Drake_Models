# CLAUDE.md -- Drake_Models

## Branch Policy

All work on `main` branch. PRs target `main`.

## What This Is

Drake multibody simulation models for classical barbell exercises (squat,
deadlift, bench press, snatch, clean & jerk) plus gait and sit-to-stand.
Models are generated as SDFormat (SDF 1.8) XML and can be loaded by the
Drake physics engine for biomechanical analysis.

## Key Directories

- `src/drake_models/exercises/` -- exercise-specific model builders
- `src/drake_models/shared/` -- shared body, barbell, geometry, contracts
- `src/drake_models/optimization/` -- trajectory optimization and IK
- `tests/unit/` -- unit tests (no pydrake required)
- `tests/integration/` -- integration tests (XML validation, optional pydrake)
- `tests/parity/` -- cross-implementation parity checks
- `rust_core/` -- Rust FFI core (Cargo workspace)
- `examples/` -- usage examples and model generation scripts

## Python and Tooling

- **Python 3.10+**. Use `python3`.
- **Formatter:** `ruff format` (line length 88).
- **Linter:** `ruff check` with rules: E, F, I, UP, B, T201, SIM, C4, PIE, PLE, FURB, RSE, LOG, PERF, RET.
- **Type checker:** `mypy src` (strict: `disallow_untyped_defs = true`).
- **Tests:** pytest with 80% coverage minimum.

## Development Commands

```bash
ruff check src scripts tests                    # lint
ruff format --check src scripts tests           # format check
mypy src                                        # type check
python3 -m pytest tests/ -v                     # run all tests
python3 -m pytest --cov=src --cov-fail-under=80 # tests with coverage
python3 -m drake_models squat --mass 80         # generate a model
```

## CI Requirements (All Must Pass)

1. `ruff check` -- zero violations
2. `ruff format --check` -- zero diffs
3. `mypy src` -- zero errors
4. No TODO/FIXME in source unless tracked by a GitHub issue
5. `pip-audit` -- no known vulnerabilities (advisory ignores configured)
6. `bandit -r src/` -- no high-severity findings
7. pytest with **80% coverage minimum** across Python 3.10, 3.11, 3.12

## Architecture Principles

- **TDD:** Tests first. 80% coverage floor enforced in CI.
- **DbC:** Preconditions in `shared/contracts/preconditions.py`, postconditions in `postconditions.py`. Inputs raise `ValueError`, output violations raise `AssertionError`.
- **DRY:** Geometry, inertia, XML generation live once in `shared/`. Exercise builders inherit from `ExerciseModelBuilder` base class.
- **LoD:** Callers use public APIs only. Complex internals stay behind module boundaries.

## Drake-Specific Conventions

- **SDFormat 1.8** model description format.
- **Z-up**: vertical = Z, forward = X, gravity = (0, 0, -9.80665).
- Bodies are `<link>` with `<inertial>`, `<visual>`, `<collision>`.
- Joints: `revolute`, `fixed`, `floating`.
- Models generate SDF XML strings; tests verify XML without pydrake.

## Coding Standards

- `from __future__ import annotations` in every module.
- Type hints on all public function signatures.
- Google-style docstrings on all public modules, classes, and functions.
- No `print()` in `src/` -- use `logging`.
- No f-strings in logging calls -- use `%s` lazy formatting.
- No wildcard imports.
- No bare `except:` -- always specify exception types.

## Git Workflow

- Branch from `main`. Naming: `fix/issue-XXXX-desc` or `feat/desc`.
- PRs require CI green before merge.
- Conventional commits: `feat:`, `fix:`, `test:`, `docs:`, `refactor:`.
- Never force-push to `main`.
