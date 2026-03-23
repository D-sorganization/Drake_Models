# AGENTS.md -- Drake_Models

## Safety

- Never merge PRs that fail CI.
- Never use `--admin` to bypass branch protection.
- Never commit secrets, credentials, or API keys.
- Never force-push to `main`.

## Python Standards

- Python 3.10+ with `from __future__ import annotations`.
- Use `python3` not `python`.
- Type hints on all public function signatures.
- Docstrings (Google style) on all public modules, classes, and functions.
- No wildcard imports. No `print()` in `src/` (use logging).

## Architecture Principles

- **TDD** -- Write tests first, then implementation. 100% coverage target.
- **DbC** -- Design-by-Contract: preconditions validate inputs (raise `ValueError`), postconditions validate outputs (raise `AssertionError`).
- **DRY** -- Single source of truth. Geometry, inertia, XML generation defined once in `shared/`.
- **LoD** -- Law of Demeter: callers use public APIs only; never reach into internal data structures.

## Drake-Specific

- **SDFormat (SDF 1.8)** is the model description format.
- **Z-up convention**: vertical axis is Z, forward is X, gravity = (0, 0, -9.80665).
- Bodies are `<link>` elements with `<inertial>`, `<visual>`, `<collision>`.
- Joints: `<joint type="revolute">`, `<joint type="fixed">`, `<joint type="floating">`.
- Model generation produces SDF XML strings; tests verify XML structure without requiring pydrake.
- When using pydrake: `from pydrake.X import Y` (explicit imports, never attribute access on `pydrake`).

## Testing

- `python3 -m pytest tests/ -v` to run all tests.
- Unit tests in `tests/unit/`, integration tests in `tests/integration/`.
- Tests must NOT require pydrake installed -- they verify SDF XML generation only.
- Use `pytest.approx()` for floating-point comparisons.
- No `TODO` / `FIXME` in source unless tracked by a GitHub issue.

## Linting

- `ruff check src scripts tests` -- must pass with zero violations.
- `ruff format --check src scripts tests` -- must pass.
- `mypy src` -- must pass.

## Git Workflow

- Branch from `main`. Branch naming: `fix/issue-XXXX-description` or `feat/description`.
- PRs require CI to pass before merge.
- Commit messages: conventional commits (`feat:`, `fix:`, `test:`, `docs:`, `refactor:`).
