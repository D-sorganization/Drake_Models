# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] - 2026-03-30

### Fixed (A-N Assessment Remediation — issue #88)

- **DbC**: Added input validation (ValueError) to `main` in `__main__.py` for `--mass`, `--height`, and `--plates` CLI arguments.
- **DbC**: Added precondition guards to `_interpolate_phases` in `inverse_kinematics.py` and `interpolate_trajectory` in `trajectory_optimizer.py`.
- **LoD**: Broke `Path(__file__).resolve().parent.parent` chain in `setup_dev.py` into named `_SCRIPT_DIR` and `PROJECT_ROOT` intermediates.
- **LoD**: Extracted `sys.stdout` into a local variable in `__main__.py` to avoid repeated chained attribute access.
- **Documentation**: Added missing docstrings to `main` in `setup_dev.py`, `__init__` in `ExerciseModelBuilder`, and all `__post_init__` methods in dataclasses (`BarbellSpec`, `BodyModelSpec`, `TrajectoryConfig`, `TrajectoryResult`, `ExercisePhase`, `ExerciseObjective`).
- **Documentation**: Added missing docstrings to `shaft_radius` and `sleeve_radius` properties in `BarbellSpec`.

## [0.1.0] - 2026-03-22

### Added

- Full-body multibody model with 15 segments and bilateral joints.
- Olympic barbell model (men's and women's) with sleeve/shaft geometry.
- Five exercise builders: squat, deadlift, bench press, snatch, clean and jerk.
- Bilateral barbell attachment for all grip-based exercises.
- Meaningful initial joint poses for each exercise.
- Design-by-Contract precondition and postcondition checks.
- Geometry utilities: cylinder, rectangular prism, sphere inertia.
- CLI entry point: `python3 -m drake_models <exercise>`.
- GitHub Actions CI with lint, format, type check, and test stages.
- Dependabot configuration for pip and GitHub Actions.
- Hypothesis property-based tests for geometry functions.
- Comprehensive unit and integration test suite.

### Changed

- Replaced custom `indent_xml` with stdlib `ET.indent()`.
- Removed phantom dependencies (lxml, scipy) from pyproject.toml.
- Tightened mypy configuration (`disallow_untyped_defs = true`).
- Extracted magic numbers to named constants in body_model.py.

### Fixed

- Dockerfile dependencies now match pyproject.toml.
- Barbell attachment is now bilateral (both hands) for all grip exercises.
