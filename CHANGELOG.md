# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
