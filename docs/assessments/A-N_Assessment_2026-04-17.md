# A-N Assessment - Drake_Models - 2026-04-17

Run time: 2026-04-17T08:01:19.6221680Z UTC
Sync status: pull-blocked
Sync notes: ff-only pull failed: fatal: couldn't find remote ref codex/an-assessment-2026-04-14

Overall grade: C (77/100)

## Coverage Notes
- Reviewed tracked first-party files from git ls-files, excluding cache, build, vendor, virtualenv, temp, and generated output directories.
- Reviewed 132 tracked files, including 88 code files, 35 test files, 2 CI files, 2 config/build files, and 35 docs/onboarding files.
- This is a read-only static assessment of committed files. TDD history and confirmed Law of Demeter semantics require commit-history review and deeper call-graph analysis; this report distinguishes those limits from confirmed file evidence.

## Category Grades
### A. Architecture and Boundaries: B (82/100)
Assesses source organization and boundary clarity from tracked first-party layout.
- Evidence: `132 tracked first-party files`
- Evidence: `52 files under source-like directories`

### B. Build and Dependency Management: C (72/100)
Assesses committed build, dependency, and tool configuration.
- Evidence: `Dockerfile`
- Evidence: `pyproject.toml`

### C. Configuration and Environment Hygiene: C (78/100)
Checks whether runtime and developer configuration is explicit.
- Evidence: `Dockerfile`
- Evidence: `pyproject.toml`

### D. Contracts, Types, and Domain Modeling: B (82/100)
Design by Contract evidence includes validation, assertions, typed models, explicit raised errors, and invariants.
- Evidence: `rust_core/src/lib.rs`
- Evidence: `src/drake_models/exercises/base.py`
- Evidence: `src/drake_models/exercises/bench_press/bench_press_model.py`
- Evidence: `src/drake_models/exercises/squat/squat_model.py`
- Evidence: `src/drake_models/optimization/exercise_objectives.py`
- Evidence: `src/drake_models/optimization/inverse_kinematics.py`
- Evidence: `src/drake_models/optimization/objectives/__init__.py`
- Evidence: `src/drake_models/optimization/trajectory_optimizer.py`
- Evidence: `src/drake_models/shared/barbell/barbell_model.py`
- Evidence: `src/drake_models/shared/body/body_anthropometrics.py`

### E. Reliability and Error Handling: C (76/100)
Reliability is graded from test presence plus explicit validation/error-handling signals.
- Evidence: `docs/assessments/Assessment_C_Test_Coverage.md`
- Evidence: `tests/__init__.py`
- Evidence: `tests/integration/__init__.py`
- Evidence: `tests/integration/test_all_exercises_build.py`
- Evidence: `tests/integration/test_drake_loading.py`
- Evidence: `rust_core/src/lib.rs`
- Evidence: `src/drake_models/exercises/base.py`
- Evidence: `src/drake_models/exercises/bench_press/bench_press_model.py`
- Evidence: `src/drake_models/exercises/squat/squat_model.py`
- Evidence: `src/drake_models/optimization/exercise_objectives.py`

### F. Function, Module Size, and SRP: B (84/100)
Evaluates function size, script/module size, and single responsibility using static size signals.
- Evidence: No direct tracked-file evidence found for this category.

### G. Testing and TDD Posture: B (82/100)
TDD history cannot be confirmed statically; grade reflects committed automated test posture.
- Evidence: `docs/assessments/Assessment_C_Test_Coverage.md`
- Evidence: `tests/__init__.py`
- Evidence: `tests/integration/__init__.py`
- Evidence: `tests/integration/test_all_exercises_build.py`
- Evidence: `tests/integration/test_drake_loading.py`
- Evidence: `tests/parity/__init__.py`
- Evidence: `tests/parity/test_parity_compliance.py`
- Evidence: `tests/unit/__init__.py`
- Evidence: `tests/unit/exercises/__init__.py`
- Evidence: `tests/unit/exercises/test_bench_press.py`
- Evidence: `tests/unit/exercises/test_clean_and_jerk.py`
- Evidence: `tests/unit/exercises/test_deadlift.py`

### H. CI/CD and Automation: C (78/100)
Checks for tracked CI/CD workflow files.
- Evidence: `.github/workflows/ci-standard.yml`
- Evidence: `.github/workflows/rust-ci.yml`

### I. Security and Secret Hygiene: B (82/100)
Secret scan is regex-based; findings require manual confirmation.
- Evidence: No direct tracked-file evidence found for this category.

### J. Documentation and Onboarding: B (82/100)
Checks docs, README, onboarding, and release documents.
- Evidence: `AGENTS.md`
- Evidence: `CHANGELOG.md`
- Evidence: `CLAUDE.md`
- Evidence: `CODE_OF_CONDUCT.md`
- Evidence: `CONTRIBUTING.md`
- Evidence: `Dockerfile`
- Evidence: `LICENSE`
- Evidence: `README.md`
- Evidence: `SECURITY.md`
- Evidence: `SPEC.md`
- Evidence: `docs/assessments/A-N_Assessment_2026-04-02.md`
- Evidence: `docs/assessments/A-N_Assessment_2026-04-04.md`

### K. Maintainability, DRY, and Duplication: B (80/100)
DRY is assessed through duplicate filename clusters and TODO/FIXME density as static heuristics.
- Evidence: No direct tracked-file evidence found for this category.

### L. API Surface and Law of Demeter: F (58/100)
Law of Demeter is approximated with deep member-chain hints; confirmed violations require semantic review.
- Evidence: `examples/generate_all_models.py`
- Evidence: `src/drake_models/__main__.py`
- Evidence: `src/drake_models/exercises/base.py`
- Evidence: `src/drake_models/exercises/bench_press/bench_press_model.py`
- Evidence: `src/drake_models/exercises/sit_to_stand/sit_to_stand_model.py`
- Evidence: `src/drake_models/exercises/squat/squat_model.py`
- Evidence: `src/drake_models/optimization/exercise_objectives.py`
- Evidence: `src/drake_models/optimization/objectives/_helpers.py`
- Evidence: `src/drake_models/optimization/objectives/bench_press.py`
- Evidence: `src/drake_models/optimization/objectives/clean_and_jerk.py`

### M. Observability and Operability: C (74/100)
Checks for logging, metrics, monitoring, and operational artifacts.
- Evidence: `docs/assessments/Assessment_L_Logging.md`
- Evidence: `src/drake_models/shared/body/body_anthropometrics.py`

### N. Governance, Licensing, and Release Hygiene: C (74/100)
Checks ownership, release, contribution, security, and license metadata.
- Evidence: `CHANGELOG.md`
- Evidence: `CONTRIBUTING.md`
- Evidence: `LICENSE`
- Evidence: `SECURITY.md`
- Evidence: `docs/assessments/Assessment_F_Security.md`

## Explicit Engineering Practice Review
- TDD: Automated tests are present, but red-green-refactor history is not confirmable from static files.
- DRY: No repeated filename clusters met the static threshold.
- Design by Contract: Validation/contract signals were found in tracked code.
- Law of Demeter: Deep member-chain hints were found and should be semantically reviewed.
- Function size and SRP: No large module or coarse long-definition signal crossed the threshold.

## Key Risks
- Deep member-chain usage may indicate Law of Demeter pressure points.

## Prioritized Remediation Recommendations
1. Review deep member chains and introduce boundary methods where object graph traversal leaks across modules.

## Actionable Issue Candidates
### Review deep object traversal hotspots
- Severity: medium
- Problem: Deep member-chain hints found in: examples/generate_all_models.py; src/drake_models/__main__.py; src/drake_models/exercises/base.py; src/drake_models/exercises/bench_press/bench_press_model.py; src/drake_models/exercises/sit_to_stand/sit_to_stand_model.py; src/drake_models/exercises/squat/squat_model.py; src/drake_models/optimization/exercise_objectives.py; src/drake_models/optimization/objectives/_helpers.py
- Evidence: Category L found repeated chains with three or more member hops.
- Impact: Law of Demeter pressure can make APIs brittle and increase coupling.
- Proposed fix: Review hotspots and introduce boundary methods or DTOs where callers traverse object graphs.
- Acceptance criteria: Hotspots are documented, simplified, or justified; tests cover any API boundary changes.
- Expectations: Law of Demeter, SRP, maintainability

