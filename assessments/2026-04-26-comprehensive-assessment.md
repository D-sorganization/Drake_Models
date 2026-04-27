# Drake_Models — Comprehensive A-O Health Assessment

**Date:** 2026-04-26
**Branch:** fix/issue-165-validate-displacement-shape
**HEAD:** `e297d8831989cf73b1b9e4a125f7c8f2dc9c9181`
**Owner/Repo:** D-sorganization/Drake_Models

## Scores

| Criterion | Name | Score | Weight | Weighted |
|-----------|------|-------|--------|----------|
| A | Project Organization | 85 | 5% | 4.25 |
| B | Documentation | 75 | 8% | 6.00 |
| C | Testing | 60 | 12% | 7.20 |
| D | Error Handling | 51.9 | 10% | 5.19 |
| E | Performance | 45 | 7% | 3.15 |
| F | Code Quality | 90 | 10% | 9.00 |
| G | Dependency Hygiene | 60 | 8% | 4.80 |
| H | Security | 95 | 10% | 9.50 |
| I | Configuration Management | 65 | 6% | 3.90 |
| J | Observability | 40 | 7% | 2.80 |
| K | Maintenance Debt | 3.799999999999997 | 7% | 0.27 |
| L | CI/CD | 74 | 8% | 5.92 |
| M | Deployment | 70 | 5% | 3.50 |
| N | Legal & Compliance | 95 | 4% | 3.80 |
| O | Agentic Usability | 90 | 3% | 2.70 |
| **Total** | | | | **71.98** |

## Findings Summary

- **P0 (Critical):** 0
- **P1 (High):** 4
- **P2 (Medium):** 2

### P1 Findings

- **[D]** [Drake_Models] 481 lint suppressions
- **[G]** [Drake_Models] No dependency lockfile
- **[I]** [Drake_Models] Missing .env.example
- **[K]** [Drake_Models] 481 lint/type suppressions

### P2 Findings

- **[B]** [Drake_Models] README is too short (68 lines)
- **[E]** [Drake_Models] No performance benchmarks


## Evidence

```json
{
  "repo": "Drake_Models",
  "owner_repo": "D-sorganization/Drake_Models",
  "branch": "fix/issue-165-validate-displacement-shape",
  "head_sha": "e297d8831989cf73b1b9e4a125f7c8f2dc9c9181",
  "head_date": "2026-04-25",
  "A": {
    "src_files": 140,
    "test_files": 101,
    "manifests": 1,
    "gitignore_lines": 29,
    "has_readme": 1
  },
  "B": {
    "readme_lines": 68,
    "readme_headers": 5,
    "docs_files": 24,
    "md_files": 8
  },
  "C": {
    "test_py": 39,
    "test_rs": 0,
    "src_py": 54,
    "src_rs": 0
  },
  "D": {
    "bare_except": 0,
    "except_exception": 0,
    "noqa_suppressions": 481
  },
  "F": {
    "todo_fixme": 0
  },
  "G": {
    "req_lockfiles": 0
  },
  "H": {
    "secrets_raw": 0
  },
  "I": {
    "env_example": 0
  },
  "J": {
    "logging_refs": 46,
    "metrics_refs": 9
  },
  "K": {
    "suppressions": 481
  },
  "L": {
    "workflow_files": 2
  },
  "M": {
    "dockerfile": 1
  },
  "N": {
    "license": 1
  },
  "O": {
    "claude_md": 1,
    "agents_md": 1,
    "claude_lines": 80
  }
}
```