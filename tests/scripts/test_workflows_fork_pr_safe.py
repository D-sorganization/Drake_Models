"""No workflow in this repo can run fork PR code on the self-hosted fleet.

Runs the vendored ``scripts/fork_pr_runner_guard.py`` against the real
``.github/workflows`` directory (RM#1989, issue #384).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from scripts import fork_pr_runner_guard as guard

pytestmark = pytest.mark.unit

WORKFLOWS_DIR = Path(__file__).resolve().parents[2] / ".github" / "workflows"


def test_workflows_directory_exists() -> None:
    assert WORKFLOWS_DIR.is_dir()


def test_no_job_runs_fork_pr_code_on_the_fleet() -> None:
    assert guard.find_violations(WORKFLOWS_DIR) == []
