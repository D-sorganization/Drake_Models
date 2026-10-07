"""Rust jobs must not share ``~/.rustup`` / ``~/.cargo`` on the fleet (RM#2021).

Concurrent jobs on a persistent self-hosted runner share the toolchain
directories, so one job's install or update can delete binaries from under
another. Every job that installs or runs Rust therefore sets job-level
``RUSTUP_HOME`` and ``CARGO_HOME`` inside ``github.workspace``.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Any

import pytest
import yaml

pytestmark = pytest.mark.unit

WORKFLOWS_DIR = Path(__file__).resolve().parents[2] / ".github" / "workflows"

RUST_USES = re.compile(r"rust-toolchain|setup-rust|rustup|maturin|tauri", re.IGNORECASE)
RUST_RUN = re.compile(r"(?<![\w-])(cargo|rustup|rustc|maturin)(?![\w-])")
WORKSPACE_PREFIX = "${{ github.workspace }}/"
HOME_VARS = ("RUSTUP_HOME", "CARGO_HOME")


def _jobs() -> list[tuple[str, str, dict[str, Any]]]:
    found = []
    for path in sorted(WORKFLOWS_DIR.glob("*.yml")):
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        for name, job in (data.get("jobs") or {}).items():
            found.append((path.name, name, job))
    return found


def _uses_rust(job: dict[str, Any]) -> bool:
    for step in job.get("steps") or []:
        if RUST_USES.search(str(step.get("uses", ""))):
            return True
        if RUST_RUN.search(str(step.get("run", ""))):
            return True
    return False


RUST_JOBS = [(f, n, j) for f, n, j in _jobs() if _uses_rust(j)]


def test_detects_at_least_one_rust_job() -> None:
    assert any(f == "rust-ci.yml" for f, _, _ in RUST_JOBS)


@pytest.mark.parametrize(
    ("workflow", "name", "job"),
    RUST_JOBS,
    ids=[f"{f}::{n}" for f, n, _ in RUST_JOBS],
)
@pytest.mark.parametrize("var", HOME_VARS)
def test_rust_job_isolates_toolchain_home(
    workflow: str, name: str, job: dict[str, Any], var: str
) -> None:
    value = str((job.get("env") or {}).get(var, ""))
    assert value.startswith(WORKSPACE_PREFIX), (
        f"{workflow}::{name} must set job-level {var} under github.workspace "
        f"(got {value!r})"
    )


@pytest.mark.parametrize(
    ("workflow", "name", "job"),
    RUST_JOBS,
    ids=[f"{f}::{n}" for f, n, _ in RUST_JOBS],
)
def test_rust_homes_are_distinct(workflow: str, name: str, job: dict[str, Any]) -> None:
    env = job.get("env") or {}
    assert env.get("RUSTUP_HOME") != env.get("CARGO_HOME")
