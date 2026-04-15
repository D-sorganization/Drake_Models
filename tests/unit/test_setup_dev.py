"""Tests for the developer setup script orchestration helpers."""

from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_module() -> object:
    repo_root = Path(__file__).resolve().parents[2]
    script = repo_root / "scripts" / "setup_dev.py"
    spec = importlib.util.spec_from_file_location("setup_dev", script)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_bootstrap_steps_preserves_setup_order() -> None:
    mod = _load_module()

    steps = mod._bootstrap_steps()  # type: ignore[attr-defined]

    assert [step.__name__ for step in steps] == [
        "_require_python_version",
        "_init_submodules",
        "_install_project",
        "_install_vendor_ud_tools",
    ]


def test_report_ready_prints_success_message(capsys) -> None:
    mod = _load_module()

    mod._report_ready()  # type: ignore[attr-defined]

    assert "Development environment ready" in capsys.readouterr().out
