#!/usr/bin/env python3
"""setup_dev.py — One-command developer environment bootstrapper."""

import subprocess
import sys
from pathlib import Path

MIN_PYTHON = (3, 10)
_SCRIPT_DIR: Path = Path(__file__).resolve().parent
PROJECT_ROOT: Path = _SCRIPT_DIR.parent
VENDOR_UD_TOOLS = PROJECT_ROOT / "vendor" / "ud-tools"


def _require_python_version() -> None:
    """Exit unless the running Python is at least ``MIN_PYTHON``."""
    if sys.version_info < MIN_PYTHON:
        sys.exit(f"Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]}+ is required.")


def _init_submodules() -> None:
    """Initialise and update git submodules under ``PROJECT_ROOT``."""
    print("[INFO] Initialising git submodules...")
    subprocess.run(
        ["git", "submodule", "update", "--init", "--recursive"],
        cwd=PROJECT_ROOT,
        check=True,
    )


def _install_project() -> None:
    """Install the Drake_Models package (with dev extras) in editable mode."""
    print("[INFO] Installing drake-models[dev] in editable mode...")
    subprocess.run(
        [sys.executable, "-m", "pip", "install", "-e", ".[dev]"],
        cwd=PROJECT_ROOT,
        check=True,
    )


def _install_vendor_ud_tools() -> None:
    """Install the vendored ``ud-tools`` package in editable mode.

    Exits with an error if ``vendor/ud-tools`` is missing.
    """
    if not VENDOR_UD_TOOLS.is_dir():
        sys.exit(f"vendor/ud-tools not found at {VENDOR_UD_TOOLS}.")
    print("[INFO] Installing vendor/ud-tools in editable mode...")
    subprocess.run(
        [sys.executable, "-m", "pip", "install", "-e", str(VENDOR_UD_TOOLS)],
        cwd=PROJECT_ROOT,
        check=True,
    )


def main() -> None:
    """Bootstrap the developer environment for Drake_Models.

    Checks the Python version, initialises git submodules, installs the
    package in editable mode, and installs the vendored ud-tools dependency.

    Raises:
        SystemExit: If the Python version is below the minimum or if
            vendor/ud-tools is not found.
    """
    _require_python_version()
    _init_submodules()
    _install_project()
    _install_vendor_ud_tools()
    print("[ OK ] Development environment ready!")


if __name__ == "__main__":
    main()
