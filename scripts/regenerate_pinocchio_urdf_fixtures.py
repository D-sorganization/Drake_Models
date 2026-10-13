#!/usr/bin/env python3
"""Regenerate the committed Pinocchio_Models URDF fixtures (issue #374).

The Drake cross-engine test (``tests/parity/test_pinocchio_urdf_in_drake.py``)
reads Pinocchio_Models URDFs from ``tests/fixtures/pinocchio_urdf/`` so that
Drake_Models has no runtime or CI dependency on Pinocchio_Models. This script
rebuilds those files from a CLEAN Pinocchio_Models checkout and records its
HEAD SHA and each file's SHA-256 (over LF text) in ``PROVENANCE.json``.

Usage::

    git -C ../Pinocchio_Models checkout <sha>
    python3 scripts/regenerate_pinocchio_urdf_fixtures.py \
        --pinocchio-repo ../Pinocchio_Models

Generating URDFs needs only numpy (Pinocchio_Models builds the XML itself);
the ``pinocchio`` engine is not required.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import re
import subprocess
import sys
from collections.abc import Callable, Mapping
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = PROJECT_ROOT / "tests" / "fixtures" / "pinocchio_urdf"
PROVENANCE_FILENAME = "PROVENANCE.json"
PROVENANCE_SCHEMA = "pinocchio-urdf-fixtures/v1"
SOURCE_REPO = "D-sorganization/Pinocchio_Models"
GENERATOR = "pinocchio_models.shared.parity.fingerprint.build_urdf"
_FULL_SHA = re.compile(r"[0-9a-f]{40}")


def write_fixtures(urdfs: Mapping[str, str], out_dir: Path, source_sha: str) -> Path:
    """Write ``<exercise>.urdf`` files (LF, UTF-8) and ``PROVENANCE.json``.

    Returns:
        The provenance file path.

    Raises:
        ValueError: If *urdfs* is empty or *source_sha* is not a full 40-hex SHA.
    """
    if not urdfs:
        raise ValueError("urdfs must contain at least one exercise")
    if not _FULL_SHA.fullmatch(source_sha):
        raise ValueError(f"source_sha must be a full 40-hex SHA: {source_sha!r}")
    out_dir.mkdir(parents=True, exist_ok=True)
    files: dict[str, str] = {}
    for exercise, urdf in sorted(urdfs.items()):
        data = (urdf.rstrip("\n") + "\n").encode("utf-8")
        (out_dir / f"{exercise}.urdf").write_bytes(data)
        files[f"{exercise}.urdf"] = hashlib.sha256(data).hexdigest()
    provenance = {
        "schema": PROVENANCE_SCHEMA,
        "source_repo": SOURCE_REPO,
        "source_sha": source_sha,
        "generator": GENERATOR,
        "regenerate": (
            "python3 scripts/regenerate_pinocchio_urdf_fixtures.py "
            "--pinocchio-repo <clean Pinocchio_Models checkout>"
        ),
        "files": files,
    }
    path = out_dir / PROVENANCE_FILENAME
    path.write_bytes((json.dumps(provenance, indent=2) + "\n").encode("utf-8"))
    return path


def _git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args], check=True, capture_output=True, text=True
    ).stdout.strip()


def _clean_head_sha(repo: Path) -> str:
    """HEAD SHA of *repo*; refuses a dirty tree (the SHA would not reproduce)."""
    if _git(repo, "status", "--porcelain", "--untracked-files=no"):
        raise SystemExit(f"{repo} has uncommitted changes; commit or stash first")
    return _git(repo, "rev-parse", "HEAD")


def _builder(repo: Path) -> tuple[Callable[[str], str], list[str]]:
    """Import ``build_urdf`` and the exercise list from the checkout's ``src``."""
    sys.path.insert(0, str(repo / "src"))
    fingerprint = importlib.import_module("pinocchio_models.shared.parity.fingerprint")
    model_pack = importlib.import_module("pinocchio_models.model_pack")
    return fingerprint.build_urdf, list(model_pack.list_exercises())


def main(argv: list[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pinocchio-repo", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT_DIR)
    args = parser.parse_args(argv)
    repo = args.pinocchio_repo.resolve()
    sha = _clean_head_sha(repo)
    build_urdf, exercises = _builder(repo)
    path = write_fixtures({e: build_urdf(e) for e in exercises}, args.out, sha)
    print(f"wrote {len(exercises)} URDFs from {SOURCE_REPO}@{sha} -> {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
