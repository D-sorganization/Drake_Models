"""Model-pack entry point for UpstreamDrift launcher integration.

Exposes the three functions required by the ``biomech.model_pack`` entry-point
group:

* :func:`resolve` -- locate the directory that holds the exercise sub-packages.
* :func:`manifest` -- return the parsed ``model_pack.yaml`` manifest.
* :func:`list_exercises` -- enumerate exercise ids declared in the manifest.

The manifest itself (``model_pack.yaml``) ships at the repository root and is
included in the wheel via ``[tool.hatch.build.targets.wheel.force-include]``
so installed consumers can load it without the source tree.
"""

from __future__ import annotations

from importlib import resources
from pathlib import Path
from typing import Any

import yaml

_MANIFEST_FILENAME = "model_pack.yaml"


def _load_manifest_text() -> str:
    """Return the raw YAML text of the manifest, search wheel + source tree.

    The wheel ships ``model_pack.yaml`` inside the ``drake_models`` package
    via ``force-include``; the source tree keeps it at the repository root.
    Try the package resource first, then fall back to walking up from this
    file.
    """
    try:
        pkg_files = resources.files("drake_models")
        candidate = pkg_files.joinpath(_MANIFEST_FILENAME)
        if candidate.is_file():
            return candidate.read_text(encoding="utf-8")
    except (FileNotFoundError, ModuleNotFoundError):
        pass

    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate_path = parent / _MANIFEST_FILENAME
        if candidate_path.is_file():
            return candidate_path.read_text(encoding="utf-8")

    raise FileNotFoundError(
        f"Could not locate {_MANIFEST_FILENAME} in package resources or source tree",
    )


def manifest() -> dict[str, Any]:
    """Return the parsed ``model_pack.yaml`` manifest as a ``dict``.

    Returns:
        Mapping with keys such as ``schema``, ``engine``, ``axis_convention``,
        and ``exercises`` (a list of ``{"id": str, "path": str}`` entries).

    Raises:
        FileNotFoundError: When the manifest cannot be located.
        yaml.YAMLError: When the manifest is not valid YAML.
    """
    data = yaml.safe_load(_load_manifest_text())
    if not isinstance(data, dict):
        raise ValueError(
            f"{_MANIFEST_FILENAME} must parse to a mapping, got {type(data).__name__}",
        )
    return data


def resolve() -> Path:
    """Return the absolute path to the exercises root directory.

    The path is taken from ``models_root`` in the manifest and resolved
    relative to the directory containing ``model_pack.yaml``. The resulting
    directory is guaranteed to exist (postcondition).

    Returns:
        Absolute :class:`~pathlib.Path` to the models root.

    Raises:
        FileNotFoundError: When neither the manifest nor the resolved root
            exists on disk.
    """
    here = Path(__file__).resolve()
    manifest_dir: Path | None = None
    for parent in here.parents:
        if (parent / _MANIFEST_FILENAME).is_file():
            manifest_dir = parent
            break

    if manifest_dir is None:
        # Installed wheel: manifest sits next to this module; exercises live
        # in the package itself.
        package_dir = here.parent
        if (package_dir / _MANIFEST_FILENAME).is_file():
            exercises_dir = package_dir / "exercises"
            assert exercises_dir.is_dir(), (
                f"resolved exercises dir does not exist: {exercises_dir}"
            )
            return exercises_dir
        raise FileNotFoundError(
            f"Could not locate {_MANIFEST_FILENAME} to resolve models root",
        )

    data = manifest()
    models_root = data.get("models_root", "src/drake_models/exercises")
    resolved = (manifest_dir / models_root).resolve()
    assert resolved.is_dir(), f"resolved models root does not exist: {resolved}"
    return resolved


def list_exercises() -> list[str]:
    """Return the ordered list of exercise ids declared in the manifest.

    Returns:
        A list of exercise id strings (e.g. ``["squat", "deadlift", ...]``).
    """
    data = manifest()
    exercises = data.get("exercises", [])
    return [str(entry["id"]) for entry in exercises]
