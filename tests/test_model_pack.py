"""Tests for the ``drake_models.model_pack`` UpstreamDrift integration.

Mirrors the assertions used in the MuJoCo coordination issue so the two
packages present an identical surface to the UpstreamDrift launcher.

The ``pydrake`` parse test is marked ``live_simulation`` because it requires
a working Drake install; the default pytest lane deselects that marker.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

from drake_models.model_pack import list_exercises, manifest, resolve

EXPECTED_EXERCISES = [
    "squat",
    "deadlift",
    "bench_press",
    "snatch",
    "clean_and_jerk",
    "gait",
    "sit_to_stand",
]


def _subprocess_env() -> dict[str, str]:
    """Return environment for subprocess with parent's PYTHONPATH preserved.

    This ensures subprocesses spawned by tests can find the drake_models module
    when running in CI with a venv, where the venv's site-packages must be
    explicitly passed via PYTHONPATH.
    """
    env = os.environ.copy()
    # Get current Python's site-packages by running a subprocess to extract it
    # This approach works both in editable installs and venv setups
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import sys; print(';'.join(p for p in sys.path if p))",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    pythonpath = result.stdout.strip()
    if pythonpath:
        existing_pp = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = (
            f"{pythonpath};{existing_pp}"
            if existing_pp
            else pythonpath
        )
    return env


class TestManifest:
    """Tests for :func:`drake_models.model_pack.manifest`."""

    def test_manifest_is_dict(self) -> None:
        """``manifest()`` returns a mapping."""
        data = manifest()
        assert isinstance(data, dict)

    def test_manifest_schema(self) -> None:
        """Manifest declares the ``model_pack/v1`` schema."""
        assert manifest()["schema"] == "model_pack/v1"

    def test_manifest_engine(self) -> None:
        """Engine must be ``drake``."""
        assert manifest()["engine"] == "drake"

    def test_manifest_engine_version(self) -> None:
        """Engine version pin must require Drake 1.20 or newer."""
        assert manifest()["engine_version"] == ">=1.20"

    def test_manifest_format(self) -> None:
        """Format must be ``sdformat-1.8``."""
        assert manifest()["format"] == "sdformat-1.8"

    def test_manifest_axis_convention(self) -> None:
        """Axis convention must be ``z_up``."""
        assert manifest()["axis_convention"] == "z_up"

    def test_manifest_anthropometrics(self) -> None:
        """Anthropometrics dataset must be ``winter_2009``."""
        assert manifest()["anthropometrics"] == "winter_2009"

    def test_manifest_package(self) -> None:
        """Package name must be ``drake_models``."""
        assert manifest()["package"] == "drake_models"


class TestResolve:
    """Tests for :func:`drake_models.model_pack.resolve`."""

    def test_resolve_returns_path(self) -> None:
        """``resolve()`` returns a :class:`Path`."""
        assert isinstance(resolve(), Path)

    def test_resolve_directory_exists(self) -> None:
        """The resolved models root exists on disk."""
        assert resolve().is_dir()

    @pytest.mark.parametrize("exercise_id", EXPECTED_EXERCISES)
    def test_resolve_contains_each_exercise_dir(self, exercise_id: str) -> None:
        """Each declared exercise has a sub-directory under the models root."""
        assert (resolve() / exercise_id).is_dir()


class TestListExercises:
    """Tests for :func:`drake_models.model_pack.list_exercises`."""

    def test_list_exercises_returns_list_of_str(self) -> None:
        """``list_exercises()`` returns a list of strings."""
        ids = list_exercises()
        assert isinstance(ids, list)
        assert all(isinstance(name, str) for name in ids)

    def test_list_exercises_count(self) -> None:
        """Exactly seven exercises are declared."""
        assert len(list_exercises()) == len(EXPECTED_EXERCISES)

    def test_list_exercises_membership(self) -> None:
        """The expected exercise ids are present."""
        assert set(list_exercises()) == set(EXPECTED_EXERCISES)


class TestEntryPoint:
    """Tests verifying the ``biomech.model_pack`` entry-point registration."""

    def test_entry_point_registered(self) -> None:
        """``drake_models`` is discoverable in the ``biomech.model_pack`` group."""
        from importlib.metadata import entry_points

        eps = entry_points(group="biomech.model_pack")
        names = {ep.name for ep in eps}
        if "drake_models" not in names:
            pytest.skip(
                "package not installed in editable mode; entry point unavailable",
            )
        ep = next(ep for ep in eps if ep.name == "drake_models")
        loaded = ep.load()
        assert hasattr(loaded, "resolve")
        assert hasattr(loaded, "manifest")
        assert hasattr(loaded, "list_exercises")


class TestLauncherCli:
    """Tests for the ``drake-models`` launcher CLI contract."""

    @pytest.mark.parametrize("exercise_id", EXPECTED_EXERCISES)
    def test_export_writes_sdf_file_and_exits_zero(
        self,
        exercise_id: str,
        tmp_path: Path,
    ) -> None:
        """``--exercise X --export PATH`` exits 0 and writes SDF 1.8 (Z-up)."""
        out = tmp_path / f"{exercise_id}.sdf"
        result = subprocess.run(  # noqa: S603 - controlled args
            [
                sys.executable,
                "-m",
                "drake_models",
                "--exercise",
                exercise_id,
                "--export",
                str(out),
            ],
            check=False,
            capture_output=True,
            text=True,
            env=_subprocess_env(),
        )
        assert result.returncode == 0, result.stderr
        assert out.is_file()
        text = out.read_text(encoding="utf-8")
        assert "<sdf" in text
        assert 'version="1.8"' in text

    def test_list_exercises_cli(self) -> None:
        """``--list-exercises`` exits 0 and prints every exercise id."""
        result = subprocess.run(  # noqa: S603 - controlled args
            [sys.executable, "-m", "drake_models", "--list-exercises"],
            check=False,
            capture_output=True,
            text=True,
            env=_subprocess_env(),
        )
        assert result.returncode == 0, result.stderr
        printed = set(result.stdout.split())
        assert set(EXPECTED_EXERCISES).issubset(printed)


@pytest.mark.live_simulation
@pytest.mark.requires_drake
class TestPydrakeParse:
    """Verify each exported SDF parses through ``pydrake.multibody.parsing``.

    Marked ``live_simulation`` (deselected by the default pytest lane); only
    runs when Drake is installed via the ``[drake]`` extra.
    """

    @pytest.mark.parametrize("exercise_id", EXPECTED_EXERCISES)
    def test_sdf_parses_via_pydrake(
        self,
        exercise_id: str,
        tmp_path: Path,
    ) -> None:
        """Each exported SDF file loads via :class:`pydrake...Parser`."""
        from pydrake.multibody.parsing import Parser  # type: ignore[import]
        from pydrake.multibody.plant import MultibodyPlant  # type: ignore[import]

        out = tmp_path / f"{exercise_id}.sdf"
        result = subprocess.run(  # noqa: S603 - controlled args
            [
                sys.executable,
                "-m",
                "drake_models",
                "--exercise",
                exercise_id,
                "--export",
                str(out),
            ],
            check=False,
            capture_output=True,
            text=True,
            env=_subprocess_env(),
        )
        assert result.returncode == 0, result.stderr

        plant = MultibodyPlant(time_step=0.0)
        parser = Parser(plant)
        _add_models(parser, out)
        plant.Finalize()
        assert plant.num_bodies() > 1


def _add_models(parser: Any, sdf_file: Path) -> None:
    """Load *sdf_file* across Drake parser API versions."""
    if hasattr(parser, "AddModels"):
        parser.AddModels(str(sdf_file))
        return
    if hasattr(parser, "AddModelFromFile"):
        parser.AddModelFromFile(str(sdf_file))
        return
    parser.AddModelsFromUrl(sdf_file.as_uri())
