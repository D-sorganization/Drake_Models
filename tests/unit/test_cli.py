"""Tests for the CLI entry point."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from io import StringIO
from unittest.mock import patch

import pytest

from drake_models.__main__ import main


class TestCLI:
    def test_squat_to_stdout(self) -> None:
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            main(["squat"])
        output = mock_stdout.getvalue()
        root = ET.fromstring(output.strip())  # type: ignore
        assert root.tag == "sdf"

    def test_deadlift_to_stdout(self) -> None:
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            main(["deadlift"])
        output = mock_stdout.getvalue()
        assert "<?xml" in output  # type: ignore

    def test_custom_mass_and_height(self) -> None:
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            main(["bench_press", "--mass", "100", "--height", "1.90"])
        output = mock_stdout.getvalue()
        root = ET.fromstring(output.strip())  # type: ignore
        assert root.find(".//model").get("name") == "bench_press"  # type: ignore

    def test_output_to_file(self, tmp_path: object) -> None:
        import pathlib

        assert isinstance(tmp_path, pathlib.Path)
        out_file = tmp_path / "test.sdf"
        main(["snatch", "-o", str(out_file)])
        content = out_file.read_text()
        root = ET.fromstring(content)
        assert root.tag == "sdf"

    def test_invalid_exercise_exits(self) -> None:
        with pytest.raises(SystemExit):
            main(["invalid_exercise"])

    def test_verbose_flag(self) -> None:
        with patch("sys.stdout", new_callable=StringIO):
            main(["clean_and_jerk", "-v"])
