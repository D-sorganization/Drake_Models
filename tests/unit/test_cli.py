"""Tests for the CLI entry point."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from io import StringIO
from unittest.mock import patch

import pytest

from drake_models.__main__ import (
    _build_arg_parser,
    _validate_args,
    _write_output,
    main,
)


class TestBuildArgParser:
    """Tests for the extracted _build_arg_parser helper."""

    def test_returns_argument_parser(self) -> None:
        parser = _build_arg_parser()
        assert isinstance(parser, argparse.ArgumentParser)

    def test_has_exercise_argument(self) -> None:
        parser = _build_arg_parser()
        args = parser.parse_args(["squat"])
        assert args.exercise == "squat"

    def test_has_mass_default(self) -> None:
        parser = _build_arg_parser()
        args = parser.parse_args(["squat"])
        assert args.mass == 80.0

    def test_has_height_default(self) -> None:
        parser = _build_arg_parser()
        args = parser.parse_args(["squat"])
        assert args.height == 1.75

    def test_has_plates_default(self) -> None:
        parser = _build_arg_parser()
        args = parser.parse_args(["squat"])
        assert args.plates == 60.0

    def test_verbose_flag(self) -> None:
        parser = _build_arg_parser()
        args = parser.parse_args(["squat", "-v"])
        assert args.verbose is True

    def test_output_flag(self) -> None:
        parser = _build_arg_parser()
        args = parser.parse_args(["squat", "-o", "out.sdf"])
        assert args.output == "out.sdf"


class TestValidateArgs:
    """Tests for the extracted _validate_args helper."""

    def _parse(self, argv: list[str]) -> argparse.Namespace:
        return _build_arg_parser().parse_args(argv)

    def test_valid_args_pass(self) -> None:
        parser = _build_arg_parser()
        args = self._parse(["squat", "--mass", "80", "--height", "1.75"])
        _validate_args(parser, args)  # should not raise

    def test_zero_mass_raises_system_exit(self) -> None:
        parser = _build_arg_parser()
        args = self._parse(["squat"])
        args.mass = 0.0
        with pytest.raises(SystemExit):
            _validate_args(parser, args)

    def test_negative_mass_raises_system_exit(self) -> None:
        parser = _build_arg_parser()
        args = self._parse(["squat"])
        args.mass = -1.0
        with pytest.raises(SystemExit):
            _validate_args(parser, args)

    def test_zero_height_raises_system_exit(self) -> None:
        parser = _build_arg_parser()
        args = self._parse(["squat"])
        args.height = 0.0
        with pytest.raises(SystemExit):
            _validate_args(parser, args)

    def test_negative_plates_raises_system_exit(self) -> None:
        parser = _build_arg_parser()
        args = self._parse(["squat"])
        args.plates = -1.0
        with pytest.raises(SystemExit):
            _validate_args(parser, args)

    def test_zero_plates_is_valid(self) -> None:
        parser = _build_arg_parser()
        args = self._parse(["squat"])
        args.plates = 0.0
        _validate_args(parser, args)  # should not raise


class TestWriteOutput:
    """Tests for the extracted _write_output helper."""

    def test_writes_to_stdout_when_no_path(self) -> None:
        with patch("sys.stdout", new_callable=StringIO) as mock_out:
            _write_output("<sdf/>", None)
        assert "<sdf/>" in mock_out.getvalue()

    def test_writes_to_file(self, tmp_path: object) -> None:
        import pathlib

        assert isinstance(tmp_path, pathlib.Path)
        out_file = tmp_path / "out.sdf"
        _write_output("<sdf/>", str(out_file))
        assert out_file.read_text() == "<sdf/>"


class TestCLI:
    def test_squat_to_stdout(self) -> None:
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            main(["squat"])
        output = mock_stdout.getvalue()
        root = ET.fromstring(output.strip())
        assert root.tag == "sdf"

    def test_deadlift_to_stdout(self) -> None:
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            main(["deadlift"])
        output = mock_stdout.getvalue()
        assert "<?xml" in output

    def test_custom_mass_and_height(self) -> None:
        with patch("sys.stdout", new_callable=StringIO) as mock_stdout:
            main(["bench_press", "--mass", "100", "--height", "1.90"])
        output = mock_stdout.getvalue()
        root = ET.fromstring(output.strip())
        model = root.find(".//model")
        assert model is not None
        assert model.get("name") == "bench_press"

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
