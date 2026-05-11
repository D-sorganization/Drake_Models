"""CLI entry point for Drake model generation.

Two usage modes are supported:

1. Legacy positional:
   ``python3 -m drake_models <exercise> [--mass MASS] [--height HEIGHT] [--plates PLATES]``

2. UpstreamDrift launcher contract:
   ``python3 -m drake_models --exercise <name> --export <path>``
   ``python3 -m drake_models --list-exercises``

Exercises: squat, deadlift, bench_press, snatch, clean_and_jerk, gait, sit_to_stand
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

EXERCISES = {
    "squat": "drake_models.exercises.squat.squat_model",
    "deadlift": "drake_models.exercises.deadlift.deadlift_model",
    "bench_press": "drake_models.exercises.bench_press.bench_press_model",
    "snatch": "drake_models.exercises.snatch.snatch_model",
    "clean_and_jerk": "drake_models.exercises.clean_and_jerk.clean_and_jerk_model",
    "gait": "drake_models.exercises.gait.gait_model",
    "sit_to_stand": "drake_models.exercises.sit_to_stand.sit_to_stand_model",
}

BUILDER_FUNCTIONS = {
    "squat": "build_squat_model",
    "deadlift": "build_deadlift_model",
    "bench_press": "build_bench_press_model",
    "snatch": "build_snatch_model",
    "clean_and_jerk": "build_clean_and_jerk_model",
    "gait": "build_gait_model",
    "sit_to_stand": "build_sit_to_stand_model",
}


def _add_exercise_argument(parser: argparse.ArgumentParser) -> None:
    """Add the positional ``exercise`` argument to *parser*."""
    parser.add_argument(
        "exercise",
        nargs="?",
        default=None,
        choices=sorted(EXERCISES.keys()),
        help="Exercise to generate a model for (positional form).",
    )


def _add_launcher_arguments(parser: argparse.ArgumentParser) -> None:
    """Add UpstreamDrift launcher-style flag options to *parser*."""
    parser.add_argument(
        "--exercise",
        dest="exercise_flag",
        choices=sorted(EXERCISES.keys()),
        default=None,
        help="Exercise to generate a model for (flag form).",
    )
    parser.add_argument(
        "--export",
        dest="export",
        type=str,
        default=None,
        help="Write the generated SDF to this path.",
    )
    parser.add_argument(
        "--list-exercises",
        dest="list_exercises",
        action="store_true",
        help="List declared exercise ids and exit.",
    )


def _add_anthropometric_arguments(parser: argparse.ArgumentParser) -> None:
    """Add body-mass, height, and plate-mass options to *parser*."""
    parser.add_argument(
        "--mass",
        type=float,
        default=80.0,
        help="Body mass in kg (default: 80.0).",
    )
    parser.add_argument(
        "--height",
        type=float,
        default=1.75,
        help="Body height in meters (default: 1.75).",
    )
    parser.add_argument(
        "--plates",
        type=float,
        default=60.0,
        help="Plate mass per side in kg (default: 60.0).",
    )


def _add_output_arguments(parser: argparse.ArgumentParser) -> None:
    """Add output-file and verbosity flags to *parser*."""
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Output file path (default: stdout).",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose logging.",
    )


def _build_arg_parser() -> argparse.ArgumentParser:
    """Construct and return the CLI argument parser."""
    parser = argparse.ArgumentParser(
        prog="drake-models",
        description="Generate Drake SDF models for barbell exercises.",
    )
    _add_exercise_argument(parser)
    _add_launcher_arguments(parser)
    _add_anthropometric_arguments(parser)
    _add_output_arguments(parser)
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    """Validate numeric CLI arguments; call parser.error on failure (DbC)."""
    if args.mass <= 0:
        parser.error(f"--mass must be positive, got {args.mass}")
    if args.height <= 0:
        parser.error(f"--height must be positive, got {args.height}")
    if args.plates < 0:
        parser.error(f"--plates must be non-negative, got {args.plates}")


def _write_output(xml_str: str, output_path: str | None) -> None:
    """Write *xml_str* to *output_path* or stdout when path is ``None``."""
    if output_path:
        Path(output_path).write_text(xml_str, encoding="utf-8")
        logger.info("Wrote model to %s", output_path)
    else:
        stdout = sys.stdout
        stdout.write(xml_str)
        stdout.write("\n")


def _resolve_exercise(parser: argparse.ArgumentParser, args: argparse.Namespace) -> str:
    """Resolve which exercise to build from positional and flag forms.

    The flag form (``--exercise``) takes precedence; this lets the launcher
    contract co-exist with the legacy positional CLI. ``parser.error`` is
    called when neither form supplies an exercise.
    """
    chosen = args.exercise_flag or args.exercise
    if chosen is None:
        parser.error("an exercise is required (positional or via --exercise)")
    return chosen


def _handle_list_exercises() -> int:
    """Print the declared exercise ids one per line and return exit code 0."""
    from drake_models.model_pack import list_exercises

    stdout = sys.stdout
    for name in list_exercises():
        stdout.write(name)
        stdout.write("\n")
    return 0


def main(argv: list[str] | None = None) -> int:
    """Generate a Drake SDF model for a barbell exercise.

    Returns:
        Process exit code. ``0`` on success; nonzero on argument errors
        (raised via :class:`SystemExit` from ``argparse``).
    """
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    if args.list_exercises:
        return _handle_list_exercises()

    _validate_args(parser, args)
    exercise = _resolve_exercise(parser, args)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(name)s %(levelname)s: %(message)s",
    )

    import importlib

    module = importlib.import_module(EXERCISES[exercise])
    build_fn = getattr(module, BUILDER_FUNCTIONS[exercise])

    xml_str: str = build_fn(
        body_mass=args.mass,
        height=args.height,
        plate_mass_per_side=args.plates,
    )
    output_path = args.export or args.output
    _write_output(xml_str, output_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
