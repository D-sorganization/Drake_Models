"""CLI entry point for Drake model generation.

Usage:
    python3 -m drake_models <exercise> [--mass MASS] [--height HEIGHT] [--plates PLATES]

Exercises: squat, deadlift, bench_press, snatch, clean_and_jerk
"""

from __future__ import annotations

import argparse
import logging
import sys

logger = logging.getLogger(__name__)

EXERCISES = {
    "squat": "drake_models.exercises.squat.squat_model",
    "deadlift": "drake_models.exercises.deadlift.deadlift_model",
    "bench_press": "drake_models.exercises.bench_press.bench_press_model",
    "snatch": "drake_models.exercises.snatch.snatch_model",
    "clean_and_jerk": "drake_models.exercises.clean_and_jerk.clean_and_jerk_model",
}

BUILDER_FUNCTIONS = {
    "squat": "build_squat_model",
    "deadlift": "build_deadlift_model",
    "bench_press": "build_bench_press_model",
    "snatch": "build_snatch_model",
    "clean_and_jerk": "build_clean_and_jerk_model",
}


def main(argv: list[str] | None = None) -> None:
    """Generate a Drake SDF model for a barbell exercise."""
    parser = argparse.ArgumentParser(
        prog="drake-models",
        description="Generate Drake SDF models for barbell exercises.",
    )
    parser.add_argument(
        "exercise",
        choices=sorted(EXERCISES.keys()),
        help="Exercise to generate a model for.",
    )
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

    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(name)s %(levelname)s: %(message)s",
    )

    import importlib

    module = importlib.import_module(EXERCISES[args.exercise])
    build_fn = getattr(module, BUILDER_FUNCTIONS[args.exercise])

    xml_str: str = build_fn(
        body_mass=args.mass,
        height=args.height,
        plate_mass_per_side=args.plates,
    )

    if args.output:
        with open(args.output, "w") as f:
            f.write(xml_str)
        logger.info("Wrote model to %s", args.output)
    else:
        sys.stdout.write(xml_str)
        sys.stdout.write("\n")


if __name__ == "__main__":
    main()
