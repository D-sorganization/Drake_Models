"""Generate all five exercise SDF models and write them to disk.

Usage:
    python3 examples/generate_all_models.py [output_dir]

Default output directory is ./output/
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

from drake_models.exercises.bench_press.bench_press_model import (
    build_bench_press_model,
)
from drake_models.exercises.clean_and_jerk.clean_and_jerk_model import (
    build_clean_and_jerk_model,
)
from drake_models.exercises.deadlift.deadlift_model import build_deadlift_model
from drake_models.exercises.snatch.snatch_model import build_snatch_model
from drake_models.exercises.squat.squat_model import build_squat_model

logger = logging.getLogger(__name__)

EXERCISES = {
    "back_squat": build_squat_model,
    "bench_press": build_bench_press_model,
    "deadlift": build_deadlift_model,
    "snatch": build_snatch_model,
    "clean_and_jerk": build_clean_and_jerk_model,
}


def generate_all(output_dir: Path) -> None:
    """Generate all exercise models and write SDF files to output_dir."""
    output_dir.mkdir(parents=True, exist_ok=True)

    for name, builder in EXERCISES.items():
        xml_str = builder()
        output_path = output_dir / f"{name}.sdf"
        output_path.write_text(xml_str, encoding="utf-8")
        print(f"Generated {output_path}")

    print(f"\nAll {len(EXERCISES)} models written to {output_dir}/")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("output")
    generate_all(out)
