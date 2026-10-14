"""Tests for segment inertia matching the cross-repo parity standard (#391).

Verifies that segment radius is computed from the standard's ``radius_frac``
(not from uniform tissue density) and that resulting segment inertias match
the shared standard within 1e-6 relative. Also verifies bench pad mass is
near-zero so it does not distort lifter mass in LIFT-2.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from drake_models.exercises.bench_press.bench_press_model import (
    BENCH_PAD_MASS,
    build_bench_press_model,
)
from drake_models.exercises.clean_and_jerk.clean_and_jerk_model import (
    build_clean_and_jerk_model,
)
from drake_models.exercises.deadlift.deadlift_model import build_deadlift_model
from drake_models.exercises.gait.gait_model import build_gait_model
from drake_models.exercises.sit_to_stand.sit_to_stand_model import (
    build_sit_to_stand_model,
)
from drake_models.exercises.snatch.snatch_model import build_snatch_model
from drake_models.exercises.squat.squat_model import build_squat_model
from drake_models.shared.body import BodyModelSpec, create_full_body
from drake_models.shared.body.body_anthropometrics import _seg
from drake_models.shared.parity._canonical import conformance
from drake_models.shared.parity.standard import (
    SEGMENT_LENGTH_FRACTIONS,
    SEGMENT_MASS_FRACTIONS,
    SEGMENT_RADIUS_FRACTIONS,
    SEGMENT_TABLE,
)
from drake_models.shared.utils.geometry import (
    cylinder_inertia,
    rectangular_prism_inertia,
)

STD = conformance.load_standard()
EXPECTED_SEGMENTS = conformance.expected_segments(STD)

EXERCISE_BUILDERS: dict[str, Any] = {
    "squat": build_squat_model,
    "bench_press": build_bench_press_model,
    "deadlift": build_deadlift_model,
    "snatch": build_snatch_model,
    "clean_and_jerk": build_clean_and_jerk_model,
    "gait": build_gait_model,
    "sit_to_stand": build_sit_to_stand_model,
}


def _compute_standard_inertia(
    segment_name: str,
    total_mass: float = 80.0,
    height: float = 1.75,
) -> tuple[float, float, float]:
    """Compute shared-standard principal inertia (ixx, iyy, izz) from radius_frac."""
    base_name = segment_name.removesuffix("_l").removesuffix("_r")
    seg_info = STD["anthropometrics"]["segments"][base_name]

    mass = total_mass * float(seg_info["mass_frac"])
    length = height * float(seg_info["length_frac"])
    radius = height * float(seg_info["radius_frac"])

    if base_name in ("pelvis", "torso"):
        return rectangular_prism_inertia(mass, radius * 2, length, radius * 2)
    return cylinder_inertia(mass, radius, length)


@pytest.mark.parametrize("segment_name", sorted(EXPECTED_SEGMENTS.keys()))
def test_generated_body_inertia_matches_standard(segment_name: str) -> None:
    """Inertia from create_full_body matches the shared standard within 1e-6."""
    model = ET.Element("model", name="test_body")
    spec = BodyModelSpec(total_mass=80.0, height=1.75)
    create_full_body(model, spec=spec)

    link = model.find(f".//link[@name='{segment_name}']")
    assert link is not None, f"{segment_name} missing from created body model"

    inertial = link.find("inertial/inertia")
    assert inertial is not None, f"no inertia block on {segment_name}"

    ixx = float(inertial.findtext("ixx", "0.0"))
    iyy = float(inertial.findtext("iyy", "0.0"))
    izz = float(inertial.findtext("izz", "0.0"))
    expected = _compute_standard_inertia(segment_name, spec.total_mass, spec.height)

    assert ixx == pytest.approx(expected[0], rel=1e-6, abs=1e-6)
    assert iyy == pytest.approx(expected[1], rel=1e-6, abs=1e-6)
    assert izz == pytest.approx(expected[2], rel=1e-6, abs=1e-6)


@pytest.mark.parametrize("segment_name", sorted(EXPECTED_SEGMENTS.keys()))
def test_exercise_model_inertia_matches_standard(segment_name: str) -> None:
    """Inertia from generated squat exercise model matches shared standard."""
    sdf_str = build_squat_model(80.0, 1.75, 20.0)
    root = ET.fromstring(sdf_str)

    link = root.find(f".//link[@name='{segment_name}']")
    assert link is not None, f"{segment_name} missing from squat exercise model"

    inertial = link.find("inertial/inertia")
    assert inertial is not None, f"no inertia block on {segment_name}"

    ixx = float(inertial.findtext("ixx", "0.0"))
    iyy = float(inertial.findtext("iyy", "0.0"))
    izz = float(inertial.findtext("izz", "0.0"))
    expected = _compute_standard_inertia(segment_name, 80.0, 1.75)

    assert ixx == pytest.approx(expected[0], rel=1e-6, abs=1e-6)
    assert iyy == pytest.approx(expected[1], rel=1e-6, abs=1e-6)
    assert izz == pytest.approx(expected[2], rel=1e-6, abs=1e-6)


def test_segment_radius_computed_from_radius_frac() -> None:
    """Segment radius is computed from radius_frac for all segments."""
    spec = BodyModelSpec(total_mass=80.0, height=1.75)
    for name in SEGMENT_TABLE:
        mass, length, radius = _seg(spec, name)
        assert mass == pytest.approx(spec.total_mass * SEGMENT_MASS_FRACTIONS[name])
        assert length == pytest.approx(spec.height * SEGMENT_LENGTH_FRACTIONS[name])
        assert radius == pytest.approx(spec.height * SEGMENT_RADIUS_FRACTIONS[name])


def test_bench_press_pad_mass_near_zero() -> None:
    """Bench pad mass is near-zero (<= 1e-3 kg) so it does not add lifter mass."""
    assert BENCH_PAD_MASS <= 1e-3
    sdf_str = build_bench_press_model(80.0, 1.75, 20.0)
    root = ET.fromstring(sdf_str)
    bench_pad = root.find(".//link[@name='bench_pad']")
    assert bench_pad is not None, "bench_pad link missing from bench press model"
    mass_elem = bench_pad.find("inertial/mass")
    assert mass_elem is not None
    assert float(mass_elem.text or "0.0") <= 1e-3


def test_real_engine_segment_inertia_matches_standard() -> None:
    """Real Drake engine parses SDF and reports matching spatial inertia."""
    pytest.importorskip("pydrake")
    from drake_models.loader import load_sdf

    spec = BodyModelSpec(total_mass=80.0, height=1.75)
    sdf_str = build_squat_model(spec.total_mass, spec.height, 20.0)
    loaded = load_sdf(sdf_str, "squat", apply_pose=False)
    plant = loaded.plant

    for segment_name in sorted(EXPECTED_SEGMENTS.keys()):
        body = plant.GetBodyByName(segment_name)
        assert body is not None, f"Body {segment_name} not found in Drake plant"
        expected = _compute_standard_inertia(segment_name, spec.total_mass, spec.height)

        spatial_inertia = body.default_spatial_inertia()
        # In Drake, spatial inertia is about the body origin Bo.
        # Shifting to center of mass Bcm yields the rotational inertia about CoM.
        p_BoBcm_B = spatial_inertia.get_com()
        rot_inertia_com = spatial_inertia.Shift(p_BoBcm_B).CalcRotationalInertia()
        moments = rot_inertia_com.get_moments()

        assert moments[0] == pytest.approx(expected[0], rel=1e-6, abs=1e-6)
        assert moments[1] == pytest.approx(expected[1], rel=1e-6, abs=1e-6)
        assert moments[2] == pytest.approx(expected[2], rel=1e-6, abs=1e-6)
