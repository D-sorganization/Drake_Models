"""Unit tests for standing pelvis height derivation and grounding (#364)."""

from __future__ import annotations

import itertools
import xml.etree.ElementTree as ET

import pytest

from drake_models.exercises.bench_press.bench_press_model import (
    BENCH_HEIGHT,
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
from drake_models.shared.body.body_anthropometrics import (
    FOOT_CONTACT_HEIGHT,
    FOOT_CONTACT_LENGTH,
    FOOT_CONTACT_WIDTH,
    PELVIS_STANDING_HEIGHT,
    BodyModelSpec,
    _seg,
    standing_pelvis_height,
)
from drake_models.shared.utils.sdf_poses import posed_link_transforms

FREE_ROOT_BUILDERS = (
    ("squat", build_squat_model),
    ("deadlift", build_deadlift_model),
    ("snatch", build_snatch_model),
    ("clean_and_jerk", build_clean_and_jerk_model),
    ("gait", build_gait_model),
    ("sit_to_stand", build_sit_to_stand_model),
)


class TestStandingPelvisHeight:
    """Tests for standing_pelvis_height calculation and constants."""

    def test_default_standing_pelvis_height(self) -> None:
        spec = BodyModelSpec()
        expected = (
            _seg(spec, "pelvis")[1] / 2.0
            + _seg(spec, "thigh")[1]
            + _seg(spec, "shank")[1]
            + _seg(spec, "foot")[1]
            + FOOT_CONTACT_HEIGHT
        )
        assert standing_pelvis_height() == pytest.approx(expected)
        assert standing_pelvis_height() == pytest.approx(1.03675)
        assert pytest.approx(expected) == PELVIS_STANDING_HEIGHT

    def test_standing_pelvis_height_scales_with_spec_height(self) -> None:
        spec_tall = BodyModelSpec(height=2.0)
        h_tall = standing_pelvis_height(spec_tall)
        assert h_tall > standing_pelvis_height()

        expected_tall = (
            _seg(spec_tall, "pelvis")[1] / 2.0
            + _seg(spec_tall, "thigh")[1]
            + _seg(spec_tall, "shank")[1]
            + _seg(spec_tall, "foot")[1]
            + FOOT_CONTACT_HEIGHT
        )
        assert h_tall == pytest.approx(expected_tall)

    @pytest.mark.parametrize("name,build", FREE_ROOT_BUILDERS)
    def test_free_root_sdf_seeds_pelvis_at_standing_height(
        self, name: str, build: object
    ) -> None:
        """Every free-root SDF seeds <link name='pelvis'><pose> at standing height."""
        sdf = str(build())  # type: ignore[operator]
        model = ET.fromstring(sdf).find("model")
        assert model is not None
        pelvis = next(
            (link for link in model.findall("link") if link.get("name") == "pelvis"),
            None,
        )
        assert pelvis is not None, f"model {name} has no pelvis link"
        pose_text = pelvis.findtext("pose")
        assert pose_text is not None, f"pelvis in {name} has no <pose>"
        pose_vals = [float(v) for v in pose_text.split()]
        assert pose_vals[2] == pytest.approx(standing_pelvis_height(), abs=1e-6)

    def test_bench_press_seeds_pelvis_at_bench_height(self) -> None:
        """Bench press pelvis is placed at BENCH_HEIGHT (0.43 m)."""
        sdf = build_bench_press_model()
        model = ET.fromstring(sdf).find("model")
        assert model is not None
        pelvis = next(
            (link for link in model.findall("link") if link.get("name") == "pelvis"),
            None,
        )
        assert pelvis is not None
        pose_text = pelvis.findtext("pose")
        assert pose_text is not None
        pose_vals = [float(v) for v in pose_text.split()]
        assert pose_vals[2] == pytest.approx(BENCH_HEIGHT, abs=1e-6)

    @pytest.mark.parametrize("name,build", FREE_ROOT_BUILDERS)
    def test_neutral_pose_lowest_foot_point_is_zero(
        self, name: str, build: object
    ) -> None:
        """Forward kinematics at neutral joint angles places foot sole bottom on z=0."""
        sdf = str(build())  # type: ignore[operator]
        model = ET.fromstring(sdf).find("model")
        assert model is not None

        h = standing_pelvis_height()
        transforms = posed_link_transforms(
            model, {}, root_poses={"pelvis": (0.0, 0.0, h, 0.0, 0.0, 0.0)}
        )

        spec = BodyModelSpec()
        _ft_mass, ft_len, _ft_rad = _seg(spec, "foot")

        lowest_points: list[float] = []
        for side in ("l", "r"):
            foot_xf = transforms[f"foot_{side}"]
            for corner in itertools.product((-0.5, 0.5), repeat=3):
                pt = [
                    corner[0] * FOOT_CONTACT_LENGTH,
                    corner[1] * FOOT_CONTACT_WIDTH,
                    -ft_len
                    - FOOT_CONTACT_HEIGHT / 2.0
                    + corner[2] * FOOT_CONTACT_HEIGHT,
                    1.0,
                ]
                lowest_points.append(float((foot_xf @ pt)[2]))

        assert min(lowest_points) == pytest.approx(0.0, abs=1e-6)
