"""Headless Meshcat recording of a short exercise rollout on real pydrake (#375).

A passive rollout of the gait model is recorded with ``MeshcatVisualizer``
(``StartRecording``/``PublishRecording``) and saved as Meshcat static HTML.
Meshcat binds to localhost only, no browser is opened, and the server is shut
down when the last reference is dropped (checked by rebinding its port).
"""

from __future__ import annotations

import gc
import math
import socket
from pathlib import Path
from typing import Any

import pytest

pytest.importorskip("pydrake")

from pydrake.geometry import Meshcat, MeshcatParams  # noqa: E402

from drake_models.loader import load_sdf  # noqa: E402
from drake_models.recording import (  # noqa: E402
    RolloutRecording,
    animation_frame_count,
    record_rollout,
)
from drake_models.shared.parity.fingerprint import _build_sdf  # noqa: E402

pytestmark = pytest.mark.requires_drake

DURATION_S = 0.25
FPS = 32.0
# One frame per publish, at t = 0, 1/fps, ..., duration (inclusive).
EXPECTED_FRAMES = math.floor(DURATION_S * FPS) + 1


def _gait_loaded() -> Any:
    return load_sdf(_build_sdf("gait"), "gait", time_step=1e-3)


def _record_with_own_meshcat(output: Path) -> tuple[RolloutRecording, int]:
    """Record into a caller-owned localhost Meshcat; return the result and port.

    Every pydrake reference (Meshcat, plant, diagram) is local to this
    function, so the server must shut down once it returns.
    """
    params = MeshcatParams()
    params.host = "localhost"
    meshcat = Meshcat(params)
    loaded = _gait_loaded()
    result = record_rollout(
        loaded.builder,
        loaded.scene_graph,
        output,
        duration=DURATION_S,
        frames_per_second=FPS,
        meshcat=meshcat,
    )
    return result, meshcat.port()


class TestRecordRollout:
    def test_records_expected_frames_to_static_html(self, tmp_path: Path) -> None:
        output = tmp_path / "gait_rollout.html"
        result, port = _record_with_own_meshcat(output)
        assert result.path == output
        assert output.stat().st_size > 0
        html = output.read_text(encoding="utf-8")
        assert animation_frame_count(html) == EXPECTED_FRAMES
        assert result.frame_count == EXPECTED_FRAMES
        assert result.duration == pytest.approx(DURATION_S)
        gc.collect()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.bind(("127.0.0.1", port))  # raises if the server still listens

    def test_owns_a_localhost_meshcat_when_none_given(self, tmp_path: Path) -> None:
        loaded = _gait_loaded()
        result = record_rollout(
            loaded.builder,
            loaded.scene_graph,
            tmp_path / "gait.html",
            duration=DURATION_S,
            frames_per_second=FPS,
        )
        assert result.frame_count == EXPECTED_FRAMES
        assert result.path.stat().st_size > 0
