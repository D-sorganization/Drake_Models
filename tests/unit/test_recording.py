"""Pure-python tests for :mod:`drake_models.recording` (#375).

These run without pydrake: they cover the preconditions of
:func:`record_rollout` and the frame counter that reads a Meshcat static-HTML
artifact.  The real headless recording is exercised in
``tests/integration/test_record_rollout.py``.
"""

from __future__ import annotations

import base64
import struct
from pathlib import Path
from typing import Any

import pytest

from drake_models.recording import animation_frame_count, record_rollout


def _pack(value: Any) -> bytes:
    """Minimal msgpack encoder for the fixture subset (fixmap/str/array/int/float)."""
    if isinstance(value, bool):
        return b"\xc3" if value else b"\xc2"
    if isinstance(value, int):
        assert 0 <= value < 128
        return bytes([value])
    if isinstance(value, float):
        return b"\xcb" + struct.pack(">d", value)
    if isinstance(value, str):
        raw = value.encode()
        assert len(raw) < 32
        return bytes([0xA0 | len(raw)]) + raw
    if isinstance(value, list):
        assert len(value) < 16
        return bytes([0x90 | len(value)]) + b"".join(_pack(v) for v in value)
    assert isinstance(value, dict) and len(value) < 16
    body = b"".join(_pack(k) + _pack(v) for k, v in value.items())
    return bytes([0x80 | len(value)]) + body


def _command_html(*commands: dict[str, Any]) -> str:
    """A static-HTML stand-in embedding *commands* the way Meshcat does."""
    fetches = "\n".join(
        'fetch("data:application/octet-binary;base64,'
        f'{base64.b64encode(_pack(c)).decode()}")'
        for c in commands
    )
    return f"<html><script>\n{fetches}\n</script></html>"


def _animation(frames: int) -> dict[str, Any]:
    keys = [{"time": i, "value": [0.0, 0.0, float(i)]} for i in range(frames)]
    track = {"name": ".position", "type": "vector3", "keys": keys}
    clip = {"fps": 32.0, "name": "default", "tracks": [track, track]}
    return {
        "type": "set_animation",
        "animations": [{"path": "/drake/a", "clip": clip}],
        "options": {"play": True},
    }


class TestAnimationFrameCount:
    def test_counts_distinct_frames_of_embedded_animation(self) -> None:
        html = _command_html({"type": "set_property", "path": "/x"}, _animation(9))
        assert animation_frame_count(html) == 9

    def test_rejects_html_without_animation(self) -> None:
        html = _command_html({"type": "set_property", "path": "/x"})
        with pytest.raises(ValueError, match="no Meshcat animation"):
            animation_frame_count(html)

    def test_rejects_empty_html(self) -> None:
        with pytest.raises(ValueError, match="non-empty"):
            animation_frame_count("")


class TestRecordRolloutPreconditions:
    @pytest.mark.parametrize("duration", [0.0, -1.0, float("nan"), float("inf")])
    def test_rejects_non_positive_or_non_finite_duration(
        self, duration: float, tmp_path: Path
    ) -> None:
        with pytest.raises(ValueError, match="duration"):
            record_rollout(object(), object(), tmp_path / "a.html", duration=duration)

    @pytest.mark.parametrize("fps", [0.0, -30.0, float("nan")])
    def test_rejects_non_positive_frame_rate(self, fps: float, tmp_path: Path) -> None:
        with pytest.raises(ValueError, match="frames_per_second"):
            record_rollout(
                object(),
                object(),
                tmp_path / "a.html",
                duration=0.1,
                frames_per_second=fps,
            )

    def test_rejects_missing_output_directory(self, tmp_path: Path) -> None:
        with pytest.raises(ValueError, match="directory"):
            record_rollout(
                object(), object(), tmp_path / "missing" / "a.html", duration=0.1
            )
