"""Record an exercise rollout as a headless Meshcat animation (#375).

:func:`record_rollout` adds a ``MeshcatVisualizer`` to an already-configured
diagram builder, simulates it for a short duration with recording enabled
(``StartRecording`` / ``PublishRecording``) and saves the result as a
self-contained Meshcat static-HTML page.  Opening that file in any browser
plays the animation back offline; nothing is opened here.  The Meshcat server
binds to localhost only and shuts down once its last reference is dropped.

The artifact is a Meshcat *animation*, not a video file: there is no offscreen
rasterisation, so the ``video_export`` capability is rated ``partial``.

:func:`animation_frame_count` reads the frame count back out of a saved page.
Meshcat embeds each command as a base64 msgpack payload in a ``fetch("data:...")``
call; the ``set_animation`` payload holds one key per frame for every track.

pydrake is imported lazily so the rest of the package works without Drake.
"""

from __future__ import annotations

import base64
import importlib
import logging
import math
import re
import struct
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

DEFAULT_FRAMES_PER_SECOND = 32.0
_LOCALHOST = "localhost"
_DATA_URL = re.compile(r"data:application/octet-binary;base64,([A-Za-z0-9+/=]+)")
_SET_ANIMATION = b"set_animation"


@dataclass(frozen=True)
class RolloutRecording:
    """The static-HTML artifact written by :func:`record_rollout`.

    Attributes:
        path: Where the self-contained Meshcat HTML page was written.
        frame_count: Animation frames embedded in the page.
        frames_per_second: Recording (and playback) frame rate.
        duration: Simulated time covered by the recording (s).
    """

    path: Path
    frame_count: int
    frames_per_second: float
    duration: float


def _require_positive_finite(name: str, value: float) -> None:
    if not (math.isfinite(value) and value > 0.0):
        raise ValueError(f"{name} must be a positive finite number, got {value!r}")


def _localhost_meshcat(geometry: Any) -> Any:
    """A Meshcat server bound to localhost only (never opens a browser)."""
    params = geometry.MeshcatParams()
    params.host = _LOCALHOST
    return geometry.Meshcat(params)


def _simulate_recorded(
    builder: Any,
    visualizer: Any,
    duration: float,
    initialize: Callable[[Any, Any], None] | None,
) -> None:
    """Build *builder*, run it for *duration* and publish the recording."""
    analysis = importlib.import_module("pydrake.systems.analysis")
    simulator = analysis.Simulator(builder.Build())
    if initialize is not None:
        initialize(simulator.get_system(), simulator.get_mutable_context())
    visualizer.StartRecording(set_transforms_while_recording=False)
    simulator.AdvanceTo(duration)
    visualizer.StopRecording()
    visualizer.PublishRecording()


def record_rollout(
    builder: Any,
    scene_graph: Any,
    output_path: Path,
    *,
    duration: float,
    frames_per_second: float = DEFAULT_FRAMES_PER_SECOND,
    meshcat: Any | None = None,
    initialize: Callable[[Any, Any], None] | None = None,
) -> RolloutRecording:
    """Simulate *builder* headlessly and save the rollout as Meshcat HTML.

    The builder is consumed (``Build()`` is called on it), so pass one that is
    fully wired: a passive plant from :func:`drake_models.loader.load_sdf`, or
    a plant with a controller connected.  The visualizer publishes once per
    frame, so a rollout of ``duration`` seconds yields
    ``floor(duration * frames_per_second) + 1`` frames.

    Args:
        builder: An unbuilt ``DiagramBuilder`` containing *scene_graph*.
        scene_graph: The ``SceneGraph`` whose geometry is visualized.
        output_path: Destination ``.html`` file; its directory must exist.
        duration: Simulated time to record (s).
        frames_per_second: Recording frame rate.
        meshcat: An existing ``Meshcat`` to record into.  When ``None`` a
            localhost-only server is created and released with the diagram.
        initialize: Optional ``(diagram, context)`` hook run before the
            rollout, e.g. to set initial state or fix input ports.

    Returns:
        The written artifact and its frame count.

    Raises:
        ValueError: If *duration* or *frames_per_second* is not a positive
            finite number, or the output directory does not exist.
        AssertionError: If the written page is empty or holds no frames.
    """
    _require_positive_finite("duration", duration)
    _require_positive_finite("frames_per_second", frames_per_second)
    output_path = Path(output_path)
    if not output_path.parent.is_dir():
        raise ValueError(f"output directory does not exist: {output_path.parent}")
    geometry = importlib.import_module("pydrake.geometry")
    meshcat = _localhost_meshcat(geometry) if meshcat is None else meshcat
    params = geometry.MeshcatVisualizerParams()
    params.publish_period = 1.0 / frames_per_second
    visualizer = geometry.MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat, params
    )
    _simulate_recorded(builder, visualizer, duration, initialize)
    html = meshcat.StaticHtml()
    output_path.write_text(html, encoding="utf-8")
    frame_count = animation_frame_count(html)
    if output_path.stat().st_size == 0 or frame_count < 1:
        raise AssertionError(f"empty rollout recording written to {output_path}")
    logger.info("Recorded %d frames to %s", frame_count, output_path)
    return RolloutRecording(output_path, frame_count, frames_per_second, duration)


class _MsgpackReader:
    """Minimal msgpack decoder for the payloads Meshcat embeds in static HTML."""

    _FIXED = {0xC0: None, 0xC2: False, 0xC3: True}
    # tag: (struct format, kind) for sized scalars and length-prefixed values.
    _SCALARS = {
        0xCA: ">f",
        0xCB: ">d",
        0xCC: ">B",
        0xCD: ">H",
        0xCE: ">I",
        0xCF: ">Q",
        0xD0: ">b",
        0xD1: ">h",
        0xD2: ">i",
        0xD3: ">q",
    }
    _SIZED = {
        0xC4: (">B", "bin"),
        0xC5: (">H", "bin"),
        0xC6: (">I", "bin"),
        0xD9: (">B", "str"),
        0xDA: (">H", "str"),
        0xDB: (">I", "str"),
        0xDC: (">H", "array"),
        0xDD: (">I", "array"),
        0xDE: (">H", "map"),
        0xDF: (">I", "map"),
    }
    _EXT_SIZE = {0xC7: ">B", 0xC8: ">H", 0xC9: ">I"}
    _FIXEXT = {0xD4: 1, 0xD5: 2, 0xD6: 4, 0xD7: 8, 0xD8: 16}

    def __init__(self, data: bytes) -> None:
        self._data = data
        self._pos = 0

    def _take(self, count: int) -> bytes:
        chunk = self._data[self._pos : self._pos + count]
        if len(chunk) != count:
            raise ValueError("truncated msgpack payload")
        self._pos += count
        return chunk

    def _unpack(self, fmt: str) -> Any:
        return struct.unpack(fmt, self._take(struct.calcsize(fmt)))[0]

    def read(self) -> Any:
        """Decode and return the next value."""
        tag = self._take(1)[0]
        if tag <= 0x7F or tag >= 0xE0:
            return tag if tag <= 0x7F else tag - 0x100
        if tag <= 0x8F:
            return self._compound("map", tag & 0x0F)
        if tag <= 0x9F:
            return self._compound("array", tag & 0x0F)
        if tag <= 0xBF:
            return self._take(tag & 0x1F).decode("utf-8")
        return self._read_tagged(tag)

    def _read_tagged(self, tag: int) -> Any:
        if tag in self._FIXED:
            return self._FIXED[tag]
        if tag in self._SCALARS:
            return self._unpack(self._SCALARS[tag])
        if tag in self._SIZED:
            fmt, kind = self._SIZED[tag]
            return self._compound(kind, self._unpack(fmt))
        if tag in self._EXT_SIZE:
            size = self._unpack(self._EXT_SIZE[tag])
            return self._take(1 + size)[1:]
        if tag in self._FIXEXT:
            return self._take(1 + self._FIXEXT[tag])[1:]
        raise ValueError(f"unsupported msgpack tag 0x{tag:02x}")

    def _compound(self, kind: str, size: int) -> Any:
        if kind == "bin":
            return self._take(size)
        if kind == "str":
            return self._take(size).decode("utf-8")
        if kind == "array":
            return [self.read() for _ in range(size)]
        return {self.read(): self.read() for _ in range(size)}


def _animation_frames(command: dict[str, Any]) -> set[Any]:
    """Distinct key-frame indices across every track of a set_animation."""
    return {
        key["time"]
        for animation in command.get("animations", [])
        for track in animation["clip"]["tracks"]
        for key in track["keys"]
    }


def animation_frame_count(html: str) -> int:
    """Return the number of animation frames embedded in Meshcat static HTML.

    Args:
        html: A page from ``Meshcat.StaticHtml()`` (e.g. a
            :func:`record_rollout` artifact).

    Raises:
        ValueError: If *html* is empty, contains no ``set_animation``
            command, or holds a malformed payload.
    """
    if not html.strip():
        raise ValueError("html must be non-empty")
    frames: set[Any] = set()
    found = False
    for payload in _DATA_URL.findall(html):
        raw = base64.b64decode(payload)
        if _SET_ANIMATION not in raw:
            continue
        command = _MsgpackReader(raw).read()
        if isinstance(command, dict) and command.get("type") == "set_animation":
            found = True
            frames |= _animation_frames(command)
    if not found:
        raise ValueError("html contains no Meshcat animation")
    return len(frames)


__all__ = [
    "DEFAULT_FRAMES_PER_SECOND",
    "RolloutRecording",
    "animation_frame_count",
    "record_rollout",
]
