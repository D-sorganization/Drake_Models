"""Engine-free tests for ``drake_models.loader`` SDF parsing (no pydrake needed)."""

from __future__ import annotations

import pytest

from drake_models.exercises.squat.squat_model import build_squat_model
from drake_models.loader import parse_initial_pose

_ENTITY_BOMB = """<?xml version="1.0"?>
<!DOCTYPE sdf [<!ENTITY a "aaaaaaaaaa"><!ENTITY b "&a;&a;&a;&a;&a;">]>
<sdf version="1.8"><model name="m">&b;</model></sdf>"""


def test_parse_initial_pose_reads_generated_model() -> None:
    pose = parse_initial_pose(build_squat_model())
    assert pose is not None


def test_parse_initial_pose_returns_none_without_pose_block() -> None:
    assert parse_initial_pose('<sdf version="1.8"><model name="m"/></sdf>') is None


def test_parse_initial_pose_rejects_entity_declarations() -> None:
    """Untrusted XML with entity declarations is refused, not expanded."""
    with pytest.raises(ValueError, match="(?i)entit"):
        parse_initial_pose(_ENTITY_BOMB)
