"""Tests for the DbC helper extracted from ``_attach_bilateral_grip``.

Introduced by A-N Refresh 2026-04-14 (issue #129). The helper gives
exercise builders a single, authoritative precondition check for the
links required to weld the barbell shaft to a hand.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from drake_models.exercises.base import ExerciseModelBuilder


def _hand_links() -> dict[str, ET.Element]:
    return {
        "hand_l": ET.Element("link", name="hand_l"),
        "hand_r": ET.Element("link", name="hand_r"),
    }


def _barbell_links() -> dict[str, ET.Element]:
    return {"barbell_shaft": ET.Element("link", name="barbell_shaft")}


class TestRequireBilateralGripLinks:
    def test_accepts_fully_populated_link_dicts(self) -> None:
        # No exception = precondition satisfied
        ExerciseModelBuilder._validate_grip_preconditions(
            _hand_links(), _barbell_links()
        )

    def test_rejects_missing_left_hand(self) -> None:
        body = _hand_links()
        del body["hand_l"]
        with pytest.raises(ValueError, match="hand_l"):
            ExerciseModelBuilder._validate_grip_preconditions(body, _barbell_links())

    def test_rejects_missing_right_hand(self) -> None:
        body = _hand_links()
        del body["hand_r"]
        with pytest.raises(ValueError, match="hand_r"):
            ExerciseModelBuilder._validate_grip_preconditions(body, _barbell_links())

    def test_rejects_missing_barbell_shaft(self) -> None:
        with pytest.raises(ValueError, match="barbell_shaft"):
            ExerciseModelBuilder._validate_grip_preconditions(_hand_links(), {})

    def test_reports_all_missing_links_in_one_message(self) -> None:
        """Caller debugging is easier when all missing keys show up at once."""
        with pytest.raises(ValueError) as exc_info:
            ExerciseModelBuilder._validate_grip_preconditions({}, {})
        msg = str(exc_info.value)
        for token in ("hand_l", "hand_r", "barbell_shaft"):
            assert token in msg
