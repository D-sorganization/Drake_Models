"""Real-Drake segment origins match the standard's reference FK (RM#2011)."""

from __future__ import annotations

import pytest

pytest.importorskip("pydrake")

from drake_models.model_pack import list_exercises  # noqa: E402
from drake_models.shared.parity._canonical import conformance, topology  # noqa: E402
from drake_models.shared.parity.fingerprint import fingerprint  # noqa: E402

STD = conformance.load_standard()


@pytest.mark.parametrize("exercise", list_exercises())
def test_origins_match_the_reference_at_every_test_pose(exercise: str) -> None:
    fp = fingerprint(exercise)
    assert fp["loaded_in_engine"], fp["load_error"]
    assert set(fp["segment_origins_test_poses_m"]) == set(topology.standard_poses(STD))
    assert topology.check_origins(fp, STD) == []
