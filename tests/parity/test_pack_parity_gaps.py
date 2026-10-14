"""Drake_Models parity gaps from the UpstreamDrift lift-pack audit (#429).

Both checks are pure SDF-text tests (no pydrake):

* Every lift's ``shoulder_{l,r}_adduct`` limits equal the vendored standard's
  ``coordinates`` entry (read from the standard, never hard-coded here).
* The bench-press bench body carries the cross-pack ground-borne bench mass
  (1e-4 kg, as in MuJoCo_Models, OpenSim_Models and Pinocchio_Models).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import pytest

from drake_models.model_pack import list_exercises
from drake_models.shared.parity._canonical import conformance
from drake_models.shared.parity.fingerprint import _build_sdf

LIFTS = ("squat", "deadlift", "bench_press", "snatch", "clean_and_jerk")
SIDES = ("l", "r")
# Cross-pack convention: the bench is welded to ground, so its mass is borne by
# the ground and the body carries only a near-zero placeholder mass (kg).
CROSS_PACK_BENCH_MASS_KG = 1e-4

STD = conformance.load_standard()


def _standard_limits_rad(coordinate: str) -> tuple[float, float]:
    for entry in STD["coordinates"]:
        if entry["name"] == coordinate:
            lo, hi = entry["limits_deg"]
            return math.radians(lo), math.radians(hi)
    raise AssertionError(f"{coordinate} missing from the vendored standard")


def _joint_limits_rad(sdf: str, joint_name: str) -> tuple[float, float]:
    root = ET.fromstring(sdf)
    for joint in root.iter("joint"):
        if joint.get("name") == joint_name:
            limit = joint.find("axis/limit")
            assert limit is not None, f"{joint_name} has no limit"
            return float(limit.findtext("lower")), float(limit.findtext("upper"))
    raise AssertionError(f"joint {joint_name} not found")


def test_all_five_lifts_are_covered() -> None:
    assert set(LIFTS) <= set(list_exercises())


@pytest.mark.parametrize("side", SIDES)
@pytest.mark.parametrize("lift", LIFTS)
def test_shoulder_adduct_limits_match_standard(lift: str, side: str) -> None:
    want = _standard_limits_rad("shoulder_{side}_adduct")
    got = _joint_limits_rad(_build_sdf(lift), f"shoulder_{side}_adduct")
    assert got == pytest.approx(want, abs=1e-4)


def test_bench_pad_mass_matches_cross_pack_value() -> None:
    root = ET.fromstring(_build_sdf("bench_press"))
    pads = [link for link in root.iter("link") if link.get("name") == "bench_pad"]
    assert len(pads) == 1
    mass = float(pads[0].findtext("inertial/mass"))
    assert mass == pytest.approx(CROSS_PACK_BENCH_MASS_KG, rel=1e-6)


def test_bench_pad_inertia_stays_physically_valid() -> None:
    """A near-zero mass must not round the inertia tensor to an invalid one."""
    root = ET.fromstring(_build_sdf("bench_press"))
    pad = next(lk for lk in root.iter("link") if lk.get("name") == "bench_pad")
    i = {k: float(pad.findtext(f"inertial/inertia/{k}")) for k in ("ixx", "iyy", "izz")}
    assert all(v > 0.0 for v in i.values()), i
    assert i["ixx"] + i["iyy"] > i["izz"], i
    assert i["ixx"] + i["izz"] > i["iyy"], i
    assert i["iyy"] + i["izz"] > i["ixx"], i
