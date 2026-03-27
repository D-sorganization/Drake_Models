"""Cross-repo parity standard — canonical biomechanical parameters."""

from __future__ import annotations

import math


def _rad(deg: float) -> float:
    """Convert degrees to radians (used internally by JOINT_LIMITS)."""
    return math.radians(deg)


STANDARD_BODY_MASS = 80.0
STANDARD_HEIGHT = 1.75

SEGMENT_MASS_FRACTIONS = {
    "pelvis": 0.142,
    "torso": 0.355,
    "head": 0.081,
    "upper_arm": 0.028,
    "forearm": 0.016,
    "hand": 0.006,
    "thigh": 0.100,
    "shank": 0.047,
    "foot": 0.014,
}
SEGMENT_LENGTH_FRACTIONS = {
    "pelvis": 0.100,
    "torso": 0.288,
    "head": 0.130,
    "upper_arm": 0.186,
    "forearm": 0.146,
    "hand": 0.050,
    "thigh": 0.245,
    "shank": 0.246,
    "foot": 0.040,
}
JOINT_LIMITS = {
    "hip_flex": (_rad(-30), _rad(120)),
    "hip_adduct": (_rad(-45), _rad(30)),
    "hip_rotate": (_rad(-45), _rad(45)),
    "knee_flex": (_rad(-150), _rad(0)),
    "ankle_flex": (_rad(-20), _rad(50)),
    "ankle_invert": (_rad(-20), _rad(20)),
    "shoulder_flex": (_rad(-60), _rad(180)),
    "shoulder_adduct": (_rad(-30), _rad(180)),
    "shoulder_rotate": (_rad(-90), _rad(90)),
    "elbow_flex": (_rad(0), _rad(150)),
    "wrist_flex": (_rad(-70), _rad(70)),
    "wrist_deviate": (_rad(-20), _rad(30)),
    "lumbar_flex": (_rad(-30), _rad(45)),
    "lumbar_lateral": (_rad(-30), _rad(30)),
    "lumbar_rotate": (_rad(-30), _rad(30)),
    "neck_flex": (_rad(-30), _rad(30)),
}
MENS_BARBELL = {
    "total_length": 2.20,
    "shaft_length": 1.31,
    "shaft_diameter": 0.028,
    "sleeve_diameter": 0.050,
    "bar_mass": 20.0,
}
FOOT_CONTACT_DIMS = {"length": 0.26, "width": 0.10, "height": 0.02}
GROUND_FRICTION = {"static": 0.8, "dynamic": 0.6}
EXERCISE_PHASE_COUNTS = {
    "back_squat": 3,
    "deadlift": 3,
    "bench_press": 3,
    "snatch": 5,
    "clean_and_jerk": 5,
    "gait": 8,
    "sit_to_stand": 6,
}
GRAVITY = (0.0, 0.0, -9.80665)
