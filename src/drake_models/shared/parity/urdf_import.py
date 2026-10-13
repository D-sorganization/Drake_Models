"""Load a Pinocchio_Models URDF in real Drake and compare it with the SDF model.

Cross-engine check for issue #374. The URDF that Pinocchio_Models generates is
parsed with Drake's ``Parser``, the plant is finalized, and a small fingerprint
is read from the engine: total mass, canonical segment masses, the human
centre of mass expressed in the pelvis frame at the all-zero pose, and the
revolute joint count. The same measurement runs on the native Drake SDF model,
and :func:`compare` reports every difference beyond the canonical standard's
tolerances as a :class:`conformance.Divergence`, so the vendored ledger
(``load_ledger`` / ``reconcile``) can track known deviations.

The URDF is not a runtime dependency: it is resolved from ``$PINOCCHIO_URDF_DIR``
or from a caller-supplied directory (the committed, SHA-pinned test fixture),
and :func:`resolve_urdf` returns ``None`` when neither has it so callers skip.
pydrake is imported lazily, so this module imports without Drake installed.
"""

from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from drake_models.shared.parity._canonical import conformance
from drake_models.shared.parity.axes_probe import zero_positions

URDF_DIR_ENV = "PINOCCHIO_URDF_DIR"
_EXERCISE_ID = re.compile(r"^[a-z][a-z0-9_]*$")
_ROOT = "pelvis"
_KEY = "urdf"

Vec3 = tuple[float, float, float]


@dataclass(frozen=True)
class UrdfFingerprint:
    """What a finalized Drake plant reports for one human model.

    Attributes:
        total_mass_kg: Sum of every body mass in the model instance (human,
            barbell and fixtures).
        segment_masses_kg: Mass of each canonical segment found in the plant.
        human_com_in_pelvis_m: Centre of mass of the canonical segments,
            expressed in the pelvis frame at the all-zero pose.
        joint_count: Number of revolute joints in the model instance.
    """

    total_mass_kg: float
    segment_masses_kg: dict[str, float]
    human_com_in_pelvis_m: Vec3
    joint_count: int

    def __post_init__(self) -> None:
        """Precondition: positive finite masses, finite COM, non-negative count."""
        if not (math.isfinite(self.total_mass_kg) and self.total_mass_kg > 0):
            raise ValueError(f"total_mass_kg must be > 0, got {self.total_mass_kg}")
        if self.joint_count < 0:
            raise ValueError(f"joint_count must be >= 0, got {self.joint_count}")
        if len(self.human_com_in_pelvis_m) != 3 or not all(
            math.isfinite(v) for v in self.human_com_in_pelvis_m
        ):
            raise ValueError("human_com_in_pelvis_m must be 3 finite values")


def resolve_urdf(exercise: str, fixture_dir: Path) -> str | None:
    """Return the Pinocchio_Models URDF text for *exercise*, or ``None``.

    ``$PINOCCHIO_URDF_DIR/<exercise>.urdf`` wins over
    ``fixture_dir/<exercise>.urdf``.

    Raises:
        ValueError: If *exercise* is not a lower-case identifier (it is used
            as a file name, so path separators are refused).
    """
    if not _EXERCISE_ID.match(exercise):
        raise ValueError(f"exercise must be a lower-case identifier: {exercise!r}")
    env_dir = os.environ.get(URDF_DIR_ENV)
    for directory in (Path(env_dir) if env_dir else None, fixture_dir):
        if directory is not None and (directory / f"{exercise}.urdf").is_file():
            return (directory / f"{exercise}.urdf").read_text(encoding="utf-8")
    return None


def _revolute_count(plant: Any, model_instance: Any) -> int:
    return sum(
        plant.get_joint(i).type_name() == "revolute"
        for i in plant.GetJointIndices(model_instance)
    )


def _human_com_in_pelvis(
    plant: Any, context: Any, bodies: dict[str, Any], model_instance: Any
) -> Vec3:
    """Mass-weighted COM of *bodies*, expressed in the pelvis frame."""
    weighted = [0.0, 0.0, 0.0]
    mass = 0.0
    for body in bodies.values():
        pose = plant.EvalBodyPoseInWorld(context, body)
        com_w = pose.multiply(body.CalcCenterOfMassInBodyFrame(context))
        m = float(body.get_mass(context))
        weighted = [w + m * float(c) for w, c in zip(weighted, com_w, strict=True)]
        mass += m
    pelvis = plant.GetBodyByName(_ROOT, model_instance)
    com_p = (
        plant.EvalBodyPoseInWorld(context, pelvis)
        .inverse()
        .multiply([w / mass for w in weighted])
    )
    return (float(com_p[0]), float(com_p[1]), float(com_p[2]))


def measure(plant: Any, model_instance: Any, std: dict[str, Any]) -> UrdfFingerprint:
    """Read the cross-engine fingerprint from a finalized Drake plant.

    Every position is zero and every floating base sits at the identity.

    Raises:
        ValueError: If the plant is not finalized or has no pelvis body.
    """
    if not plant.is_finalized():
        raise ValueError("plant must be finalized")
    if not plant.HasBodyNamed(_ROOT, model_instance):
        raise ValueError(f"model instance has no {_ROOT!r} body")
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, zero_positions(plant))
    bodies = [plant.get_body(i) for i in plant.GetBodyIndices(model_instance)]
    canonical = set(conformance.expected_segments(std))
    segments = {b.name(): b for b in bodies if b.name() in canonical}
    return UrdfFingerprint(
        total_mass_kg=sum(float(b.get_mass(context)) for b in bodies),
        segment_masses_kg={n: float(b.get_mass(context)) for n, b in segments.items()},
        human_com_in_pelvis_m=_human_com_in_pelvis(
            plant, context, segments, model_instance
        ),
        joint_count=_revolute_count(plant, model_instance),
    )


def fingerprint_urdf(urdf: str, std: dict[str, Any]) -> UrdfFingerprint:
    """Parse *urdf* with Drake's ``Parser``, finalize, and :func:`measure` it.

    Raises:
        ValueError: If *urdf* is empty or does not define exactly one model.
    """
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant

    if not urdf.strip():
        raise ValueError("urdf must be non-empty")
    plant = MultibodyPlant(time_step=0.0)
    instances = Parser(plant).AddModelsFromString(urdf, "urdf")
    if len(instances) != 1:
        raise ValueError(f"urdf must define one model, got {len(instances)}")
    plant.Finalize()
    return measure(plant, instances[0], std)


def _mass_divergence(
    key: str, got: float, want: float, rel: float
) -> list[conformance.Divergence]:
    if math.isclose(got, want, rel_tol=rel, abs_tol=1e-9):
        return []
    return [conformance.Divergence(key, want, got, f"{key} {got:.6g} != {want:.6g}")]


def _segment_divergences(
    urdf: UrdfFingerprint, native: UrdfFingerprint, rel: float
) -> list[conformance.Divergence]:
    out: list[conformance.Divergence] = []
    for name in sorted(set(native.segment_masses_kg) | set(urdf.segment_masses_kg)):
        got = urdf.segment_masses_kg.get(name)
        want = native.segment_masses_kg.get(name)
        if got is None or want is None:
            side = "missing" if got is None else "unexpected"
            key = f"{_KEY}.segment.{name}.{side}"
            out.append(conformance.Divergence(key, want, got, f"segment {name} {side}"))
            continue
        out += _mass_divergence(f"{_KEY}.segment.{name}.mass_kg", got, want, rel)
    return out


def compare(
    urdf: UrdfFingerprint, native: UrdfFingerprint, std: dict[str, Any]
) -> list[conformance.Divergence]:
    """Return every difference between the URDF and native fingerprints.

    Masses use the standard's ``mass_rel`` tolerance, the COM uses
    ``cross_engine_position_abs_m`` per axis, and the joint count is exact.
    Keys are prefixed ``urdf.`` for the divergence ledger.
    """
    tol = std["tolerances"]
    rel = float(tol["mass_rel"])
    out = _mass_divergence(
        f"{_KEY}.total_mass_kg", urdf.total_mass_kg, native.total_mass_kg, rel
    )
    out += _segment_divergences(urdf, native, rel)
    pos = float(tol["cross_engine_position_abs_m"])
    deltas = [
        abs(a - b)
        for a, b in zip(
            urdf.human_com_in_pelvis_m, native.human_com_in_pelvis_m, strict=True
        )
    ]
    if max(deltas) > pos:
        out.append(
            conformance.Divergence(
                f"{_KEY}.human_com_in_pelvis_m",
                list(native.human_com_in_pelvis_m),
                list(urdf.human_com_in_pelvis_m),
                f"human COM differs by up to {max(deltas):.4f} m (> {pos} m)",
            )
        )
    if urdf.joint_count != native.joint_count:
        out.append(
            conformance.Divergence(
                f"{_KEY}.joint_count",
                native.joint_count,
                urdf.joint_count,
                "revolute joint count",
            )
        )
    return out
