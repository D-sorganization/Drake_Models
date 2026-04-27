"""Tests for the sdf_helpers helpers introduced by A-N Refresh 2026-04-14.

Covers ``_write_inertial_block``, ``_append_geometry`` and
``_append_compliant_proximity_properties`` extracted from ``add_link``
and ``add_contact_geometry`` under issue #129.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

from drake_models.shared.utils.sdf_helpers import (
    _append_compliant_proximity_properties,
    _append_geometry,
    _write_inertial_block,
    add_contact_geometry,
    add_link,
    make_box_geometry,
    make_cylinder_geometry,
)


class TestWriteInertialBlock:
    def test_block_contains_mass_pose_and_full_inertia(self) -> None:
        link = ET.Element("link", name="torso")
        _write_inertial_block(
            link,
            mass=25.0,
            mass_center=(0.0, 0.0, 0.15),
            inertia_xx=0.1,
            inertia_yy=0.2,
            inertia_zz=0.3,
            inertia_xy=0.01,
            inertia_xz=0.02,
            inertia_yz=0.03,
        )
        inertial = link.find("inertial")
        assert inertial is not None
        assert inertial.find("mass").text == "25.000000"  # type: ignore[union-attr]
        inertia = inertial.find("inertia")
        assert inertia is not None
        tags = {child.tag: child.text for child in inertia}
        assert tags["ixx"] == "0.100000"
        assert tags["iyy"] == "0.200000"
        assert tags["izz"] == "0.300000"
        assert tags["ixy"] == "0.010000"
        assert tags["ixz"] == "0.020000"
        assert tags["iyz"] == "0.030000"


class TestAppendGeometry:
    def test_appends_visual_with_canonical_name(self) -> None:
        link = ET.Element("link", name="foo")
        _append_geometry(
            link,
            tag="visual",
            link_name="foo",
            geometry=make_box_geometry(0.1, 0.1, 0.1),
        )
        visual = link.find("visual")
        assert visual is not None
        assert visual.get("name") == "foo_visual"
        assert visual.find("geometry") is not None

    def test_appends_collision_with_canonical_name(self) -> None:
        link = ET.Element("link", name="bar")
        _append_geometry(
            link,
            tag="collision",
            link_name="bar",
            geometry=make_cylinder_geometry(0.05, 0.4),
        )
        coll = link.find("collision")
        assert coll is not None
        assert coll.get("name") == "bar_collision"


class TestAddLinkStillMatchesLegacyShape:
    def test_full_link_has_inertial_visual_and_collision(self) -> None:
        model = ET.Element("model", name="m")
        add_link(
            model,
            name="upper_arm_l",
            mass=2.24,
            mass_center=(0, 0, -0.15),
            inertia_xx=0.01,
            inertia_yy=0.01,
            inertia_zz=0.005,
            visual_geometry=make_cylinder_geometry(0.03, 0.3),
            collision_geometry=make_cylinder_geometry(0.03, 0.3),
        )
        link = model.find("link")
        assert link is not None
        assert link.find("inertial") is not None
        assert link.find("visual[@name='upper_arm_l_visual']") is not None
        assert link.find("collision[@name='upper_arm_l_collision']") is not None

    def test_link_without_geometry_is_inertial_only(self) -> None:
        model = ET.Element("model", name="m")
        add_link(
            model,
            name="point",
            mass=0.1,
            mass_center=(0, 0, 0),
            inertia_xx=1e-6,
            inertia_yy=1e-6,
            inertia_zz=1e-6,
        )
        link = model.find("link")
        assert link is not None
        assert link.find("inertial") is not None
        assert link.find("visual") is None
        assert link.find("collision") is None


class TestAppendCompliantProximityProperties:
    def test_writes_friction_and_hydroelastic_modulus(self) -> None:
        collision = ET.Element("collision", name="c")
        _append_compliant_proximity_properties(
            collision,
            mu_static=0.9,
            mu_dynamic=0.7,
            hydroelastic_modulus=2e7,
            hunt_crossley_dissipation=1.5,
        )
        prox = collision.find("{drake.mit.edu}proximity_properties")
        assert prox is not None
        # Compliant hydroelastic tag must be present (switches contact solver)
        assert prox.find("{drake.mit.edu}compliant_hydroelastic") is not None
        mu_s = prox.find("{drake.mit.edu}mu_static")
        assert mu_s is not None
        assert mu_s.text == "0.9"
        mu_d = prox.find("{drake.mit.edu}mu_dynamic")
        assert mu_d is not None
        assert mu_d.text == "0.7"

    def test_add_contact_geometry_still_produces_full_block(self) -> None:
        link = ET.Element("link", name="foot_l")
        add_contact_geometry(
            link,
            name="foot_l_contact",
            geometry=make_box_geometry(0.26, 0.1, 0.02),
        )
        coll = link.find("collision[@name='foot_l_contact']")
        assert coll is not None
        prox = coll.find("{drake.mit.edu}proximity_properties")
        assert prox is not None
        assert prox.find("{drake.mit.edu}compliant_hydroelastic") is not None
