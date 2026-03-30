"""Tests for ground contact models and hydroelastic contact properties."""

import xml.etree.ElementTree as ET
from typing import Any

import pytest

from drake_models.shared.body import create_full_body
from drake_models.shared.utils.sdf_helpers import (
    DRAKE_NS,
    add_collision_filter_group,
    add_contact_geometry,
    add_ground_plane_contact,
    add_link,
    make_box_geometry,
)

# Namespace map for XPath queries on Drake-namespaced elements.
_NS = {"drake": DRAKE_NS}


def _dtag(local: str) -> str:
    """Build a fully-qualified tag in the Drake namespace (for tag comparison)."""
    return f"{{{DRAKE_NS}}}{local}"


class TestAddContactGeometry:
    @pytest.fixture()
    def link(self) -> ET.Element:
        model = ET.Element("model", name="test")
        return add_link(
            model,
            name="test_link",
            mass=1.0,
            mass_center=(0, 0, 0),
            inertia_xx=0.1,
            inertia_yy=0.1,
            inertia_zz=0.1,
        )

    def test_creates_collision_element(self, link: Any) -> None:
        geom = make_box_geometry(0.26, 0.10, 0.02)
        collision = add_contact_geometry(link, name="foot_contact", geometry=geom)
        assert collision.tag == "collision"
        assert collision.get("name") == "foot_contact"

    def test_has_geometry(self, link: Any) -> None:
        geom = make_box_geometry(0.26, 0.10, 0.02)
        collision = add_contact_geometry(link, name="foot_contact", geometry=geom)
        box = collision.find("geometry/box")
        assert box is not None

    def test_has_drake_proximity_properties(self, link: Any) -> None:
        geom = make_box_geometry(0.26, 0.10, 0.02)
        collision = add_contact_geometry(link, name="foot_contact", geometry=geom)
        prox = collision.find("drake:proximity_properties", _NS)
        assert prox is not None

    def test_has_compliant_hydroelastic(self, link: Any) -> None:
        geom = make_box_geometry(0.26, 0.10, 0.02)
        collision = add_contact_geometry(link, name="foot_contact", geometry=geom)
        prox = collision.find("drake:proximity_properties", _NS)
        assert prox.find("drake:compliant_hydroelastic", _NS) is not None

    def test_has_hydroelastic_modulus(self, link: Any) -> None:
        geom = make_box_geometry(0.26, 0.10, 0.02)
        collision = add_contact_geometry(
            link,
            name="foot_contact",
            geometry=geom,
            hydroelastic_modulus=1e7,
        )
        prox = collision.find("drake:proximity_properties", _NS)
        modulus = prox.find("drake:hydroelastic_modulus", _NS)
        assert modulus is not None
        assert modulus is not None and modulus.text is not None
        assert float(modulus.text) == pytest.approx(1e7)

    def test_has_friction_coefficients(self, link: Any) -> None:
        geom = make_box_geometry(0.26, 0.10, 0.02)
        collision = add_contact_geometry(
            link,
            name="foot_contact",
            geometry=geom,
            mu_static=0.8,
            mu_dynamic=0.6,
        )
        prox = collision.find("drake:proximity_properties", _NS)
        assert float(prox.find("drake:mu_static", _NS).text if prox.find("drake:mu_static", _NS) is not None and prox.find("drake:mu_static", _NS).text else 0.0) == pytest.approx(0.8)
        assert float(prox.find("drake:mu_dynamic", _NS).text if prox.find("drake:mu_dynamic", _NS) is not None and prox.find("drake:mu_dynamic", _NS).text else 0.0) == pytest.approx(0.6)

    def test_has_dissipation(self, link: Any) -> None:
        geom = make_box_geometry(0.26, 0.10, 0.02)
        collision = add_contact_geometry(
            link,
            name="foot_contact",
            geometry=geom,
            hunt_crossley_dissipation=1.0,
        )
        prox = collision.find("drake:proximity_properties", _NS)
        diss = prox.find("drake:hunt_crossley_dissipation", _NS)
        assert diss is not None
        assert diss is not None and diss.text is not None
        assert float(diss.text) == pytest.approx(1.0)

    def test_has_pose(self, link: Any) -> None:
        geom = make_box_geometry(0.26, 0.10, 0.02)
        collision = add_contact_geometry(
            link,
            name="foot_contact",
            geometry=geom,
            pose=(0, 0, -0.05, 0, 0, 0),
        )
        pose = collision.find("pose")
        assert pose is not None
        assert pose is not None and pose.text is not None
        assert "-0.050000" in pose.text

    def test_custom_friction(self, link: Any) -> None:
        geom = make_box_geometry(0.26, 0.10, 0.02)
        collision = add_contact_geometry(
            link,
            name="fc",
            geometry=geom,
            mu_static=1.2,
            mu_dynamic=0.9,
        )
        prox = collision.find("drake:proximity_properties", _NS)
        assert float(prox.find("drake:mu_static", _NS).text if prox.find("drake:mu_static", _NS) is not None and prox.find("drake:mu_static", _NS).text else 0.0) == pytest.approx(1.2)
        assert float(prox.find("drake:mu_dynamic", _NS).text if prox.find("drake:mu_dynamic", _NS) is not None and prox.find("drake:mu_dynamic", _NS).text else 0.0) == pytest.approx(0.9)


class TestAddGroundPlaneContact:
    @pytest.fixture()
    def model(self) -> ET.Element:
        return ET.Element("model", name="test")

    def test_creates_ground_plane_link(self, model: Any) -> None:
        ground = add_ground_plane_contact(model)
        assert ground.tag == "link"
        assert ground.get("name") == "ground_plane"

    def test_has_collision(self, model: Any) -> None:
        ground = add_ground_plane_contact(model)
        collision = ground.find("collision")
        assert collision is not None
        assert collision.get("name") == "ground_plane_collision"

    def test_has_rigid_hydroelastic(self, model: Any) -> None:
        ground = add_ground_plane_contact(model)
        collision = ground.find("collision")
        prox = collision.find("drake:proximity_properties", _NS)
        assert prox is not None
        assert prox.find("drake:rigid_hydroelastic", _NS) is not None

    def test_has_friction(self, model: Any) -> None:
        ground = add_ground_plane_contact(model)
        collision = ground.find("collision")
        prox = collision.find("drake:proximity_properties", _NS)
        assert float(prox.find("drake:mu_static", _NS).text if prox.find("drake:mu_static", _NS) is not None and prox.find("drake:mu_static", _NS).text else 0.0) == pytest.approx(0.8)
        assert float(prox.find("drake:mu_dynamic", _NS).text if prox.find("drake:mu_dynamic", _NS) is not None and prox.find("drake:mu_dynamic", _NS).text else 0.0) == pytest.approx(0.6)

    def test_welded_to_world(self, model: Any) -> None:
        add_ground_plane_contact(model)
        joints = model.findall("joint")
        weld = None
        for j in joints:
            if j.get("name") == "ground_plane_weld":
                weld = j
                break
        assert weld is not None
        assert weld.get("type") == "fixed"
        assert weld.find("parent").text == "world"
        assert weld.find("child").text == "ground_plane"

    def test_ground_box_geometry(self, model: Any) -> None:
        ground = add_ground_plane_contact(model)
        collision = ground.find("collision")
        box = collision.find("geometry/box")
        assert box is not None
        size_el = box.find("size")
        assert size_el is not None and size_el.text is not None
        size = size_el.text
        assert "100.000000" in size


class TestFootContactGeometry:
    @pytest.fixture()
    def model(self) -> ET.Element:
        return ET.Element("model", name="test")

    def test_foot_links_have_contact_collision(self, model: Any) -> None:
        """Both foot links must have a contact collision element."""
        create_full_body(model)
        for side in ("l", "r"):
            foot = model.find(f"link[@name='foot_{side}']")
            assert foot is not None
            collisions = foot.findall("collision")
            contact_names = [c.get("name") for c in collisions]
            assert f"foot_{side}_contact" in contact_names, (
                f"foot_{side} missing contact collision"
            )

    def test_foot_contact_has_box_geometry(self, model: Any) -> None:
        create_full_body(model)
        for side in ("l", "r"):
            foot = model.find(f"link[@name='foot_{side}']")
            contact = None
            for c in foot.findall("collision"):
                if c.get("name") == f"foot_{side}_contact":
                    contact = c
                    break
            assert contact is not None
            box = contact.find("geometry/box")
            assert box is not None
            size_el = box.find("size")
            assert size_el is not None and size_el.text is not None
            size = size_el.text
            assert "0.260000" in size
            assert "0.100000" in size
            assert "0.020000" in size

    def test_foot_contact_has_drake_properties(self, model: Any) -> None:
        create_full_body(model)
        for side in ("l", "r"):
            foot = model.find(f"link[@name='foot_{side}']")
            contact = None
            for c in foot.findall("collision"):
                if c.get("name") == f"foot_{side}_contact":
                    contact = c
                    break
            assert contact is not None
            prox = contact.find("drake:proximity_properties", _NS)
            assert prox is not None
            assert prox.find("drake:compliant_hydroelastic", _NS) is not None
            assert prox.find("drake:hydroelastic_modulus", _NS) is not None
            assert prox.find("drake:mu_static", _NS) is not None
            assert prox.find("drake:mu_dynamic", _NS) is not None


class TestCollisionFilterGroup:
    @pytest.fixture()
    def model(self) -> ET.Element:
        return ET.Element("model", name="test")

    def test_creates_filter_group(self, model: Any) -> None:
        group = add_collision_filter_group(
            model, name="hip_l", members=["pelvis", "thigh_l"]
        )
        assert group.tag == _dtag("collision_filter_group")
        assert group.get("name") == "hip_l"

    def test_has_members(self, model: Any) -> None:
        group = add_collision_filter_group(
            model,
            name="knee_r",
            members=["thigh_r", "shank_r"],
        )
        members = [m.text for m in group.findall("drake:member", _NS) if m.text is not None ]
        assert "thigh_r" in members
        assert "shank_r" in members

    def test_ignores_self(self, model: Any) -> None:
        group = add_collision_filter_group(
            model, name="hip_l", members=["pelvis", "thigh_l"]
        )
        ignored = group.find("drake:ignored_collision_filter_group", _NS)
        assert ignored is not None
        assert ignored.text == "hip_l"
