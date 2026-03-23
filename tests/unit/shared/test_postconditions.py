"""Tests for Design-by-Contract postcondition checks."""

import pytest

from drake_models.shared.contracts.postconditions import (
    ensure_positive_definite_inertia,
    ensure_positive_mass,
    ensure_valid_xml,
)


class TestEnsureValidXml:
    def test_parses_valid_xml(self):
        root = ensure_valid_xml("<root><child/></root>")
        assert root.tag == "root"

    def test_returns_root_element(self):
        root = ensure_valid_xml('<sdf version="1.8"><model name="test"/></sdf>')
        assert root.tag == "sdf"

    def test_rejects_malformed_xml(self):
        with pytest.raises(ValueError, match="not well-formed"):
            ensure_valid_xml("<root><unclosed>")

    def test_rejects_empty_string(self):
        with pytest.raises(ValueError, match="not well-formed"):
            ensure_valid_xml("")

    def test_accepts_xml_with_attributes(self):
        root = ensure_valid_xml('<link name="pelvis"/>')
        assert root.get("name") == "pelvis"


class TestEnsurePositiveMass:
    def test_accepts_positive(self):
        ensure_positive_mass(10.0, "torso")

    def test_rejects_zero(self):
        with pytest.raises(AssertionError, match="not positive"):
            ensure_positive_mass(0.0, "torso")

    def test_rejects_negative(self):
        with pytest.raises(AssertionError, match="not positive"):
            ensure_positive_mass(-1.0, "pelvis")

    def test_includes_body_name(self):
        with pytest.raises(AssertionError, match="my_link"):
            ensure_positive_mass(0.0, "my_link")


class TestEnsurePositiveDefiniteInertia:
    def test_accepts_valid_inertia(self):
        ensure_positive_definite_inertia(1.0, 1.0, 1.0, "box")

    def test_accepts_cylinder_inertia(self):
        ensure_positive_definite_inertia(0.5, 0.5, 0.3, "cyl")

    def test_rejects_zero_ixx(self):
        with pytest.raises(AssertionError, match="Ixx"):
            ensure_positive_definite_inertia(0.0, 1.0, 1.0, "bad")

    def test_rejects_zero_iyy(self):
        with pytest.raises(AssertionError, match="Iyy"):
            ensure_positive_definite_inertia(1.0, 0.0, 1.0, "bad")

    def test_rejects_zero_izz(self):
        with pytest.raises(AssertionError, match="Izz"):
            ensure_positive_definite_inertia(1.0, 1.0, 0.0, "bad")

    def test_rejects_negative_inertia(self):
        with pytest.raises(AssertionError):
            ensure_positive_definite_inertia(-1.0, 1.0, 1.0, "bad")

    def test_rejects_triangle_inequality_violation(self):
        with pytest.raises(AssertionError, match="triangle inequality"):
            ensure_positive_definite_inertia(0.1, 0.1, 10.0, "bad")

    def test_includes_body_name(self):
        with pytest.raises(AssertionError, match="shaft"):
            ensure_positive_definite_inertia(0.0, 1.0, 1.0, "shaft")
