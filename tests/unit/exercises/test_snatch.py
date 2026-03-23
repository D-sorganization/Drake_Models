"""Tests for snatch model builder."""

import xml.etree.ElementTree as ET

from drake_models.exercises.snatch.snatch_model import (
    SnatchModelBuilder,
    build_snatch_model,
)


class TestSnatchModelBuilder:
    def test_exercise_name(self):
        builder = SnatchModelBuilder()
        assert builder.exercise_name == "snatch"

    def test_build_returns_valid_sdf(self):
        xml_str = build_snatch_model()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_model_name(self):
        xml_str = build_snatch_model()
        root = ET.fromstring(xml_str)
        model = root.find("model")
        assert model.get("name") == "snatch"

    def test_barbell_attached_to_hand(self):
        xml_str = build_snatch_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":
                assert j.find("parent").text == "hand_l"
                assert j.find("child").text == "barbell_shaft"

    def test_attachment_is_fixed(self):
        xml_str = build_snatch_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":
                assert j.get("type") == "fixed"

    def test_has_gravity(self):
        xml_str = build_snatch_model()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")
        assert "-9.806650" in gravity.text

    def test_default_plate_mass_40(self):
        xml_str = build_snatch_model()
        root = ET.fromstring(xml_str)
        assert root.find(".//model") is not None

    def test_custom_params(self):
        xml_str = build_snatch_model(body_mass=96, height=1.80, plate_mass_per_side=55)
        root = ET.fromstring(xml_str)
        assert root.find(".//model").get("name") == "snatch"
