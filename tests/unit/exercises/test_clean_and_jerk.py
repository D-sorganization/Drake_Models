"""Tests for clean-and-jerk model builder."""

import xml.etree.ElementTree as ET

from drake_models.exercises.clean_and_jerk.clean_and_jerk_model import (
    CleanAndJerkModelBuilder,
    build_clean_and_jerk_model,
)


class TestCleanAndJerkModelBuilder:
    def test_exercise_name(self):
        builder = CleanAndJerkModelBuilder()
        assert builder.exercise_name == "clean_and_jerk"

    def test_build_returns_valid_sdf(self):
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_model_name(self):
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        model = root.find("model")
        assert model.get("name") == "clean_and_jerk"

    def test_barbell_attached_to_hand(self):
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":
                assert j.find("parent").text == "hand_l"
                assert j.find("child").text == "barbell_shaft"

    def test_attachment_is_fixed(self):
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":
                assert j.get("type") == "fixed"

    def test_has_gravity(self):
        xml_str = build_clean_and_jerk_model()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")
        assert "-9.806650" in gravity.text

    def test_default_config(self):
        builder = CleanAndJerkModelBuilder()
        assert builder.config.body_spec.total_mass == 80.0

    def test_custom_params(self):
        xml_str = build_clean_and_jerk_model(
            body_mass=85,
            height=1.78,
            plate_mass_per_side=60,
        )
        root = ET.fromstring(xml_str)
        assert root.find(".//model").get("name") == "clean_and_jerk"
