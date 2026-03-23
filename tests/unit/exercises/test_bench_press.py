"""Tests for bench press model builder."""

import xml.etree.ElementTree as ET

import pytest

from drake_models.exercises.bench_press.bench_press_model import (
    BENCH_HEIGHT,
    BenchPressModelBuilder,
    build_bench_press_model,
)


class TestBenchPressModelBuilder:
    def test_exercise_name(self):
        builder = BenchPressModelBuilder()
        assert builder.exercise_name == "bench_press"

    def test_build_returns_valid_sdf(self):
        xml_str = build_bench_press_model()
        root = ET.fromstring(xml_str)
        assert root.tag == "sdf"

    def test_model_name(self):
        xml_str = build_bench_press_model()
        root = ET.fromstring(xml_str)
        model = root.find("model")
        assert model.get("name") == "bench_press"

    def test_barbell_attached_to_hand(self):
        xml_str = build_bench_press_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":
                assert j.find("parent").text == "hand_l"
                assert j.find("child").text == "barbell_shaft"

    def test_attachment_is_fixed(self):
        xml_str = build_bench_press_model()
        root = ET.fromstring(xml_str)
        for j in root.findall(".//joint"):
            if j.get("name") == "barbell_to_left_hand":
                assert j.get("type") == "fixed"

    def test_bench_height_constant(self):
        assert pytest.approx(0.43) == BENCH_HEIGHT

    def test_has_gravity(self):
        xml_str = build_bench_press_model()
        root = ET.fromstring(xml_str)
        gravity = root.find(".//gravity")
        assert gravity is not None
        assert "-9.806650" in gravity.text

    def test_default_config(self):
        builder = BenchPressModelBuilder()
        assert builder.config.body_spec.total_mass == 80.0

    def test_custom_plate_mass(self):
        xml_str = build_bench_press_model(plate_mass_per_side=70.0)
        root = ET.fromstring(xml_str)
        assert root.find(".//model") is not None
