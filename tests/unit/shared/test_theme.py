"""Tests for the shared plotting theme fallback."""

from __future__ import annotations

from drake_models.shared import theme


def test_theme_falls_back_to_none_without_ud_tools_plot_theme() -> None:
    assert theme.theme is None


def test_style_axis_fallback_is_noop() -> None:
    axis = object()

    assert theme.style_axis(axis) is None
