"""Guard tests for Drake-specific API usage patterns.

Two sharp edges in ``pydrake`` (documented in the UpstreamDrift CLAUDE.md)
must be respected by any code in this repository that touches Drake:

1. **Explicit imports only.** Attribute access on the ``pydrake`` namespace
   does not work -- callers must use ``from pydrake.X import Y``.
2. **``body.body_frame()`` direct access.** Wrapping a body in a
   ``FixedOffsetFrame`` to retrieve its frame is incorrect; the body's own
   ``body_frame()`` accessor is the supported API.

These tests encode each rule as a guard so a regression that re-introduces
the bad pattern fails *here* rather than later in UpstreamDrift.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

SRC_ROOT = Path(__file__).resolve().parent.parent / "src" / "drake_models"

# Match ``from pydrake import ...`` (and ``import pydrake`` without submodule).
# Allowed:   ``from pydrake.X import Y`` / ``from pydrake.X.Y import Z``.
# Disallowed: ``from pydrake import X`` / ``import pydrake`` as bare module.
_BAD_IMPORT_PATTERNS = (
    re.compile(r"^\s*from\s+pydrake\s+import\s+", re.MULTILINE),
    re.compile(r"^\s*import\s+pydrake\s*$", re.MULTILINE),
    re.compile(r"^\s*import\s+pydrake\s+as\s+", re.MULTILINE),
)

# Match construction of ``FixedOffsetFrame(... body_frame ...)`` -- the
# disallowed pattern when used to get a body's frame. Any reference to
# ``FixedOffsetFrame`` in source under ``src/drake_models`` is flagged.
_BAD_FRAME_PATTERN = re.compile(r"\bFixedOffsetFrame\s*\(")


def _iter_python_sources() -> list[Path]:
    """Return every ``.py`` file under ``src/drake_models``."""
    return sorted(SRC_ROOT.rglob("*.py"))


def _read(path: Path) -> str:
    """Read source file text (utf-8); empty string on read failure."""
    try:
        return path.read_text(encoding="utf-8")
    except OSError:
        return ""


class TestExplicitPydrakeImports:
    """Source must never import the bare ``pydrake`` namespace."""

    @pytest.mark.parametrize(
        "source_path", _iter_python_sources(), ids=lambda p: p.name
    )
    def test_no_bare_pydrake_import(self, source_path: Path) -> None:
        """Each source file uses explicit ``from pydrake.X import Y`` imports."""
        text = _read(source_path)
        for pattern in _BAD_IMPORT_PATTERNS:
            matches = pattern.findall(text)
            assert not matches, (
                f"{source_path.relative_to(SRC_ROOT)}: matched disallowed import "
                f"pattern {pattern.pattern!r}. Use explicit "
                f"'from pydrake.<submodule> import <Name>' instead."
            )


class TestBodyFrameDirectAccess:
    """Source must use ``body.body_frame()`` rather than ``FixedOffsetFrame``."""

    @pytest.mark.parametrize(
        "source_path", _iter_python_sources(), ids=lambda p: p.name
    )
    def test_no_fixed_offset_frame_construction(self, source_path: Path) -> None:
        """``FixedOffsetFrame(...)`` must not appear in ``src/drake_models``."""
        text = _read(source_path)
        matches = _BAD_FRAME_PATTERN.findall(text)
        assert not matches, (
            f"{source_path.relative_to(SRC_ROOT)}: constructs FixedOffsetFrame. "
            "Use ``body.body_frame()`` directly instead."
        )
