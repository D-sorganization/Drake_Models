"""Barbell model generation for Drake (SDF).

Provides factory functions to create standard Olympic barbell links
with correct mass, geometry, and inertia properties.
"""

from drake_models.shared.barbell.barbell_model import (
    BarbellSpec,
    create_barbell_links,
)

__all__ = ["BarbellSpec", "create_barbell_links"]
