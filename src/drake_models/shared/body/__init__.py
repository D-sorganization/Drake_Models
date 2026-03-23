"""Full-body multibody model for Drake (SDF).

Provides a simplified but anatomically grounded full-body model with
major body segments and joints suitable for barbell exercise simulation.
"""

from drake_models.shared.body.body_model import (
    BodyModelSpec,
    create_full_body,
)

__all__ = ["BodyModelSpec", "create_full_body"]
