"""Design-by-Contract postcondition checks.

Used to validate outputs after computation — catches bugs in model
generation before they propagate to downstream SDF XML or simulation.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET


def ensure_valid_xml(xml_string: str) -> ET.Element:
    """Parse *xml_string* and return the root element.

    Raises ValueError if the string is not well-formed XML.
    """
    try:
        return ET.fromstring(xml_string)
    except ET.ParseError as exc:
        raise ValueError(f"Generated XML is not well-formed: {exc}") from exc


def ensure_positive_mass(mass: float, body_name: str) -> None:
    """Assert that a body's mass is positive after computation."""
    if mass <= 0:
        raise AssertionError(
            f"Postcondition violated: {body_name} mass={mass} is not positive"
        )


def ensure_positive_definite_inertia(
    ixx: float, iyy: float, izz: float, body_name: str
) -> None:
    """Assert that principal inertias are positive (necessary for PD)."""
    for label, val in [("Ixx", ixx), ("Iyy", iyy), ("Izz", izz)]:
        if val <= 0:
            raise AssertionError(
                f"Postcondition violated: {body_name} {label}={val} not positive"
            )
    # Triangle inequality for principal inertias
    if ixx + iyy < izz or ixx + izz < iyy or iyy + izz < ixx:
        raise AssertionError(
            f"Postcondition violated: {body_name} inertias "
            f"({ixx}, {iyy}, {izz}) violate triangle inequality"
        )
