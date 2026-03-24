"""Root conftest for pytest.

Shared fixtures available to all tests in the suite.
"""

import xml.etree.ElementTree as ET

import pytest


@pytest.fixture()
def model() -> ET.Element:
    """Return a bare <model> XML element for use in unit tests.

    DRY: previously copy-pasted into TestCreateBarbellLinks and
    TestCreateFullBody; now defined once here.
    """
    return ET.Element("model", name="test")
