"""Root conftest for pytest.

Shared fixtures available to all tests in the suite.

Fleet testing standards §5: thread-safety env vars are set BEFORE any
heavy import to keep xdist workers stable and tests headless-safe.
See Repository_Management/docs/FLEET_TESTING_STANDARDS.md.
"""

from __future__ import annotations

import os

# C-extension thread safety. Many "xdist worker crashed" failures
# come from MKL/OpenBLAS forking under xdist. Pin to single-threaded
# for tests; production code can re-thread itself if it needs to.
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

# matplotlib headless backend, set before any matplotlib import.
os.environ.setdefault("MPLBACKEND", "Agg")

# Qt headless backend, for repos that import PyQt/PySide indirectly.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import xml.etree.ElementTree as ET  # noqa: E402

import pytest  # noqa: E402


@pytest.fixture()
def model() -> ET.Element:
    """Return a bare <model> XML element for use in unit tests.

    DRY: previously copy-pasted into TestCreateBarbellLinks and
    TestCreateFullBody; now defined once here.
    """
    return ET.Element("model", name="test")
