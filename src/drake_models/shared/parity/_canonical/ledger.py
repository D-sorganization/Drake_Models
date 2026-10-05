# SPDX-License-Identifier: MIT
"""Divergence ledger for model-pack parity checks (vendored bundle).

CANONICAL SOURCE: Repository_Management/shared_scripts/model_parity/.
Never edit a vendored copy.

A pack's ``parity_divergences.json`` lists every known, issue-tracked deviation
from the standard. An entry may be scoped to named exercises. The ledger only
ratchets down: a deviation missing from it is unexpected, and an entry (or a
scoped exercise of one) that no longer deviates is stale. Stdlib only; works on
any divergence object with a ``key`` (``conformance.Divergence``).
"""

from __future__ import annotations

import fnmatch
import json
import re
from collections.abc import Sequence
from pathlib import Path
from typing import Any, Protocol, TypeVar

LEDGER_SCHEMA = "parity-divergences/v1"
_ISSUE_REF = re.compile(r"(#\d+|https://github\.com/\S+/issues/\d+)")


class _Keyed(Protocol):
    @property
    def key(self) -> str: ...


_D = TypeVar("_D", bound=_Keyed)


def load_ledger(path: Path | str) -> dict[str, Any]:
    """Load a ``parity_divergences.json`` ledger (missing file = empty ledger)."""
    p = Path(path)
    if not p.exists():
        return {"schema": LEDGER_SCHEMA, "divergences": {}}
    ledger: dict[str, Any] = json.loads(p.read_text(encoding="utf-8"))
    return ledger


def _is_name_list(value: Any) -> bool:
    """True for a non-empty list of non-empty strings."""
    return (
        isinstance(value, list)
        and bool(value)
        and all(isinstance(s, str) and s for s in value)
    )


def _check_entry(key: str, entry: Any) -> None:
    """Precondition for one ledger entry: it cites an issue, and any
    ``exercises`` scope is a non-empty list of exercise names."""
    issue = entry.get("issue") if isinstance(entry, dict) else None
    if not isinstance(issue, str) or not _ISSUE_REF.search(issue):
        raise ValueError(f"ledger entry {key!r} must cite an issue (#N or URL)")
    if "exercises" in entry and not _is_name_list(entry["exercises"]):
        raise ValueError(f"ledger entry {key!r}: exercises must list names")


def _validated_entries(ledger: dict[str, Any]) -> dict[str, Any]:
    """Precondition: the schema matches and every entry passes ``_check_entry``."""
    if ledger.get("schema") != LEDGER_SCHEMA:
        raise ValueError(f"ledger schema must be {LEDGER_SCHEMA!r}")
    entries: dict[str, Any] = ledger.get("divergences", {})
    for key, entry in entries.items():
        _check_entry(key, entry)
    return entries


def _match(  # noqa: UP047 - packs still support Python 3.10
    divergences: Sequence[_D], entries: dict[str, Any], exercise: str | None
) -> tuple[list[_D], set[str]]:
    """Return (unexpected divergences, ledger keys that absorbed one)."""
    scoped = {
        p
        for p, e in entries.items()
        if "exercises" not in e or (exercise is not None and exercise in e["exercises"])
    }
    used: set[str] = set()
    unexpected: list[_D] = []
    for div in divergences:
        hits = [p for p in scoped if fnmatch.fnmatchcase(div.key, p)]
        used.update(hits)
        if not hits:
            unexpected.append(div)
    return unexpected, used


def reconcile(  # noqa: UP047 - packs still support Python 3.10
    divergences: list[_D],
    ledger: dict[str, Any],
    exercise: str | None = None,
) -> tuple[list[_D], list[str]]:
    """Split divergences into (unexpected, stale_ledger_keys).

    Every ledger entry must cite an issue. Patterns may use ``*`` wildcards.
    An entry with an ``exercises`` list absorbs divergences only from those
    exercises, so it never matches when ``exercise`` is None. A ledger entry
    matching no current divergence is stale: the gap was fixed and the entry
    must be deleted (the ledger only ratchets down). To find stale entries
    across several exercises, use :func:`reconcile_all`.
    """
    entries = _validated_entries(ledger)
    unexpected, used = _match(divergences, entries, exercise)
    return unexpected, sorted(set(entries) - used)


def _stale_keys(
    entries: dict[str, Any], hits_by_exercise: dict[str, set[str]]
) -> set[str]:
    """Stale ledger keys. An unscoped entry is stale as ``key`` when nothing
    used it. A scoped entry is stale as ``key`` only when every exercise in its
    scope was checked and none used it; otherwise each checked member that no
    longer uses it is stale as ``key@exercise`` (unchecked members never are).
    """
    used = set().union(*hits_by_exercise.values())
    stale: set[str] = set()
    for key, entry in entries.items():
        if "exercises" not in entry:
            if key not in used:
                stale.add(key)
            continue
        scope = entry["exercises"]
        missed = [
            x for x in scope if x in hits_by_exercise and key not in hits_by_exercise[x]
        ]
        if len(missed) == len(scope):
            stale.add(key)
        else:
            stale.update(f"{key}@{x}" for x in missed)
    return stale


def reconcile_all(  # noqa: UP047 - packs still support Python 3.10
    by_exercise: dict[str, list[_D]], ledger: dict[str, Any]
) -> tuple[dict[str, list[_D]], list[str]]:
    """Reconcile every exercise's divergences against one ledger.

    Returns ({exercise: unexpected} for exercises with any, stale keys); see
    :func:`_stale_keys` for when a scoped entry or one of its members is stale.
    """
    entries = _validated_entries(ledger)
    unexpected: dict[str, list[_D]] = {}
    hits_by_exercise: dict[str, set[str]] = {}
    for exercise, divs in by_exercise.items():
        found, hits_by_exercise[exercise] = _match(divs, entries, exercise)
        if found:
            unexpected[exercise] = found
    return unexpected, sorted(_stale_keys(entries, hits_by_exercise))
