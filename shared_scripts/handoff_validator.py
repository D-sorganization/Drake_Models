#!/usr/bin/env python3
"""Canonical handoff schema validation and enforcement for the repository fleet.

Ensures implementation state survives agent replacement and context exhaustion
by validating canonical handoff schema, detecting unedited placeholders, ensuring
implementation commits update continuation state, and protecting against secrets.

Part of Repository_Management#1938: partitioned into ``handoff_schema.py`` and
``handoff_validator.py``.
Portable: standard library only, copied fleet-wide next to ``development_log.py``.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import os
import re
import sys
from collections.abc import Sequence
from pathlib import Path
from types import ModuleType
from typing import TYPE_CHECKING


def sibling(name: str) -> ModuleType:
    """Import a sibling fleet module by package, else by file path."""
    try:
        return importlib.import_module(f"shared_scripts.{name}")
    except ImportError:
        cached = sys.modules.get(f"_fleet_{name}")
        if cached is not None:
            return cached
        path = Path(__file__).with_name(f"{name}.py")
        spec = importlib.util.spec_from_file_location(f"_fleet_{name}", path)
        if spec is None or spec.loader is None:  # pragma: no cover - defensive
            raise
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module


_schema = sibling("handoff_schema")

# Re-export schema symbols
HEX_COMMIT_PATTERN = _schema.HEX_COMMIT_PATTERN
IMPLEMENTATION_PREFIXES = _schema.IMPLEMENTATION_PREFIXES
IMPLEMENTATION_SUFFIXES = _schema.IMPLEMENTATION_SUFFIXES

if TYPE_CHECKING:
    from shared_scripts.handoff_schema import HandoffFinding
else:
    HandoffFinding = _schema.HandoffFinding
OVERRIDE_PATTERN = _schema.OVERRIDE_PATTERN
PLACEHOLDER_PATTERN = _schema.PLACEHOLDER_PATTERN
REQUIRED_HEADINGS = _schema.REQUIRED_HEADINGS
REQUIRED_IDENTITY_FIELDS = _schema.REQUIRED_IDENTITY_FIELDS
SAFE_EXEMPT_PATHS = _schema.SAFE_EXEMPT_PATHS
SAFE_EXEMPT_SUFFIXES = _schema.SAFE_EXEMPT_SUFFIXES
SECRET_PATTERNS = _schema.SECRET_PATTERNS
check_handoff_freshness = _schema.check_handoff_freshness
get_git_head = _schema.get_git_head
is_implementation_file = _schema.is_implementation_file
requires_handoff_update = _schema.requires_handoff_update
resolve_canonical_handoff_path = _schema.resolve_canonical_handoff_path

__all__ = [
    "HEX_COMMIT_PATTERN",
    "HandoffFinding",
    "IMPLEMENTATION_PREFIXES",
    "IMPLEMENTATION_SUFFIXES",
    "OVERRIDE_PATTERN",
    "PLACEHOLDER_PATTERN",
    "REQUIRED_HEADINGS",
    "REQUIRED_IDENTITY_FIELDS",
    "SAFE_EXEMPT_PATHS",
    "SAFE_EXEMPT_SUFFIXES",
    "SECRET_PATTERNS",
    "check_handoff_freshness",
    "get_git_head",
    "is_implementation_file",
    "main",
    "requires_handoff_update",
    "resolve_canonical_handoff_path",
    "sibling",
    "validate_handoff_content",
    "validate_repository_handoff",
]


def validate_handoff_content(
    content: str,
    path: Path,
    is_template: bool = False,
) -> list[HandoffFinding]:
    """Validate handoff content against the canonical schema."""
    findings: list[HandoffFinding] = []
    lines = content.splitlines()

    # 1. Level 1 title check
    if not content.startswith("# ") and not re.search(
        r"^#\s+.*Handoff", content, re.MULTILINE
    ):
        findings.append(
            HandoffFinding(
                path=path,
                line=1,
                kind="missing_title",
                message="Document must start with '# Implementation Handoff'.",
                remediation="Add '# Implementation Handoff' as the first heading.",
            )
        )

    # 2. Required section headings
    for heading_title, pattern in REQUIRED_HEADINGS:
        if not pattern.search(content):
            findings.append(
                HandoffFinding(
                    path=path,
                    line=None,
                    kind="missing_section",
                    message=f"Missing required section '{heading_title}'.",
                    remediation=(
                        f"Add '{heading_title}' section per docs/templates/HANDOFF.md."
                    ),
                )
            )

    # 3. Required identity fields under ## Identity
    identity_match = re.search(
        r"^##\s+Identity\s*\n(.*?)(?=\n##|\Z)", content, re.DOTALL | re.MULTILINE
    )
    if identity_match:
        identity_text = identity_match.group(1)
        for field in REQUIRED_IDENTITY_FIELDS:
            field_re = re.compile(
                rf"^-\s+{re.escape(field)}:", re.MULTILINE | re.IGNORECASE
            )
            if not field_re.search(identity_text):
                findings.append(
                    HandoffFinding(
                        path=path,
                        line=None,
                        kind="missing_field",
                        message=f"Missing required Identity field '- {field}:'.",
                        remediation=f"Add '- {field}: <value>' under '## Identity'.",
                    )
                )

        # Validate Implementation commit value
        commit_match = re.search(
            r"^-\s+Implementation commit:\s*([^\n]+)",
            identity_text,
            re.MULTILINE | re.IGNORECASE,
        )
        if commit_match and not is_template:
            commit_val = commit_match.group(1).strip()
            first_token = commit_val.split()[0].strip("`'\",")
            if first_token != "SELF" and not HEX_COMMIT_PATTERN.match(first_token):
                findings.append(
                    HandoffFinding(
                        path=path,
                        line=None,
                        kind="invalid_commit",
                        message=(
                            f"Implementation commit '{first_token}' is not "
                            "'SELF' or a valid SHA."
                        ),
                        remediation=(
                            "Set 'Implementation commit: `SELF`' or the exact SHA."
                        ),
                    )
                )

    # 4. Check for unedited placeholders when not validating the template itself
    if not is_template:
        for idx, line in enumerate(lines, start=1):
            if line.strip().startswith("<!--") or line.strip().startswith("```"):
                continue
            for match in PLACEHOLDER_PATTERN.finditer(line):
                placeholder = match.group(0)
                findings.append(
                    HandoffFinding(
                        path=path,
                        line=idx,
                        kind="placeholder",
                        message=f"Unedited template placeholder: {placeholder}",
                        remediation=(
                            f"Replace '{placeholder}' on line {idx} with evidence."
                        ),
                    )
                )

    # 5. Validate Change log section
    changelog_match = re.search(
        r"^##\s+Change\s*log\s*\n(.*?)(?=\n##|\Z)",
        content,
        re.DOTALL | re.MULTILINE | re.IGNORECASE,
    )
    if changelog_match:
        changelog_text = changelog_match.group(1).strip()
        bullet_items = [
            line.strip()
            for line in changelog_text.splitlines()
            if line.strip().startswith("- ") or line.strip().startswith("* ")
        ]
        if not bullet_items:
            findings.append(
                HandoffFinding(
                    path=path,
                    line=None,
                    kind="invalid_changelog",
                    message="Change log must contain at least one bullet entry.",
                    remediation=(
                        "Add '- `SELF` — <changes>' or "
                        "'- `SELF` — No material handoff change — <reason>'."
                    ),
                )
            )

    # 6. Secret scanning
    for idx, line in enumerate(lines, start=1):
        for pattern, secret_type in SECRET_PATTERNS:
            if pattern.search(line):
                findings.append(
                    HandoffFinding(
                        path=path,
                        line=idx,
                        kind="secret_detected",
                        message=f"Potential {secret_type} detected in handoff.",
                        remediation=(
                            "Remove credentials, tokens, or secret keys from handoff."
                        ),
                    )
                )

    return findings


def validate_repository_handoff(
    repo_root: Path,
    changed_files: Sequence[str] | None = None,
    warn_only: bool = False,
) -> list[HandoffFinding]:
    """Validate repository handoff status and compliance."""
    findings: list[HandoffFinding] = []
    canonical_path = resolve_canonical_handoff_path(repo_root)

    # Check existence
    if not canonical_path.is_file():
        rel_disp = (
            canonical_path.relative_to(repo_root).as_posix()
            if repo_root in canonical_path.parents or canonical_path == repo_root
            else canonical_path.as_posix()
        )
        findings.append(
            HandoffFinding(
                path=canonical_path,
                line=None,
                kind="missing_handoff",
                message=f"Canonical handoff file not found at '{rel_disp}'.",
                remediation="Create canonical handoff from docs/templates/HANDOFF.md.",
            )
        )
        return findings

    # Read and validate canonical content
    try:
        content = canonical_path.read_text(encoding="utf-8")
        content_findings = validate_handoff_content(content, path=canonical_path)
        findings.extend(content_findings)
        findings.extend(check_handoff_freshness(repo_root, canonical_path, content))
    except OSError as err:
        findings.append(
            HandoffFinding(
                path=canonical_path,
                line=None,
                kind="read_error",
                message=f"Unable to read canonical handoff: {err}",
                remediation="Ensure file exists and has read permissions.",
            )
        )
        return findings

    # Check commit-level enforcement if changed_files are supplied
    if changed_files:
        norm_changed = [p.replace("\\", "/") for p in changed_files]
        if requires_handoff_update(norm_changed):
            canonical_rel = canonical_path.relative_to(repo_root).as_posix()
            if canonical_rel not in norm_changed and canonical_path.name not in [
                Path(p).name for p in norm_changed
            ]:
                findings.append(
                    HandoffFinding(
                        path=canonical_path,
                        line=None,
                        kind="uncommitted_handoff",
                        message=(
                            "Implementation files modified without updating "
                            f"canonical handoff at '{canonical_rel}'."
                        ),
                        remediation=(
                            f"Stage an update to '{canonical_rel}', or add "
                            "'- `SELF` — No material handoff change — <reason>'."
                        ),
                    )
                )

    return findings


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entrypoint for handoff validation."""
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="backslashreplace")

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo",
        type=Path,
        default=Path.cwd(),
        help="Root path of repository to validate (default: current directory)",
    )
    parser.add_argument(
        "--warn-only",
        action="store_true",
        help="Report findings as warnings without returning non-zero exit status",
    )
    parser.add_argument(
        "--check-template",
        action="store_true",
        help="Validate docs/templates/HANDOFF.md as a template schema definition",
    )
    parser.add_argument(
        "files",
        nargs="*",
        help="Optional list of changed files to check for commit enforcement",
    )

    args = parser.parse_args(argv)
    repo_root = args.repo.resolve()

    if args.check_template:
        template_path = repo_root / "docs" / "templates" / "HANDOFF.md"
        if not template_path.is_file():
            print(f"ERROR: Template file not found at {template_path}")
            return 1
        content = template_path.read_text(encoding="utf-8")
        findings = validate_handoff_content(
            content, path=template_path, is_template=True
        )
    else:
        findings = validate_repository_handoff(
            repo_root=repo_root,
            changed_files=args.files or None,
            warn_only=args.warn_only,
        )

    if not findings:
        print("Canonical handoff validation passed.")
        return 0

    label = "WARNING" if args.warn_only else "ERROR"
    print(f"{label}: durable implementation handoff")
    for finding in findings:
        loc = f":{finding.line}" if finding.line else ""
        rel_path = (
            finding.path.relative_to(repo_root).as_posix()
            if repo_root in finding.path.parents or finding.path == repo_root
            else finding.path.as_posix()
        )
        print(f"  - {rel_path}{loc} [{finding.kind}]: {finding.message}")
        print(f"    Remediation: {finding.remediation}")

    return 0 if args.warn_only else 1


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUTF8", "1")
    raise SystemExit(main())
