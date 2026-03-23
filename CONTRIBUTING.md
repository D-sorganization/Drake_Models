# Contributing to Drake Models

Thank you for your interest in contributing to Drake Models.

## Development Setup

```bash
git clone https://github.com/D-sorganization/Drake_Models.git
cd Drake_Models
pip install -e ".[dev]"
pre-commit install
```

## Code Quality

All code must pass:

- **Linting**: `ruff check src tests scripts`
- **Formatting**: `ruff format --check src tests scripts`
- **Type checking**: `mypy src --config-file pyproject.toml`
- **Tests**: `python3 -m pytest tests/ -v`

## Pull Request Process

1. Create a feature branch from `main`.
2. Write tests for any new functionality.
3. Ensure all CI checks pass before requesting review.
4. Keep PRs focused on a single concern.

## Coding Standards

- Use type annotations on all public functions (`disallow_untyped_defs = true`).
- Use `logging.getLogger(__name__)` instead of `print()` in library code.
- Use Design-by-Contract: validate inputs with precondition guards.
- Follow Drake Z-up convention (gravity along -Z, forward along X).
- Use `xml.etree.ElementTree` for all XML generation (no lxml dependency).
- Extract magic numbers to named constants.

## Reporting Issues

Open a GitHub issue with a clear description and, if applicable, a minimal
reproduction case.
