# Drake_Models

Drake (pydrake) multibody models for five classical barbell exercises:

- **Back Squat** -- barbell on upper trapezius, sagittal-plane hip/knee/ankle flexion
- **Bench Press** -- supine press, barbell gripped at shoulder width
- **Deadlift** -- floor to lockout, hip-hinge dominant
- **Snatch** -- floor to overhead in one motion, wide grip
- **Clean & Jerk** -- floor to shoulders (clean) then overhead (jerk)

## Architecture

Models are generated as **SDFormat (SDF 1.8)** XML using the Z-up convention (Drake default). The SDF files can be loaded into Drake via `Parser().AddModels()`.

```
src/drake_models/
  shared/
    contracts/     # Design-by-Contract precondition/postcondition guards
    utils/         # Geometry (inertia math) and SDF XML helpers
    barbell/       # Olympic barbell model (IWF/IPF spec)
    body/          # Full-body model (Winter 2009 anthropometrics)
  exercises/
    base.py        # Abstract ExerciseModelBuilder
    squat/         # Back squat
    bench_press/   # Bench press
    deadlift/      # Conventional deadlift
    snatch/        # Snatch
    clean_and_jerk/ # Clean & jerk
```

## Quick Start

```bash
pip install -e ".[dev]"
python3 -m pytest tests/ -v
```

Generate a model:

```python
from drake_models.exercises.squat.squat_model import build_squat_model

sdf_xml = build_squat_model(body_mass=80.0, height=1.75, plate_mass_per_side=60.0)
print(sdf_xml)
```

Load into Drake:

```python
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant

plant = MultibodyPlant(time_step=0.001)
parser = Parser(plant)
parser.AddModelsFromString(sdf_xml, "sdf")
plant.Finalize()
```

## Design Principles

- **TDD** -- Tests written first, 100% coverage
- **Design-by-Contract** -- Preconditions validate inputs, postconditions validate outputs
- **DRY** -- Geometry and XML generation defined once in shared modules
- **Law of Demeter** -- Callers use public APIs only

## Requirements

- Python 3.10+
- [Drake](https://drake.mit.edu/) (pydrake) — multibody dynamics and control
- NumPy, SciPy, Matplotlib

## Installation

```bash
# Clone the repository
git clone https://github.com/D-sorganization/Drake_Models.git
cd Drake_Models

# Create a virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install in development mode
pip install -e ".[dev]"
```

## Running Tests

```bash
# Run the full test suite
python3 -m pytest tests/ -v

# Run with coverage
pytest tests/ --cov=src --cov-report=html

# Run a specific test module
pytest tests/test_squat_model.py -v
```

## CI/CD

This repository uses GitHub Actions with a self-hosted runner (`d-sorg-fleet`):
- **Quality gate**: ruff linting, formatting, mypy type checking, bandit security scan
- **Tests**: Python 3.10–3.12 matrix with pytest

## Contributing

Contributions are welcome. Please:
1. Open an issue to discuss your proposed change.
2. Create a feature branch from `main`.
3. Follow TDD and Design-by-Contract principles.
4. Ensure all CI checks pass before requesting review.
5. Reference the issue in your PR description.

## Troubleshooting

| Issue | Solution |
|---|---|
| `ImportError: No module named 'pydrake'` | Install Drake following the [official instructions](https://drake.mit.edu/installation.html) or use `pip install drake` |
| `pytest` not found | Install dev dependencies: `pip install -e ".[dev]"` |
| SDF load errors | Verify Drake is on PATH and `DRAKE_RESOURCE_ROOT` is set |

## License

MIT License. See [LICENSE](LICENSE).
