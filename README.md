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

## License

MIT License. See [LICENSE](LICENSE).
