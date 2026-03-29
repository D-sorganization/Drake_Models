# Drake Loop-Closure Constraints for Barbell Attachment

## Problem: SDF 1.8 Kinematic Tree Invariant

SDFormat 1.8 requires a **strict kinematic tree**: every `<link>` may be the
`<child>` of at most one `<joint>`.  This means you cannot create two fixed
joints that both claim the same link as their child.

In barbell exercises, the lifter grips the bar with **both hands**.
Naively this would require:

```
hand_l  --[fixed]--> barbell_shaft   (grip left)
hand_r  --[fixed]--> barbell_shaft   (grip right)
```

This violates the tree invariant because `barbell_shaft` would be the child
of two joints.

## Solution: Single-Parent Attachment

The SDF generator attaches the barbell via **one hand only**:

```
hand_l  --[fixed]--> barbell_shaft   (valid: single parent)
```

For a fully rigid barbell, attaching via one hand is **kinematically
equivalent** to attaching via both hands.  The barbell's three links
(left_sleeve, shaft, right_sleeve) are already connected by fixed joints
(welds), so the entire barbell assembly moves as a rigid body.

The right hand's grip is **not expressed in the SDF** because adding a
second fixed joint from `hand_r` to `barbell_shaft` would create a
kinematic loop.

## When Loop Closure Matters

If you need **compliant grip** (e.g., the bar can rotate slightly in the
hands, or the hands can slide along the bar), you must use Drake's
runtime constraint mechanisms rather than SDF joints:

1. **`MultibodyPlant.AddDistanceConstraint()`** -- constrains the distance
   between two frames to a fixed value.  Suitable for keeping `hand_r`
   at a fixed offset from `barbell_shaft` while allowing the tree topology
   to remain valid.

2. **`MultibodyPlant.AddBallConstraint()`** -- constrains two frames to
   share the same position (3-DOF point constraint).  Useful for a ball-
   and-socket grip model.

3. **`MultibodyPlant.AddWeldConstraint()`** -- a runtime weld that does
   not participate in the SDF tree.  This is the most direct analogue of
   "weld hand_r to barbell_shaft" without violating tree topology.

These constraints are applied **after** loading the SDF into a
`MultibodyPlant`, not within the SDF file itself.

### Example: Adding a Runtime Weld for the Right Hand

```python
from pydrake.all import Parser, AddMultibodyPlantSceneGraph, DiagramBuilder

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
parser = Parser(plant)
parser.AddModelsFromString(sdf_string, "sdf")

# Add loop-closure weld for right hand grip
hand_r_frame = plant.GetFrameByName("hand_r")
shaft_frame = plant.GetFrameByName("barbell_shaft")

# RigidTransform specifying grip offset along Y axis
from pydrake.math import RigidTransform
grip_offset = RigidTransform([0, grip_width, 0])

plant.AddWeldConstraint(
    frame_on_parent_F=hand_r_frame,
    X_PF=RigidTransform(),
    frame_on_child_M=shaft_frame,
    X_CM=grip_offset,
)

plant.Finalize()
```

## Per-Exercise Attachment Strategy

| Exercise       | Attachment Point     | Grip Type      | Notes                              |
|----------------|---------------------|----------------|------------------------------------|
| Back Squat     | Torso (trap height) | Torso weld     | Bar rests on upper trapezius       |
| Deadlift       | hand_l              | Bilateral grip | Floor to lockout                   |
| Bench Press    | hand_l              | Bilateral grip | Supine; pelvis welded to bench     |
| Snatch         | hand_l              | Wide grip      | Grip offset ~0.58 m from center    |
| Clean & Jerk   | hand_l              | Clean grip     | Grip offset ~0.25 m from center    |
| Gait           | None                | N/A            | No barbell                         |
| Sit-to-Stand   | None                | N/A            | No barbell; chair body added       |

## References

- Drake SDF documentation: https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html
- SDFormat 1.8 specification: http://sdformat.org/spec?ver=1.8
- `ExerciseModelBuilder._attach_bilateral_grip()` in `exercises/base.py`
- `create_barbell_links()` in `shared/barbell/barbell_model.py`
