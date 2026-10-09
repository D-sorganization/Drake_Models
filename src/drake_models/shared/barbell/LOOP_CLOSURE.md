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

## Solution: Left Fixed Joint + Right-Hand Weld Constraint

The SDF generator attaches the barbell to the left hand with a fixed joint:

```
hand_l  --[fixed]--> barbell_shaft   (valid: single parent)
```

and declares the right hand as a namespaced `biomech:weld` element, which the
SDF parser ignores:

```xml
<biomech:weld name="barbell_to_right_hand" parent="hand_r" child="barbell_shaft"/>
```

`drake_models.loader.load_sdf` reads it (`parse_weld_constraints`) and, on a
discrete plant (`time_step > 0`; Drake supports weld constraints only there),
calls `MultibodyPlant.AddWeldConstraint()` before `Finalize()`.  The weld frame
is the bar's pose in the `hand_r` frame measured at the initial pose, so the
closure starts with zero residual (`weld_residuals`) and cannot over-constrain
the neutral pose.  Continuous plants (`time_step = 0`) record the specs but add
no constraint.

Note: the nominal `GRIP_OFFSET` only places the bar on the left hand.  The
right hand is welded where the initial pose puts it, so the bar is not
necessarily centred between the hands.

## Other Runtime Constraints

For a compliant grip (bar rotating or sliding in the hands) use other Drake
constraints instead of a rigid weld:

1. `MultibodyPlant.AddDistanceConstraint()` -- fixed distance between two points.
2. `MultibodyPlant.AddBallConstraint()` -- coincident points (3-DOF), a
   ball-and-socket grip.

Note that `AddWeldConstraint(body_A, X_AP, body_B, X_BQ)` takes bodies and
poses on them, not frames.

## Per-Exercise Attachment Strategy

| Exercise       | Attachment Point     | Grip Type      | Notes                              |
|----------------|---------------------|----------------|------------------------------------|
| Back Squat     | Torso (trap height) | Torso weld     | Bar rests on upper trapezius       |
| Deadlift       | hand_l + hand_r weld | Bilateral grip | Floor to lockout                   |
| Bench Press    | hand_l + hand_r weld | Bilateral grip | Supine; pelvis welded to bench     |
| Snatch         | hand_l + hand_r weld | Wide grip      | Grip offset ~0.58 m from center    |
| Clean & Jerk   | hand_l + hand_r weld | Clean grip     | Grip offset ~0.25 m from center    |
| Gait           | None                | N/A            | No barbell                         |
| Sit-to-Stand   | None                | N/A            | No barbell; chair body added       |

## References

- Drake SDF documentation: https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html
- SDFormat 1.8 specification: http://sdformat.org/spec?ver=1.8
- `ExerciseModelBuilder._attach_bilateral_grip()` in `exercises/base.py`
- `create_barbell_links()` in `shared/barbell/barbell_model.py`
