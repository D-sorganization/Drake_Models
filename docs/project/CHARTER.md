# Project Charter

> Drafted 2026-09-25 by the fleet charter sweep (Gemini) from README, git history, and open issues/PRs.
> The project-steward role keeps this current; owners should correct feature statuses.

## End Goal

Drake_Models provides validated, parametric multibody model generators for classical barbell exercises and human movements, emitting standardized SDFormat (SDF 1.8) XML for direct loading into Drake (pydrake). The repository is "done" when it reliably delivers contract-validated SDF generation for all supported exercise motions (back squat, bench press, deadlift, snatch, clean & jerk, gait, and sit-to-stand), integrates seamlessly with the UpstreamDrift fleet launcher contract (`model_pack.yaml`), provides complete trajectory optimization and inverse kinematics helpers, and verifies kinematic and dynamic fidelity via automated design-by-contract test suites without requiring a runtime Drake dependency for baseline testing.

## Non-Goals

- Requiring `pydrake` at test time: model generation and XML structural integrity must remain verifiable in pure Python test environments.
- Serving as a standalone physics simulator or 3D renderer: Drake_Models generates model definitions and optimization objectives for Drake rather than embedding its own physics solver.
- Supporting non-SDFormat model formats (e.g. URDF, MJCF, OpenSim .osim) directly within this package.
- Implementing real-time interactive motion-capture visualization tools or graphical user interfaces.

## Features

| ID | Feature | Status | Tracking | Notes |
| --- | --- | --- | --- | --- |
| F1 | Back Squat Model | shipped | - | Generates SDF 1.8 back squat model with sagittal hip knee and ankle flexion |
| F2 | Bench Press Model | shipped | #53 | Generates SDF 1.8 supine press model with shoulder-width barbell grip |
| F3 | Conventional Deadlift Model | shipped | #50 | Generates SDF 1.8 hip-hinge dominant deadlift model from floor to lockout |
| F4 | Snatch Model | shipped | #54 | Generates SDF 1.8 Olympic snatch model for floor-to-overhead motion |
| F5 | Clean & Jerk Model | shipped | #51 | Generates SDF 1.8 two-phase clean and overhead jerk model |
| F6 | Gait & Sit-to-Stand Builders | shipped | #68 | Generates functional human movement models for gait and sit-to-stand tasks |
| F7 | Anthropometric Full-Body Model | shipped | #112 | Segmented human body model using Winter 2009 anthropometric scaling |
| F8 | Olympic Barbell Assembly | shipped | #52 | Three-link IWF and IPF compliant barbell model with plate mass options |
| F9 | Design-by-Contract Guard System | shipped | #118 | Runtime precondition and postcondition validations for kinematics and inertia |
| F10 | CLI Model Generator | shipped | - | Thin command-line interface drake-models for generating and saving SDF XML |
| F11 | UpstreamDrift Model Pack Contract | shipped | #240 | Exposes model_pack.yaml manifest and entry points for fleet discovery |
| F12 | Trajectory Optimization & IK Utilities | shipped | #142 | Cost functions direct transcription and inverse kinematics utilities |
| F13 | Biomechanics DBC Test Suite | shipped | #220 | 43 design-by-contract tests verifying kinematics and SDF without Drake |
| F14 | Rust Acceleration Core | shipped | #91 | Optional native Rust accelerator under rust_core for batch dynamics |
| F15 | C4 Architecture Map Contract | shipped | - | Automated C4 Mermaid diagrams and traceability validated via CI |

## Links

- Status (generated): [`STATUS.md`](STATUS.md)
- Steward playbook: Repository_Management `docs/fleet-project-steward.md`
