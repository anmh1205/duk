# Contributing to DUK

DUK spans mechanical design, electronics, embedded firmware, mathematical software, ROS 2, simulation, research and operational applications. Contributions should preserve the boundaries between these disciplines while keeping changes traceable and reviewable.

## Before making changes

Read the [repository overview](README.md), [layout contract](docs/architecture/repository-layout.md) and [engineering rules](docs/development/engineering-rules.md). Use the closest subsystem documentation to identify the authoritative source and affected consumers. Avoid creating a second copy of a model, algorithm, board library or generated asset.

## Development workflow

1. Identify the owning subsystem and the interfaces affected by the change.
2. Document assumptions: units, frames, robot configuration, software targets, and any external hardware requirements.
3. Implement the smallest coherent change that preserves the agreed architecture.
4. Add or update validation appropriate to the artifact: unit tests, schematic review, CAD checks, model consistency checks, ROS integration tests or scientific experiments.
5. Update API, configuration, build and architectural documentation in the same contribution.
6. Open a pull request explaining impact, tested targets and unresolved risks. Avoid committing caches, debug captures, generated output or machine-specific settings.

## Change-specific evidence

| Change | Expected evidence |
| --- | --- |
| Mathematical model or controller | Assumptions, parameter provenance, unit/frame conventions, numerical checks and comparison to reference data |
| CAD or manufacturing release | Editable source, revision, critical dimensions, export traceability and fabrication notes |
| PCB or wiring | Schematic/PCB source, electrical constraints, connector pinout and design-rule review |
| Firmware | Target board, toolchain, pin assignments, protocol contract, rollback/recovery and hardware-test status |
| ROS 2 integration | Package manifest, interface compatibility, build and launch/test results |
| Simulation | Model revision, physics assumptions, scenario setup and output comparison |
| Research experiment | Configuration, seed or repeatability controls, metrics, run provenance and result analysis |
| Docker/CI/platform | Container image/version, tested host and architecture, reproducible commands and logs |

## Review and merge

Reviewers evaluate correctness, source ownership, safety implications, reproducibility and whether tests substantiate the change. A green CI workflow is necessary for checked software paths but does not replace mechanical, electrical or physical-robot verification.

The [Pull Request template](.github/PULL_REQUEST_TEMPLATE.md) captures scope, verification and deployment impact. Architectural decisions that alter long-lived boundaries should be recorded as an [ADR](docs/decisions/README.md).

## Licensing and sensitive information

The repository uses the [MIT License](LICENSE). Confirm third-party licenses before committing vendor designs or code. Never commit credentials, private network configuration, customer data, personal information, unreleased proprietary schematics or device secrets.

## Safety

Do not test a new controller on unconstrained actuators. Physical experiments require an authorized test plan, defined limits, emergency-stop procedure and supervision appropriate to the test.
