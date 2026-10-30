# DUK — Repository Agent Contract

This file governs agent-assisted changes throughout the DUK monorepo. Agents must read [README.md](README.md), [Documentation](docs/README.md) and [Engineering Rules](docs/development/engineering-rules.md), then any narrower `AGENTS.md` covering the edited path.

## Operating principles

1. **Respect ownership.** Place each change in the subsystem that owns it. Do not create parallel core, workbench, simulation, or CI implementations.
2. **Maintain truthful artifacts.** A directory, package, test, Dockerfile or configuration must not imply capabilities that have not been implemented and verified.
3. **Preserve model contracts.** Every physical value requires units, frame conventions, provenance and applicable revision. Never invent DUK-specific dimensions or dynamics.
4. **Prefer reusable APIs.** Mathematical implementations belong in the core source package; notebooks and experiment runners consume them.
5. **Keep integration thin.** ROS nodes and application gateways translate interfaces rather than reimplement robot mathematics and control design.
6. **Preserve existing work.** Inspect code and repository status before editing; modify only the requested scope and avoid unrelated renames, generated artifacts and history rewrites.
7. **Verify changes.** Run relevant checks and report executed commands and remaining unverified targets. Never claim a hardware or platform test passed without evidence.
8. **Protect hardware.** Do not flash firmware, command actuators, change safety limits or deploy to an actual robot without explicit authorization.

## References by task

| Scope | Required reference |
| --- | --- |
| Repository organization | [Repository layout](docs/architecture/repository-layout.md) |
| Physical parameters, frames and interfaces | [Data contracts](docs/architecture/data-contracts.md) |
| Core mathematical software | [Core architecture](docs/software/core.md) |
| ROS 2 packages and launch | [ROS architecture](docs/software/ros2.md) |
| CAD, electronics and firmware | [Hardware](docs/hardware/README.md) and [Firmware](docs/software/firmware.md) |
| Scientific studies | [Research methodology](docs/research/methodology.md) |
| CI, tests and developer environment | [Testing](docs/development/testing.md) and [Environment](docs/development/environment.md) |

## Change completion

A contribution includes its implementation, appropriately scoped tests, documentation updates, and a concise account of verification. Avoid placeholder algorithms, fake benchmark results, speculative runtime compatibility and unnecessary scaffolding.
