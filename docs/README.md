# DUK Documentation

The DUK documentation structure covers the full intended engineering and product lifecycle. Its directory organization may be established before individual engineering designs, requirements and verification procedures have been decided.

**Documented now:** the [repository layout](architecture/repository-layout.md), development environment, [static simulation preview](development/simulation-preview.md), and [kinematics / parameterized candidate dynamics](theory/modeling.md), alongside area navigation and agent guidance.

**Reserved where marked:** technical architecture, hardware specifications, controller decisions, research methods and operational plans. A populated modeling or preview page does not identify physical parameters or establish a working controller.

## Documentation navigation

| Area | Contents |
| --- | --- |
| [Architecture](architecture/README.md) | Repository organization, system design and cross-component contracts |
| [Hardware](hardware/README.md) | Mechanical, electrical, manufacturing and assembly |
| [Software](software/README.md) | Robotics core, ROS 2, firmware and diagnostics |
| [Theory](theory/README.md) | Mathematical modeling and control |
| [Simulation](simulation/README.md) | Simulation design and validation |
| [Research](research/README.md) | Research methodology and benchmarks |
| [Experiments](experiments/README.md) | Experiment execution and evidence |
| [Applications](applications/README.md) | Operator and external applications |
| [Development](development/README.md) | Onboarding, tooling, testing and engineering practices |
| [Operations](operations/README.md) | Safety, commissioning, maintenance and releases |
| [Platforms](platforms/README.md) | Target compatibility and verification |
| [Decisions](decisions/README.md) | Engineering decisions when formally established |

## Source of information

A page's presence does not make its subject an adopted design. When a technical decision is made, its owning source artifacts and corresponding documentation must be updated together. Until then, a reserved document remains deliberately free of assumed implementation details.

[Project README](../README.md) · [Agent instructions](../AGENTS.md).
