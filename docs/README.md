# DUK Engineering Documentation

This documentation set defines the system architecture, artifact ownership, engineering conventions and reproducibility requirements across the DUK robotics project.

## Architecture

| Document | Scope |
| --- | --- |
| [System architecture](architecture/overview.md) | Subsystem relationships, dependency directions and deployment boundaries |
| [Repository layout](architecture/repository-layout.md) | Canonical directory ownership and placement rules |
| [Data contracts](architecture/data-contracts.md) | State, units, frames, timestamps and interface conventions |
| [Parameter governance](architecture/parameter-governance.md) | Sources of truth and mechanical–software consistency |

## Engineering disciplines

| Document | Scope |
| --- | --- |
| [Mechanical design](hardware/mechanical.md) | Editable CAD, assemblies, drawings, fabrication assets |
| [Electronics design](hardware/electronics.md) | PCB projects, wiring, power and manufacturing packages |
| [Robotics core](software/core.md) | Modeling, control, visualization and interactive numerical work |
| [ROS 2 integration](software/ros2.md) | Package boundaries, dependencies, integration tests |
| [Firmware](software/firmware.md) | MCU targets, protocols, safety and release traceability |
| [Simulation](simulation/README.md) | Worlds, scenarios, dynamics and validation |
| [Applications](applications/README.md) | Operator interfaces, gateway and mobile responsibilities |

## Research and quality

| Document | Scope |
| --- | --- |
| [Research methodology](research/methodology.md) | Experiment design, benchmarks, versioned evidence |
| [Experiment protocol](experiments/README.md) | Recording conditions, execution and result analysis |
| [Mathematical theory](theory/README.md) | Model assumptions, mathematical derivations and references |
| [Engineering rules](development/engineering-rules.md) | Contribution and coding conventions |
| [Testing strategy](development/testing.md) | Verification layers and acceptance evidence |
| [Development environment](development/environment.md) | Local setup, Docker, Dev Containers and ROS |
| [Platform compatibility](platforms/compatibility.md) | Supported combinations and verification status |
| [Architecture decisions](decisions/README.md) | Design records and long-lived trade-offs |

Documentation changes should accompany the engineering artifacts they describe. **Do not use documentation to assert compatibility or functionality without corresponding implementation and verification evidence.**
