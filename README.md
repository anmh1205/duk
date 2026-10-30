# DUK

DUK is a wheel-legged biped robot project. This monorepo is organized to support the full engineering lifecycle: mechanical design, electronics, firmware, robotics software, mathematical modeling, simulation, research, applications and robot operations.

## Repository navigation

| Area | Start here |
| --- | --- |
| Full repository structure and artifact ownership | [Repository layout](docs/architecture/repository-layout.md) |
| Engineering documentation | [Documentation index](docs/README.md) |
| Robot hardware, software and simulation | [Robot](robot/README.md) |
| Numerical models and interactive development | [Robotics core](robot/software/core/README.md) |
| Scientific work and publications | [Research](research/README.md) |
| Operator-facing and external applications | [Applications](applications/README.md) |
| Development and deployment environments | [Environment](environment/README.md) |
| Maintained utilities and ad hoc scripts | [Tools](tools/README.md) · [Scripts](scripts/README.md) |
| Contribution and agent guidance | [Contributing](CONTRIBUTING.md) · [AGENTS.md](AGENTS.md) |

## Scope

```text
robot/          Physical design, robot software, simulation and configuration
research/       Experiments, benchmarks and publications
applications/   Operator interfaces and external applications
environment/    Development environments and target-platform definitions
tools/          Maintained engineering tools
scripts/        Local developer scripts
docs/           Technical documentation
tests/          Cross-component testing
data/           Local recordings, results and generated artifacts
```

The repository contains both real files and reserved directories. A reserved ROS package, firmware target, robot model, application or engineering document does not imply that its implementation or design has been approved.

## Documentation approach

The documentation tree reserves a production-scale scope. The [repository layout](docs/architecture/repository-layout.md) describes the established folder organization. Area README files direct contributors to the relevant materials. Technical specifications, methods and operational procedures remain placeholders until the project reaches the decisions that establish them.

## License

[MIT License](LICENSE).
