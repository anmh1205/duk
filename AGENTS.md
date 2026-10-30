# DUK — Agent Instructions

## Required reading

Before modifying the repository:
1. Read [README.md](README.md).
2. Read [docs/README.md](docs/README.md).
3. Follow [engineering rules](docs/development/engineering-rules.md) and the documents relevant to the task.
4. Read a more specific `AGENTS.md` if one exists within the affected directory.

## Architecture contracts

- `robot/` owns the DUK product: hardware, software, simulation and configuration.
- `robot/software/core/` owns reusable Python mathematics and controller design; avoid dependencies on ROS and Docker in the core modeling/control modules.
- Core notebooks explore existing APIs. Do not keep independent canonical algorithms inside notebooks.
- `robot/software/ros2_ws/` integrates core algorithms with ROS. Placeholder directories are **not** valid ROS packages.
- `research/` owns reproducible scientific experiments, not a duplicate visualization workbench.
- `applications/` does not own low-level motor control.
- `.github/workflows/` owns CI/CD; `scripts/` is for ad hoc developer scripts. Reusable engineering logic goes in `tools/`.

## Engineering discipline

- Preserve a single source of truth for robot geometry and physical parameters; record units, frames and provenance.
- Do not invent real robot dimensions, inertias, controllers or device compatibility.
- Update relevant tests and documentation with code changes.
- Do not claim tests or Docker/Jetson deployment passed without executing them.
- Never command real actuators, flash firmware or deploy to hardware without explicit authorization.

## References

- [Repository boundaries](docs/architecture/repository-layout.md)
- [Testing](docs/development/testing.md)
- [Development environment](docs/development/environment.md)
- [Platform status](docs/platforms/compatibility.md)
