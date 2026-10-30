# DUK — Wheel-Legged Biped Robot

DUK is a monorepo for the design, development, research, and integration of a wheel-legged bipedal robot. It unifies the robot's mechanical and electronic design, mathematical models, control software, simulation, scientific evaluation, and supporting applications under a single engineering source tree.

The repository is organized around **engineering ownership**. Robot-specific design and software live under `robot/`; scientific studies live under `research/`; operational applications, development environments, tooling, and documentation have separate responsibilities. The directory layout describes the architecture and does not, by itself, assert that a subsystem has been implemented or validated.

## Architecture at a glance

| Area | Purpose | Primary entry |
| --- | --- | --- |
| Mechanical engineering | Assemblies, parts, drawings, manufacturing deliverables | [Mechanical design](docs/hardware/mechanical.md) |
| Electronics | PCB projects, power distribution, wiring and shared component libraries | [Electronics design](docs/hardware/electronics.md) |
| Robotics core | Reusable numerical modeling, control algorithms, plotting and interactive experiments | [Core architecture](docs/software/core.md) |
| ROS 2 integration | Interfaces, state estimation, hardware abstraction, simulation integration and bringup | [ROS 2 architecture](docs/software/ros2.md) |
| Embedded software | Firmware and hardware-specific communication | [Firmware architecture](docs/software/firmware.md) |
| Simulation | Worlds, scenarios and model-consistency validation | [Simulation](docs/simulation/README.md) |
| Research | Reproducible experiments, benchmarks and publications | [Research methodology](docs/research/methodology.md) |
| Applications | Operator interfaces, gateway and mobile clients | [Application architecture](docs/applications/README.md) |
| Environments | Development containers, ROS dependencies, platform variants | [Development environment](docs/development/environment.md) |
| Project governance | Engineering practices, review, testing and architectural decisions | [Contributing](CONTRIBUTING.md) |

## Repository layout

```text
duk/
├── .devcontainer/                  # IDE development-container integration
├── .github/                        # GitHub Actions and collaboration templates
├── robot/
│   ├── hardware/
│   │   ├── mechanical/             # Editable CAD, drawings and manufacturing
│   │   └── electronics/            # PCB projects, power, wiring and libraries
│   ├── software/
│   │   ├── core/                   # ROS-independent robotics Python package
│   │   ├── ros2_ws/                # ROS 2 packages and integration adapters
│   │   └── firmware/               # Embedded targets
│   ├── simulation/                 # Simulation environments and scenarios
│   └── config/                     # Physical, controller and calibration data
├── research/                       # Experiments, benchmarks and publications
├── applications/                   # Operator dashboard, gateway and mobile
├── environment/                    # Docker, dependency and platform definitions
├── scripts/                        # Ad hoc local development scripts
├── tools/                          # Maintained, reusable engineering utilities
├── tests/                          # Cross-component and system-level tests
├── data/                           # Local recordings, results and generated files
├── docs/                           # Architecture and engineering documentation
├── AGENTS.md                       # Repository-wide agent development contract
├── CONTRIBUTING.md                 # Contribution and review process
└── compose.yaml                    # Development-container entry point
```

See [Repository layout](docs/architecture/repository-layout.md) for the complete directory contract, placement rules, and ownership boundaries.

## Development

### Python robotics core

The core is a standalone Python package, independent of ROS. Set up a virtual environment, install the package and run its package checks:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e "./robot/software/core[test,research]"
python -m pytest robot/software/core/tests
```

Interactive studies belong in `robot/software/core/notebooks/`; reusable calculations belong in `src/duk_core/`. Notebook-only implementations must not become a parallel mathematical source of truth.

### ROS 2 workspace

ROS 2 integrations live under `robot/software/ros2_ws/src/`. Build the workspace from a configured ROS environment after actual ROS package manifests have been added:

```bash
cd robot/software/ros2_ws
colcon build --symlink-install
```

The source tree intentionally retains placeholder directories for subsystem boundaries. A placeholder is not a launchable ROS package.

### Container-based workflow

The root `compose.yaml` and `.devcontainer/` provide one entry point for an isolated development environment. Select the ROS distribution through `.env` and open the repository using a Dev Container-capable IDE, or use:

```bash
cp .env.example .env
docker compose up -d dev
docker compose exec dev bash
```

Host-specific GPU, graphics, real-time and peripheral access is intentionally not implied by the generic development environment. Consult the [platform matrix](docs/platforms/compatibility.md) before enabling a hardware target.

## Engineering contracts

- Maintain a single documented source of truth for physical parameters, coordinate frames, units and robot configuration.
- Keep core modeling and control APIs independent of ROS middleware; ROS packages adapt those APIs to messages, device interfaces and lifecycle management.
- Preserve editable hardware source projects. URDF/Xacro, meshes and manufacturing exports are distinct downstream artifacts.
- Keep exploratory notebooks separate from reproducible scientific experiment definitions.
- Keep CI/CD in `.github/workflows/`, reusable utilities in `tools/`, and ad hoc helpers in `scripts/`.
- Track large recordings, logs and generated artifacts outside ordinary Git history.
- Require appropriate hardware safety reviews before executing actuator commands or deploying firmware.

## Documentation and collaboration

Start at the [documentation index](docs/README.md). Follow [engineering rules](docs/development/engineering-rules.md), [testing practices](docs/development/testing.md), [data contracts](docs/architecture/data-contracts.md) and [contribution guidelines](CONTRIBUTING.md). Agent-assisted work is governed by [AGENTS.md](AGENTS.md).

## License

The repository is distributed under the [MIT License](LICENSE), subject to any additional license terms declared for third-party dependencies and externally sourced hardware assets.
