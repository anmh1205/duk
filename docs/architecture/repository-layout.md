# Repository Layout

This document defines the agreed directory ownership of the DUK monorepo. It is a **folder and artifact-placement reference**, not a robot system design, a list of adopted algorithms, or a statement that the described subsystems are operational.

## Project tree

```text
duk/
├── README.md                           # Repository entry and navigation
├── AGENTS.md                           # Root agent-instruction framework
├── CONTRIBUTING.md                     # Contribution documentation (reserved)
├── LICENSE                             # Repository license
├── compose.yaml                        # Existing Compose entry point
├── .env.example                        # Example environment variables
├── .gitignore                          # Git exclusions
├── .dockerignore                       # Docker build-context exclusions
├── .editorconfig                       # Editor settings
├── .devcontainer/                      # Development-container configuration
│   └── devcontainer.json
├── .github/                            # GitHub collaboration and CI
│   ├── workflows/                      # Workflow definitions
│   └── ISSUE_TEMPLATE/                 # Issue templates
│
├── robot/                              # Robot-specific engineering assets
│   ├── README.md
│   ├── AGENTS.md
│   ├── hardware/
│   │   ├── mechanical/                 # Mechanical engineering
│   │   │   ├── cad/
│   │   │   │   ├── assemblies/         # Editable assembly sources
│   │   │   │   └── parts/              # Editable part sources
│   │   │   ├── drawings/               # Engineering drawings
│   │   │   └── manufacturing/          # Fabrication exports
│   │   └── electronics/                # Electrical and electronic engineering
│   │       ├── boards/                 # Board design projects
│   │       ├── system/
│   │       │   ├── power/              # System power design
│   │       │   └── wiring/             # System wiring design
│   │       └── libraries/              # Shared electronic design libraries
│   ├── software/
│   │   ├── core/                       # ROS-independent Python robotics library
│   │   │   ├── README.md
│   │   │   ├── pyproject.toml          # Python package configuration
│   │   │   ├── src/duk_core/
│   │   │   │   ├── modeling/           # Mathematical models
│   │   │   │   ├── control/            # Control algorithms
│   │   │   │   └── visualization/      # Reusable plotting and animation
│   │   │   ├── notebooks/
│   │   │   │   ├── kinematics/         # Interactive kinematics
│   │   │   │   ├── dynamics/           # Interactive dynamics
│   │   │   │   ├── control/            # Interactive control exploration
│   │   │   │   └── motion/             # Interactive motion studies
│   │   │   ├── examples/               # Executable API examples
│   │   │   └── tests/                  # Core package tests
│   │   ├── ros2_ws/
│   │   │   └── src/                    # ROS 2 package locations
│   │   │       ├── duk_description/    # Robot description
│   │   │       ├── duk_interfaces/     # Custom ROS interfaces
│   │   │       ├── duk_control/        # ROS control integration
│   │   │       ├── duk_estimation/     # State estimation
│   │   │       ├── duk_hardware/       # Hardware interface
│   │   │       ├── duk_gazebo/         # Gazebo integration
│   │   │       └── duk_bringup/        # ROS bringup
│   │   └── firmware/                   # Embedded software
│   ├── simulation/
│   │   ├── worlds/                     # Simulation environments
│   │   └── scenarios/                  # Simulation scenarios
│   └── config/
│       ├── physical/                   # Physical robot parameters
│       ├── controllers/                # Controller configuration
│       └── calibration/                # Calibration records
│
├── research/
│   ├── README.md
│   ├── AGENTS.md
│   ├── experiments/                    # Reproducible scientific studies
│   ├── benchmarks/
│   │   ├── balancing/                  # Balance evaluation
│   │   └── trajectory-tracking/        # Trajectory evaluation
│   └── publications/                   # Scientific manuscripts
│
├── applications/
│   ├── README.md
│   ├── operator-dashboard/             # Operator interface
│   ├── gateway/                        # External connectivity
│   └── mobile/                         # Mobile application
│
├── environment/
│   ├── README.md
│   ├── AGENTS.md
│   ├── docker/
│   │   ├── ros/                        # Existing ROS image definition
│   │   ├── research/                   # Reserved research environment
│   │   ├── applications/               # Reserved application image area
│   │   └── jetson/                     # Reserved Jetson image area
│   ├── compose/                        # Compose extensions
│   ├── platforms/                      # Platform declarations
│   └── dependencies/                   # Environment dependencies
│
├── scripts/                            # Ad hoc developer scripts
├── tools/                              # Maintained engineering utilities
│   ├── model-generation/
│   ├── validation/
│   └── environment-check/
├── tests/                              # Cross-component verification
│   ├── integration/
│   ├── compatibility/
│   └── system/
├── data/                               # Local records and generated results
│   ├── recordings/
│   ├── results/
│   └── generated/
└── docs/                               # Full engineering documentation scope
```

Git preserves intentional empty directories using `.gitkeep`. The tree omits those marker files and individual implementation files unless their names clarify a structural boundary.

## Documentation tree

The documentation hierarchy is reserved at full project scope. Only the directory layout and navigation have been agreed; technical pages are placeholders until their respective topics have been decided.

```text
docs/
├── README.md                           # Documentation index
├── architecture/
│   ├── README.md                       # Architecture navigation
│   ├── repository-layout.md            # Established folder architecture
│   ├── overview.md                     # Reserved: system architecture
│   ├── runtime-architecture.md         # Reserved: runtime architecture
│   ├── data-contracts.md               # Reserved: cross-component data contracts
│   ├── parameter-governance.md         # Reserved: parameter lifecycle
│   └── engineering-workflows.md        # Reserved: cross-discipline workflows
├── hardware/
│   ├── README.md                       # Hardware navigation
│   ├── mechanical.md                   # Reserved: mechanical engineering
│   ├── electronics.md                  # Reserved: electronics engineering
│   ├── manufacturing.md                # Reserved: fabrication
│   ├── assembly.md                     # Reserved: assembly procedures
│   └── hardware-validation.md          # Reserved: hardware verification
├── software/
│   ├── README.md                       # Software navigation
│   ├── core.md                         # Reserved: robotics core specification
│   ├── ros2.md                         # Reserved: ROS 2 integration
│   ├── firmware.md                     # Reserved: embedded software
│   └── diagnostics.md                  # Reserved: diagnostics
├── theory/
│   ├── README.md                       # Theory navigation
│   ├── modeling.md                     # Reserved: mathematical modeling
│   └── control.md                      # Reserved: control theory
├── simulation/
│   ├── README.md                       # Simulation navigation
│   ├── design.md                       # Reserved: simulation design
│   └── validation.md                   # Reserved: simulation validation
├── research/
│   ├── README.md                       # Research navigation
│   ├── methodology.md                  # Reserved: research methodology
│   └── benchmarking.md                 # Reserved: benchmark definition
├── experiments/
│   ├── README.md                       # Experiment navigation
│   └── protocols.md                    # Reserved: experiment protocols
├── applications/
│   ├── README.md                       # Application navigation
│   └── architecture.md                 # Reserved: application architecture
├── development/
│   ├── README.md                       # Development navigation
│   ├── getting-started.md              # Reserved: onboarding
│   ├── environment.md                  # Reserved: environment instructions
│   ├── engineering-rules.md            # Reserved: engineering practices
│   ├── testing.md                      # Reserved: testing procedures
│   ├── documentation.md                # Reserved: documentation policy
│   └── troubleshooting.md              # Reserved: troubleshooting
├── operations/
│   ├── README.md                       # Operations navigation
│   ├── safety.md                       # Reserved: operational safety
│   ├── commissioning.md                # Reserved: commissioning
│   ├── maintenance.md                  # Reserved: maintenance
│   └── release.md                      # Reserved: delivery and release
├── platforms/
│   ├── README.md                       # Platform navigation
│   └── compatibility.md                # Reserved: compatibility verification
└── decisions/
    └── README.md                       # Decision-record navigation
```

## Ownership boundaries

The following boundaries describe **where work belongs**, not how undecided subsystems must be implemented.

- **Hardware design:** editable mechanical CAD belongs in `robot/hardware/mechanical/cad/`; drawings and manufacturing exports are separated. PCB projects belong in `robot/hardware/electronics/boards/`, while system-level power and wiring reside alongside them in `electronics/system/`.
- **Robotics core:** reusable mathematical and control code belongs in `robot/software/core/src/duk_core/`. Notebooks are for interactive investigation; shared visualization helpers reside with core. This avoids a duplicate `analysis/` or `workbench/` subsystem.
- **ROS integration:** middleware-specific source belongs in `robot/software/ros2_ws/`. Its package folders reserve the named integration areas, but do not constitute functioning ROS packages on their own.
- **Simulation:** general worlds and scenarios belong in `robot/simulation/`, whereas ROS-to-Gazebo integration belongs in `duk_gazebo/`.
- **Research:** reproducible scientific studies and benchmarks belong in `research/`, distinct from exploratory core notebooks and package unit tests.
- **Applications:** the dashboard, gateway and mobile application own external interaction, not numerical modeling or the ROS package layout.
- **Development infrastructure:** `.github/` owns GitHub workflows, `tools/` owns maintained reusable engineering utilities, and `scripts/` is for ad hoc developer scripts.
- **Verification and data:** component tests live near their implementations; cross-component tests live in root `tests/`. Recordings, computed results and generated outputs have dedicated locations under `data/`.

The [Documentation Index](../README.md) is the navigation entry point. Future technical documents are populated only when the relevant engineering work establishes their contents.
