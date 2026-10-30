# Repository Structure and Ownership

## Directory map

```text
duk/
├── .devcontainer/                        # IDE entry into the development environment
├── .github/
│   ├── workflows/                       # CI/CD definitions
│   └── ISSUE_TEMPLATE/                  # Issue templates
├── robot/
│   ├── hardware/
│   │   ├── mechanical/
│   │   │   ├── cad/assemblies/           # Editable mechanical assemblies
│   │   │   ├── cad/parts/                # Editable mechanical parts
│   │   │   ├── drawings/                 # Dimensioned drawings
│   │   │   └── manufacturing/            # Controlled manufacturing exports
│   │   └── electronics/
│   │       ├── boards/                   # Per-PCB design projects
│   │       ├── system/power/             # System-level power architecture
│   │       ├── system/wiring/            # Harnesses, connectors and pinouts
│   │       └── libraries/                # Shared component CAD libraries
│   ├── software/
│   │   ├── core/
│   │   │   ├── src/duk_core/modeling/     # Mathematical models
│   │   │   ├── src/duk_core/control/      # Reusable control algorithms
│   │   │   ├── src/duk_core/visualization/# Optional reusable plots and animation
│   │   │   ├── notebooks/                # Interactive model and algorithm exploration
│   │   │   ├── examples/                 # Executable library usage examples
│   │   │   └── tests/                    # Core-specific unit and API tests
│   │   ├── ros2_ws/src/
│   │   │   ├── duk_description/          # URDF/Xacro, meshes and robot frames
│   │   │   ├── duk_interfaces/           # Custom messages, actions and services
│   │   │   ├── duk_control/              # ROS-facing control integration
│   │   │   ├── duk_estimation/           # State estimation
│   │   │   ├── duk_hardware/             # Device and hardware interfaces
│   │   │   ├── duk_gazebo/               # Gazebo–ROS integration
│   │   │   └── duk_bringup/              # Launch and runtime orchestration
│   │   └── firmware/                     # Embedded firmware projects
│   ├── simulation/
│   │   ├── worlds/                       # Physics environments and terrain
│   │   └── scenarios/                    # Defined simulation test conditions
│   └── config/
│       ├── physical/                     # Versioned nominal physical parameters
│       ├── controllers/                  # Controller gains and limits
│       └── calibration/                  # Device/robot-specific calibration
├── research/
│   ├── experiments/                      # Reproducible studies
│   ├── benchmarks/                       # Comparable evaluation scenarios
│   └── publications/                     # Papers, theses and manuscripts
├── applications/
│   ├── operator-dashboard/              # Operational monitoring and controls
│   ├── gateway/                         # External APIs and telemetry
│   └── mobile/                          # Mobile client
├── environment/
│   ├── docker/                           # Development and runtime image definitions
│   ├── compose/                          # Optional Compose extensions
│   ├── dependencies/                     # Environment dependency definitions
│   └── platforms/                        # Supported-target matrix
├── scripts/                              # Local, ad hoc development helpers only
├── tools/                                # Maintained engineering tools
├── docs/                                 # Technical source documentation
├── tests/                                # Integration, compatibility and system tests
├── data/                                 # Ignored local outputs and recordings
├── compose.yaml                          # Compose entry point
├── README.md                             # Project entry point
└── AGENTS.md                             # Global agent instructions
```

## Placement rules

| Artifact | Canonical owner | Not owned by |
| --- | --- | --- |
| Reusable kinematics/dynamics/control functions | `robot/software/core/src/duk_core/` | Notebook, ROS node, experiment runner |
| Interactive charts and exploratory numerical trials | `robot/software/core/notebooks/` | Root `scripts/` or a second workbench |
| Shared drawing/plotting utilities | `robot/software/core/src/duk_core/visualization/` | Applications or research copy |
| Reproducible benchmark definition and metrics | `research/benchmarks/` | Core unit tests |
| Parameterized scientific study | `research/experiments/` | One-off notebooks |
| Source mechanical CAD | `robot/hardware/mechanical/cad/` | URDF/mesh exports |
| Source PCB design | `robot/hardware/electronics/boards/` | System wiring diagrams |
| ROS robot description and integration | `duk_description/`, `duk_gazebo/` | Core mathematical model |
| GitHub automation | `.github/workflows/` | `scripts/` |
| Maintained conversion/validation utilities | `tools/` | `scripts/` |
| Local recordings and generated artifacts | `data/` | Git source history |
| Cross-module tests | `tests/` | Arbitrary package source folders |

## Repository discipline

Git does not track empty directories; `.gitkeep` retains agreed placeholder boundaries. Remove it when the folder receives a real file. Do not create manifest-only ROS packages, fabricated models or dummy code just to populate the tree.

Large rosbag recordings, videos, build outputs, machine-specific secrets and temporary plots do not belong in normal commits. Version-control authoritative engineering assets and reproducible experiment definitions; manage large derived output and controlled exports through an explicit artifact policy.

Related contracts: [System architecture](overview.md), [Data contracts](data-contracts.md), [Parameter governance](parameter-governance.md) and [Engineering rules](../development/engineering-rules.md).
