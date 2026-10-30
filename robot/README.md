# DUK Robot Engineering

The `robot/` directory contains the authoritative engineering assets for the DUK physical robot and its associated software. Its scope includes structural and electronic design, a numerical robotics core, ROS 2 integration, embedded firmware, simulation and robot-specific configuration.

## Hardware

Mechanical source models, assemblies, drawings and manufacturing packages live in `hardware/mechanical/`. PCB projects, component libraries, power architecture and wiring live in `hardware/electronics/`. These artifacts carry their own revisions and discipline-specific validation; software-generated meshes and derived descriptions are not substitutes for original design projects.

Refer to [Mechanical design](../docs/hardware/mechanical.md) and [Electronics](../docs/hardware/electronics.md).

## Software

The reusable numerical library `software/core/` provides ROS-independent algorithms and interactive exploration tools. ROS packages inside `software/ros2_ws/` handle middleware, state estimation, hardware abstraction, simulation integration and runtime composition. Embedded targets belong in `software/firmware/`.

Canonical mathematical functions are implemented once; notebooks, simulation experiments and ROS integrations consume those APIs through defined contracts. See [Core architecture](../docs/software/core.md), [ROS 2 integration](../docs/software/ros2.md) and [Firmware](../docs/software/firmware.md).

## Simulation and physical parameters

`simulation/` holds worlds and scenario definitions. `config/` holds reviewed nominal physical parameters, controller configuration and calibration data. Model transforms, physical constants and controller gains must be traceable to engineering evidence, not inferred from mesh geometry or copied between packages without checks.

See [Simulation](../docs/simulation/README.md), [Parameter governance](../docs/architecture/parameter-governance.md) and [Data contracts](../docs/architecture/data-contracts.md).

## Integration expectations

Changes affecting joint naming, coordinate frames, connectors, actuator command semantics or calibration require a cross-disciplinary review. Physical actuation and firmware deployment require explicit authorization and a suitable safety procedure.
