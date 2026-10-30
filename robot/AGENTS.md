# Robot Engineering Agent Rules

Read [repository instructions](../AGENTS.md), [robot overview](README.md) and the relevant hardware, software or simulation document before modifying any robot-owned asset.

Preserve discipline boundaries:

- Source CAD and PCB design files belong in `hardware/`. Keep URDF/Xacro and runtime meshes in the ROS description package.
- Canonical modeling/control code belongs in `software/core/`. ROS packages consume its API and own only integration behavior.
- Physical parameters require units, frames, source revision and calibration provenance. Never invent real DUK dimensions, inertias, pinouts or gains.
- Model and actuator interfaces must retain defined state ordering, constraints and failure semantics.
- Empty ROS package directories are placement reservations, not valid executable packages.

Do not send physical actuator commands, flash firmware, connect a robot to an unsafe controller or bypass a hardware limit without specific authorization and an approved test procedure. Update affected integration tests and technical documentation with every interface change.
