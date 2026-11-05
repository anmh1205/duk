# DUK Robot

`robot/` contains the design and implementation areas belonging to the physical DUK robot.

| Directory | Scope |
| --- | --- |
| `hardware/mechanical/` | Source CAD, mechanical drawings and manufacturing exports |
| `hardware/electronics/` | PCB projects, system power, wiring and component libraries |
| `software/core/` | Models, algorithms, visualizations and notebooks |
| `software/ros2_ws/` | ROS 2 integration |
| `software/firmware/` | Embedded software |
| `simulation/` | Simulation worlds and scenarios |
| `config/` | Physical, controller and calibration data |

The directories reserve engineering ownership; their contents determine which subsystems are implemented.

`duk_description` and `duk_gazebo` now implement an opt-in
[static Gazebo/RViz preview](../docs/development/simulation-preview.md) using the
canonical leg geometry. This establishes rendering and geometry integration,
not robot dynamics, controllers or hardware support.

[Repository layout](../docs/architecture/repository-layout.md) · [Hardware documents](../docs/hardware/README.md) · [Software documents](../docs/software/README.md).
