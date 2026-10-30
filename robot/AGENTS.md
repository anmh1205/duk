# Robot-specific guidance

Read root [AGENTS.md](../AGENTS.md) first.

- Keep editable CAD / PCB sources in `hardware/`, and runtime URDF/Xacro in `software/ros2_ws/src/duk_description/`.
- Keep modeling and control independent of ROS interfaces wherever possible.
- Record units, coordinate frames and the provenance of physical/calibration parameters.
- Do not treat empty ROS directories as buildable packages.
- Do not actuate hardware, flash firmware or run unsafe hardware tests without explicit permission.
