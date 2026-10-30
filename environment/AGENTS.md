# Environment-specific guidance

Read root [AGENTS.md](../AGENTS.md) and
[environment guide](../docs/development/environment.md).

- `compose.yaml` is the canonical Compose entry point.
- Do not introduce ad hoc Docker commands into robotics core API or notebooks.
- Do not claim ROS distro, Gazebo, GUI, GPU, Jetson or device support without
  executing a test on the relevant target.
- Image, Python ABI and ROS build caches must not leak across incompatible targets.
- Do not enable privileged/hardware access for the generic development container.
