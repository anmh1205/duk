# Environment Agent Rules

Read [repository instructions](../AGENTS.md) and [environment documentation](../docs/development/environment.md) before editing Docker, Compose, platform or dependency definitions.

Maintain the root `compose.yaml` as the primary development entry point. Use narrow, explicit target overrides for hardware devices, GPU access or special runtime privileges. Do not introduce environment-specific paths into the robotics-core API.

Verify changed container and build commands against the actual target. A container build, ROS command or Python check does not establish board-level, GPU or real-time compatibility. Keep target verification records aligned with `platforms/compatibility.yaml`; do not label a platform supported without evidence.

Never commit secrets or machine-specific credentials. Avoid unnecessary dependency upgrades or generated caches as part of documentation-oriented changes.
