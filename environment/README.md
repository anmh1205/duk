# Development environments

`compose.yaml` defines the baseline ROS development service. `.devcontainer/`
lets the IDE attach directly to the container so that ROS commands can be used
normally inside its terminal.

`docker/` owns Dockerfiles by purpose, `compose/` owns future service and
platform overrides, and `platforms/compatibility.yaml` records target status.
Do not treat a planned platform as verified.

See [environment setup](../docs/development/environment.md).
