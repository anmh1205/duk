# Development and Deployment Environments

Environment assets isolate software toolchains from the developer's host machine and organize supported build/runtime targets. They do not replace the source architecture or guarantee that every device can be accessed from a generic container.

## Configuration ownership

The root `compose.yaml` is the main development entry point. `.devcontainer/` configures IDE attachment. `docker/` contains container image definitions by purpose; `compose/` contains optional service and target-specific extensions; `dependencies/` owns future environment-level dependency definitions; `platforms/` records target verification status.

## Supported workflows

The ROS 2 development environment offers standard terminal tools and a workspace bind mount. Python core work can run directly in an ordinary Python virtual environment or in the development container. Mechanical CAD and PCB tools may remain native desktop applications and are still part of the monorepo's engineering workflow.

## Portability limits

Changing ROS distributions, CPU architecture or board support packages may require rebuilding images, Python extensions and native workspace artifacts. GPU/graphics forwarding, CAN/USB access, real-time scheduling and Jetson-specific libraries must be verified on their intended targets before being treated as supported.

See [Development environment](../docs/development/environment.md) and [Platform compatibility](../docs/platforms/compatibility.md).
