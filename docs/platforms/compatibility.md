# Platform Compatibility

Platform compatibility in DUK is a verified property of a specific combination of hardware, operating system, middleware and dependency versions. It is not inferred from a Dockerfile, a successful Python unit test or an upstream package's nominal architecture list.

The source declaration is [environment/platforms/compatibility.yaml](../../environment/platforms/compatibility.yaml). Keep its entries synchronized with actual build and runtime evidence.

## Compatibility dimensions

| Dimension | Items to record |
| --- | --- |
| Host | OS release, kernel characteristics, CPU architecture and Docker/Compose versions |
| ROS | Distribution, underlying image and package/ABI versions |
| Python | Interpreter, numerical package versions, binary-extension compatibility |
| Simulation | Gazebo version, physics engine, graphical driver and GPU capability |
| Embedded target | Board model, operating system, JetPack/L4T or equivalent BSP |
| Peripherals | USB, CAN, camera, IMU and motor transport/driver behavior |
| Timing | Control frequency, worst-case loop latency, jitter and deadline misses |

## Status model

- `unconfigured`: a target has no defined toolchain/image and validation procedure.
- `unverified`: intended configuration exists, but qualifying build/runtime evidence is missing.
- `verified`: the exact target combination passed documented tests relevant to the claimed capability.
- `unsupported`: explicitly excluded from the support policy, with the constraint recorded.

A verified Python calculation does not verify a ROS controller, graphics stack, CAN interface or Jetson accelerator. Validation should be scoped to each claimed capability.

## Development and deployment

The generic development container uses ROS 2 tooling without privileged hardware access. Architecture-specific compiled modules, ROS workspaces and Python binary extensions must be rebuilt for compatible targets. Jetson GPU access and drivers depend on the installed JetPack/L4T environment and are not portable by changing only the Docker base image.

## Evidence management

For every verified combination, retain the source commit, configuration, CI/build link or captured test report, executed commands, affected capabilities and known limitations. Update the source matrix and this guide when a platform's supported scope changes.
