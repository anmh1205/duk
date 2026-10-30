# ROS 2 Integration Architecture

## Workspace

The ROS workspace is `robot/software/ros2_ws/`, with source packages under `src/`. This workspace is the integration layer for ROS 2 transports, lifecycle, hardware interfaces and simulation. It is not the canonical implementation of robot equations and controller design.

## Package responsibilities

| Package | Scope |
| --- | --- |
| `duk_description` | URDF/Xacro, link/joint/frame naming and runtime meshes |
| `duk_interfaces` | DUK-specific messages, services and actions when standard messages are insufficient |
| `duk_control` | ROS-facing control execution and controller lifecycle integration |
| `duk_estimation` | Sensor fusion and state estimation |
| `duk_hardware` | Device communication and ROS hardware abstraction |
| `duk_gazebo` | Gazebo integration, simulator plugins and bridges |
| `duk_bringup` | Launch composition, configuration selection and startup orchestration |

Keep interfaces stable and avoid circular dependencies. A package directory becomes an actual ROS package only when its manifest, build metadata and source/test structure are implemented.

## Runtime data flow

Sensors and actuator interfaces provide observations. Estimation produces a well-defined state with timestamps and reference frames. Controllers consume the state and reference commands, enforce command constraints and send actuator requests through the hardware abstraction. Simulation can replace observation and actuation sources without changing the mathematical meaning of the state.

Neither ROS transport choice nor Gazebo implementation details should leak into pure modeling and control-design functions.

## Configuration and launch

Launch files belong in `duk_bringup` or a package that directly owns a specific feature. Controller parameters and calibration should come from reviewed `robot/config/` datasets and remain traceable to hardware and model revisions.

Document QoS, communication rates, frame conventions, fault handling and startup dependencies when they affect runtime behavior.

## Build and verification

Use `colcon` for workspace builds and ROS-native tests for implemented packages. A valid build does not demonstrate that simulated dynamics, message semantics or hardware timing meet the robot's requirements. Add end-to-end tests when cross-package behavior is introduced.

See [System architecture](../architecture/overview.md), [Data contracts](../architecture/data-contracts.md) and [Simulation](../simulation/README.md).
