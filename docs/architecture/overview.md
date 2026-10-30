# DUK System Architecture

## System boundaries

DUK integrates a wheel-legged mechanical assembly, motor drive and electrical distribution, embedded control, a host compute platform, robot software, and research/simulation tooling. These components have different change cadences and verification methods; the monorepo provides a shared source tree without merging their responsibilities.

| Layer | Owned capability | Interfaces and dependencies |
| --- | --- | --- |
| Mechanical assembly | Geometry, joints, structural constraints | CAD and measured/calibrated physical parameters |
| Electronics | Power, sensing, actuation, connector topology | Schematics, board layout and wiring contracts |
| Embedded firmware | Motor/device I/O and time-critical device behavior | Explicit transport protocol and safety states |
| Robotics core | ROS-independent modeling, control design and numerical tools | Parameters and typed numerical inputs/outputs |
| ROS 2 integration | Lifecycle, communications, state estimation, hardware abstraction | Core API, ROS interfaces, sensor/actuator drivers |
| Simulation | Models, physics environments, robot scenarios | Robot description, parameters and integration adapters |
| Applications | Operator interaction, data presentation and external connectivity | Authenticated gateway/API or explicit ROS-facing adapter |
| Research | Hypotheses, experiments, benchmarks and publications | Reusable core and simulation/robot evidence |

## Dependency direction

Core modeling and control must not import ROS nodes, lifecycle objects, Docker configuration or UI frameworks. ROS integration and research code may depend on core; core must not depend on those consumers. Visualization is an optional extension inside the core package and is imported only by code that needs it.

The ROS runtime integrates estimator, controller, hardware abstraction and bringup packages. Message payloads and transport are integration details; state definitions, units, frame conventions and control-domain semantics must be stable and explicitly documented.

Operator applications access the robot through controlled interfaces. They must not bypass hardware safety mechanisms, device interlocks, or command authorization.

## Data and configuration

A robot configuration is a versioned and traceable input to modeling, simulation, control and deployment. Physical parameters, calibration data, derived model files and controller gains must not be silently duplicated. The [data contract](data-contracts.md) and [parameter governance](parameter-governance.md) define how consumers stay consistent.

A mathematically simplified model, a CAD assembly, and a full URDF/Gazebo model serve different purposes. Their consistency is verified through declared correspondences and tolerances, not by assuming their numerical representations are identical.

## Verification layers

Unit tests validate local mathematics and API contracts. Integration tests validate messages, models and adapters. Simulation establishes evidence under defined physics assumptions. Hardware-in-the-loop and physical-robot testing require approved limits and safety provisions. Scientific experiments additionally require reproducible conditions and uncertainty reporting.

See the [testing strategy](../development/testing.md) and [research methodology](../research/methodology.md).

## Lifecycle

Engineering changes progress through design, review, implementation, verification and controlled release. Source artifacts, export artifacts, calibration records and experiment results must retain enough metadata to reconstruct their provenance without relying on developer memory.
