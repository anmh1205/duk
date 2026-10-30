# DUK Applications

Application projects provide human and external-system access to DUK while keeping motion control and hardware safety inside the robot's control architecture.

The `operator-dashboard/` area owns real-time robot monitoring and authorized operator interaction. `gateway/` owns external API, telemetry distribution and access-policy enforcement. `mobile/` contains client applications sharing the supported gateway contracts.

## Interface contract

Applications should consume versioned state and command interfaces with explicit units, frame semantics, timestamps, authorization requirements and errors. The gateway translates between application protocols and the appropriate robot-facing transport. It must not expose raw motor-bus operations or bypass local safety interlocks.

The operator experience should differentiate sensor measurement time from UI receipt time and distinguish requested commands, acknowledged commands and actual robot states. Offline connectivity, stale data and recovery paths are part of the interface design.

## Integration

Application data contracts are governed by [Data contracts](../docs/architecture/data-contracts.md). The ROS and hardware layer remains responsible for state estimation, actuation and runtime control; applications are not the source of mathematical models or controller gains.

See [Application architecture](../docs/applications/README.md) for detailed subsystem responsibilities.
