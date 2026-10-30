# Applications and External Interfaces

The `applications/` area contains products that interact with the robot but do not own the robot's safety-critical motion control.

## Subsystem roles

**Operator dashboard** displays robot state, operating mode, battery/power information, telemetry health, alarms and command acknowledgment. Its controls must reflect the actual command-authorization and safety model.

**Gateway** defines authenticated external APIs, event/telemetry delivery, versioned payloads and integration with other clients. It isolates applications from internal ROS message and node implementation details where appropriate.

**Mobile** is an optional client of the gateway and supported operator functions. Client-side state must not be treated as the authoritative robot state.

## Interface requirements

Specify authentication and authorization boundaries, command scopes, rate limits, request/acknowledgment semantics, timestamps and reconnection/failure behavior. Telemetry consumers must distinguish sensor measurement time from client receipt time.

No application should bypass the robot's hardware safety limits or write directly to motor buses. Separate remote operator intent from low-level actuation.

## Development and verification

Implement application components only when their contracts and target platforms are defined. Test schema compatibility, access controls, stale telemetry, offline behavior and error handling. Keep application test fixtures free of real credentials or personal data.

See [Data contracts](../architecture/data-contracts.md) and [ROS 2 integration](../software/ros2.md).
