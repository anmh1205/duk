# Data and Interface Contracts

## Scope

Every subsystem that consumes robot state, commands or physical parameters must agree on units, reference frames, signal ordering, timestamp semantics and error behavior. Source code should not infer these from file paths or implicit conventions.

## State and command definitions

Each state vector or command message requires a documented schema:

- field name, physical meaning, shape, unit and sign convention;
- reference frame and coordinate-frame orientation;
- state ordering, including pose representation and orientation convention;
- sampling timestamp, clock domain and validity/staleness rules;
- permitted range, missing data representation and handling of invalid values;
- producer, consumers and ownership of the schema.

Numerical arrays without a declared ordering are not cross-subsystem APIs. A controller, estimator and experiment must not independently choose different positions for the same state variable.

## Coordinate frames

Robot frames should follow a documented, consistently applied convention aligned with ROS REP-103 and REP-105 when interacting with ROS. Define the body frame, base/reference frame, wheel and joint frames, odometry and world frames before publishing transforms or deriving gravity-based controllers.

Frame changes must be explicit transforms. Distinguish positions from vectors, rotations from Euler-angle coordinates, and relative joint coordinates from absolute link orientations.

## Time and sampling

Define whether timestamps represent measurement time, receipt time, state-estimation time or actuation deadline. Document sampling rates and expected latency, and avoid comparing logs from different clocks without synchronization.

Discrete-time controller parameters must identify sample period and discretization method. Continuous-time models and discrete-time controllers cannot share gain matrices without checking the underlying design assumptions.

## Errors and compatibility

Public APIs must validate dimension, finiteness and unit assumptions at appropriate boundaries. A schema modification that changes meaning, ordering or command behavior requires updated consumers, tests and versioned documentation.

## Traceability

Configuration and experiment records should identify robot/model revision, parameter set, source commit, software/toolchain versions and calibration source. See [Parameter governance](parameter-governance.md) and [Experiment protocol](../experiments/README.md).
