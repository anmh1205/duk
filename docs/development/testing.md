# Verification and Testing

Verification in DUK follows the artifact being changed. A green Python CI run does not establish the correctness of a PCB, the fidelity of a simulator, or the safety of a robot controller.

## Verification levels

| Level | Owner | Evidence |
| --- | --- | --- |
| Local unit and numerical checks | `robot/software/core/tests/`, ROS package tests, firmware/tool tests | Reproducible command, test assertions, accepted tolerance |
| Cross-component integration | `tests/integration/` | Verified interfaces, message/state conventions and error handling |
| Platform compatibility | `tests/compatibility/` | Actual OS, architecture, ROS distro, toolchain and dependency combination |
| System behavior | `tests/system/` | End-to-end behavior under defined configuration and safety conditions |
| Scientific evaluation | `research/experiments/` and `research/benchmarks/` | Versioned scenario, metrics, analysis and uncertainty |
| Design review | Mechanical and electronic source projects | Revisioned CAD/PCB checks, interface and manufacturing evidence |
| Hardware qualification | Approved bench or robot test procedure | Measured electrical/mechanical behavior and safety controls |

## Robotics core

Run the package-level checks from the repository root after setting up the required Python environment:

```bash
python -m pip install -e "./robot/software/core[test]"
python -m pytest robot/software/core/tests
python -m compileall -q robot/software/core/src
```

Numerical tests should cover valid parameter ranges, boundary cases, known analytic solutions, coordinate transformations, dimensions and finiteness checks. Tests against a generic geometric example verify its helper API, not the actual DUK geometry or dynamics. New robot-specific functions require their own validated references.

## ROS 2 and simulation

For implemented ROS packages, use `colcon build` and package-native tests. Validate state-vector mapping, transforms, timestamps, QoS assumptions and lifecycle behavior between nodes. In Gazebo, record the simulator version, model revision, contact assumptions, scenario definition and numerical timestep so results can be reproduced.

A buildable package or launchable simulation is an integration milestone, not a validation of the physical robot.

## Hardware and firmware

Mechanical checks include joint travel, fit, tolerances, interference and manufacturability; electronics checks include schematic and layout design rules, power paths, protection and pin assignments. Firmware checks include interface protocol validation, watchdog/timeout behavior, safe startup and target-specific integration.

Physical-robot tests require authorization, operating limits, current or torque constraints as applicable, emergency-stop availability, instrumentation and a stop/recovery plan.

## CI policy

`.github/workflows/` is the source for GitHub Actions configuration. CI should run deterministic checks without implicit hardware access. Document what each workflow covers; do not infer multi-platform support from a single Linux runner. Test assets and fixtures should be versioned; captured datasets and large outputs should be managed separately.

## Reporting

For each validation claim, record the exact command or procedure, source revision, environment, target, pass/fail result and limitations. If verification was not run, state that explicitly in the contribution rather than reporting an assumed success.

See [Research methodology](../research/methodology.md) and [Platform compatibility](../platforms/compatibility.md).
