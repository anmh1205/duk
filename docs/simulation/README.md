# Simulation and Scenario Management

Simulation provides controlled environments for kinematics, dynamic behavior, control integration and repeatable evaluation. It complements analytical verification; it does not replace model validation or physical-robot testing.

## Ownership

- `robot/simulation/worlds/`: terrain, obstacles, environment geometry and environmental properties.
- `robot/simulation/scenarios/`: initial conditions, disturbances, trajectories, terrain configurations and success criteria.
- `robot/software/ros2_ws/src/duk_gazebo/`: ROS/Gazebo plugins, bridges, launch integration and simulator adapters.
- `robot/software/ros2_ws/src/duk_description/`: robot URDF/Xacro and runtime geometry.

Avoid duplicating the same world or robot description inside two packages without a documented reason.

## Reproducibility

Each scenario must identify the simulator and physics engine versions, robot/model revision, controller configuration, initial state, timestep, contact/friction assumptions and any random seed. Log observed metrics and relevant simulation warnings.

## Validation

Compare predicted static geometry, actuator conventions, inertia, contact behavior and trajectories against the mathematical model and appropriate measurements. Document model fidelity and discrepancies rather than tuning physics parameters without provenance.

Use `research/experiments/` for formal scientific campaigns and `research/benchmarks/` for reusable evaluation suites. Store generated recordings and plots in `data/` or a controlled external store.

See [Parameter governance](../architecture/parameter-governance.md) and [Experiment protocol](../experiments/README.md).
