# Robotics Core

`robot/software/core/` contains ROS-independent numerical models, visualization and interactive engineering work for DUK.

## Implemented foundation

| Area | Current scope |
| --- | --- |
| `src/duk_core/modeling/leg_linkage.py` | Canonical robot-frame four-bar kinematics, rigid BCO carrier, workspace, analytic Jacobian and trajectories |
| `src/duk_core/modeling/leg_differential.py` | Analytic second derivatives, time acceleration and physical-branch inverse wheel position |
| `src/duk_core/modeling/planar_biped.py` | Candidate planar mass matrix, convective/gravity/damping forces, wheel reactions and optional contact constraints with unidentified symbols |
| `src/duk_core/modeling/mass_properties.py` | Reusable planar mass-property composition for later dynamics |
| `src/duk_core/visualization/leg_linkage.py` | Static Matplotlib mechanism, trajectory, coordinate and Jacobian plots |
| `src/duk_core/visualization/leg_animation.py` | Real-time single-pose animation built on the canonical solver; no duplicated robot equations |
| `examples/duk_leg_kinematics.py` | Numerical inspection of current physical geometry |
| `tests/` | Equation, topology, rigid-body, workspace, Jacobian and notebook validation |
| `notebooks/` | Kinematics and motion workbenches built on the reusable core API |

The physical source is `robot/config/physical/duk_leg_geometry.json`.

## Canonical model and parameterized dynamics

Only A is fixed at (0,0) in the leg frame. +x points right, +y up, and CCW is
positive. Directed D→A is −30° and q is the directed angle AB→DA, so
θ_AB = θ_DA − q. B/C/D/H/O are derived from the canonical geometry; B/C/O form
one rigid carrier and H is an auxiliary point. No display-time correction is used.

The [modeling reference](../../../docs/theory/modeling.md) derives velocity,
acceleration, inverse wheel position and the candidate planar 7-DOF biped
Lagrangian. The biped model uses SI and leaves all 43 unknown physical quantities
symbolic. It does not identify real robot dynamics, choose a contact mode or
configure the static Gazebo preview as a dynamic robot.

Install `duk-core[symbolic]` for symbolic work (already available in the research
container). The core numerical geometry remains usable without SymPy or ROS.

## Run

Inside the development container:

```bash
python robot/software/core/examples/duk_leg_kinematics.py --angle-deg 75
python -m pytest -q robot/software/core/tests
```

Here `--angle-deg` is (q), not an absolute A→B world angle.

With static analytical plots:

```bash
python robot/software/core/examples/duk_leg_kinematics.py --angle-deg 75 --plot
```

For a live MATLAB-like mechanism animation:

```bash
python robot/software/core/examples/duk_leg_motion.py
```

Optional wheel-center trace:

```bash
python robot/software/core/examples/duk_leg_motion.py --trace
```

The live animation updates one mechanism pose at a time. It does not overlay a stack of historical poses unless an explicit trace is requested.

Inspect the symbolic model without inventing measurements:

```bash
python robot/software/core/examples/duk_planar_model.py
```

Use `--parameters .local/planar-parameters.json` to bind a partial set of identified
values; unknown/null values stay symbolic. See the [parameter template](../../config/physical/duk_planar_parameters.example.json).

## Notebooks

- `notebooks/kinematics/duk-leg-kinematics.ipynb`: equations, one-pose geometry, workspace, trajectories and Jacobian.
- `notebooks/motion/duk-leg-motion.ipynb`: interactive play/pause/scrub animation, rigid-carrier invariants, motion signals and workspace analysis.

The motion workbench remains kinematic. Motor limits, contact dynamics, body mass/inertia and controller dynamics are not invented.


## Code organization rules

The kinematic stack is intentionally separated by responsibility:

1. **Modeling** owns equations, topology and invariants.
2. **Visualization** consumes solved poses; it must not duplicate geometry equations.
3. **Animation** updates Matplotlib artists from solved poses; it does not alter the robot model.
4. **Notebooks** compose reusable APIs for investigation; equations that belong to the robot model must be moved into `duk_core` rather than copied into cells.

Comments should explain physical meaning, mathematical intent and non-obvious constraints. They should not narrate trivial Python syntax line by line.
