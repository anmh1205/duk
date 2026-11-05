# Robotics Core

`robot/software/core/` contains ROS-independent numerical models, visualization and interactive engineering work for DUK.

## Implemented foundation

| Area | Current scope |
| --- | --- |
| `src/duk_core/modeling/leg_linkage.py` | Canonical robot-frame four-bar kinematics, rigid BCO carrier, workspace, analytic Jacobian and trajectories |
| `src/duk_core/modeling/mass_properties.py` | Reusable planar mass-property composition for later dynamics |
| `src/duk_core/visualization/leg_linkage.py` | Static Matplotlib mechanism, trajectory, coordinate and Jacobian plots |
| `src/duk_core/visualization/leg_animation.py` | Real-time single-pose animation built on the canonical solver; no duplicated robot equations |
| `examples/duk_leg_kinematics.py` | Numerical inspection of current physical geometry |
| `tests/` | Equation, topology, rigid-body, workspace, Jacobian and notebook validation |
| `notebooks/` | Kinematics and motion workbenches built on the reusable core API |

The physical source is `robot/config/physical/duk_leg_geometry.json`.

## Canonical coordinate model

Only A is fixed:

[
A=(0,0).
]

Every other point is derived from dimensions, directed angles and topology.

The fixed-body direction is defined by:

[
	heta_{DA}=-30^circ,
qquad
	heta_{AD}=	heta_{DA}+180^circ.
]

The linkage coordinate is:

[
q=angle(overrightarrow{AB},overrightarrow{DA}),
]

so:

[
	heta_{AB}=	heta_{DA}-q.
]

Then:

[
D=A+AD,u(	heta_{AD}),
qquad
B=A+AB,u(	heta_{AB}).
]

C is the physical B-radius-BC / D-radius-CD circle intersection selected by named left/right topology. H and O are derived from the moving rigid BCO basis attached to B→C.

No fixed B/C/D/H/O coordinates are part of the model, and no mirror/rotation correction is applied after solving.

## Rigid BCO carrier

B, C and O are one rigid mechanical link. The model enforces:

- BC constant;
- BO constant;
- CO constant;
- signed B-C-O area/orientation constant;
- rigid-body velocity constraints.

H is an auxiliary construction point only.

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
