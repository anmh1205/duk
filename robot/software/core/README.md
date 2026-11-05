# Robotics Core

`robot/software/core/` contains ROS-independent numerical models, visualization and interactive engineering work for DUK.

## Implemented foundation

| Area | Current scope |
| --- | --- |
| `src/duk_core/modeling/leg_linkage.py` | Canonical robot-frame four-bar kinematics, rigid BCO carrier, workspace, analytic Jacobian and trajectories |
| `src/duk_core/modeling/mass_properties.py` | Reusable planar mass-property composition for later dynamics |
| `src/duk_core/visualization/leg_linkage.py` | Matplotlib mechanism, trajectory, coordinate and Jacobian plots |
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

With plots:

```bash
python robot/software/core/examples/duk_leg_kinematics.py --angle-deg 75 --plot
```

## Notebooks

- `notebooks/kinematics/duk-leg-kinematics.ipynb`: equations, one-pose geometry, workspace, trajectories and Jacobian.
- `notebooks/motion/duk-leg-motion.ipynb`: workspace sweep, rigid-carrier invariants, motion outputs and time-domain kinematic examples.

The motion workbench remains kinematic. Motor limits, contact dynamics, body mass/inertia and controller dynamics are not invented.
