# Robotics Core

robot/software/core/ contains ROS-independent numerical models, visualization and interactive engineering work for DUK.

## Implemented foundation

| Area | Current scope |
| --- | --- |
| src/duk_core/modeling/leg_linkage.py | Canonical robot-frame four-bar kinematics, rigid BCO carrier, workspace, analytic Jacobian and trajectories |
| src/duk_core/modeling/mass_properties.py | Reusable planar mass-property composition for later dynamics |
| src/duk_core/visualization/leg_linkage.py | Matplotlib mechanism, trajectory, coordinate and Jacobian plots |
| examples/duk_leg_kinematics.py | Numerical inspection of the current physical geometry |
| tests/ | Geometry, topology, rigid-body, workspace, Jacobian and notebook validation |
| notebooks/ | Kinematics and motion workbenches built on the reusable core API |

The physical source is robot/config/physical/duk_leg_geometry.json.

## Canonical coordinate model

The core no longer solves in one frame and then rotates or mirrors results into another frame.

Only A = (0,0) is fixed.

Every other point is computed directly in the robot frame:

D = A + AD [cos(theta_AD), sin(theta_AD)]

B = A + AB [cos(alpha), sin(alpha)]

where theta_DA = -30°, theta_AD = 150°, and alpha is the absolute A→B angle from +Ox.

C is selected from the B-radius-BC / D-radius-CD circle intersection using the physical named topology. H and O are then derived from the rigid BCO carrier frame attached to B→C.

The config therefore describes geometry and relations, not precomputed coordinates.

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

    python robot/software/core/examples/duk_leg_kinematics.py --angle-deg 75
    python -m pytest -q robot/software/core/tests

With plots:

    python robot/software/core/examples/duk_leg_kinematics.py --angle-deg 75 --plot

The angle is the absolute A→B direction from robot +Ox.

## Notebooks

- notebooks/kinematics/duk-leg-kinematics.ipynb: single-pose geometry, workspace, trajectories and Jacobian.
- notebooks/motion/duk-leg-motion.ipynb: workspace sweep, rigid-carrier invariants, motion outputs and time-domain kinematic examples.

The motion workbench remains kinematic. Motor limits, contact dynamics, body mass/inertia and controller dynamics are not invented.
