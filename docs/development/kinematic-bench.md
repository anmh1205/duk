# Two-leg RViz kinematic bench

This preview keeps both tires in geometric contact with flat ground at world
z=0. Two independent canonical leg coordinates drive the mechanism; one common
rigid body transform supplies heave and lateral roll. It does not run Gazebo,
simulate contact forces, select a controller, or send motor commands. Unknown
physical parameters remain untouched.

## Run

From the host repository root, using the existing simulation environment:

```bash
python3 tools/dev/duk-dev.py configure --preset vmware --gpu software
python3 tools/dev/duk-dev.py up
python3 tools/dev/duk-dev.py kinematics
```

The last command opens RViz and a Qt control/plot panel. `demo` still opens the
separate static Gazebo/RViz preview. Both share an exclusive launch/build lock;
stop the current preview before switching:

```bash
python3 tools/dev/duk-dev.py stop
```

After the dependency image has been built, `up --no-build` reuses it for source,
documentation or runtime-flag changes. Changes to image dependencies still need
`build` or the default `up` behavior.

In a simulation Dev Container terminal, after building `duk_description` and
sourcing ROS plus `/opt/duk-ros/install/setup.bash`, the equivalent command is:

```bash
ros2 launch duk_description kinematics.launch.py
```

For ROS/CI checks without windows:

```bash
python3 tools/dev/duk-dev.py kinematics --headless
```

## Controls and signals

- Set qL/qR independently with the sliders or numeric angle fields.
- **Chạy tự động** starts a bounded cosine inspection profile; **Tạm dừng**
  freezes the geometric pose. Resuming starts from that pose without a jump.
- **Về 75°** restores the reference pose and clears the plot history.
- Speed scales the nominal ten-second profile between 0.25× and 2×.
- The table shows profile q/rates, wheel position/velocity/acceleration, loop
  closure error, BCO length and signed-area errors, distance to a geometric
  boundary, and the normalized loop determinant magnitude.
- Three plots show body-frame wheel X/Z position and calculated relative derivatives;
  dashed lines are X, solid lines are Z, orange is left and cyan is right.

The selectable interval excludes five degrees at each geometric endpoint. That
reserve, nominal period, speed range and warning threshold are visualization
choices, **not identified motor or mechanical operating limits**. Requests
outside the interval are rejected; they are not silently clipped in the model.

Manual changes are pose inspections, not time trajectories. Their reported rates
are zero and the UI clears history to avoid joining discontinuous poses. Pause
and speed changes are inspection controls, not acceleration-limited motor
commands. Automatic derivatives are analytic within each constant-speed segment.

The body is not fixed to the world. If both leg angles are equal, the body moves
vertically so each wheel center stays at the wheel radius. Different leg angles
require lateral body roll as well as heave; shifting the two wheels separately
would deform the mechanism and is not used.

The common transform solves both tire contact constraints. For cylindrical tires
of radius R, width w, body roll ρ and leg-plane spacing S:

- S sin(ρ) + (z_OL − z_OR) cos(ρ) = 0;
- tire vertical extent = R cos(ρ) + (w/2)|sin(ρ)|;
- body height is that extent minus the transformed mean local wheel height.

The tire rim/cap edge, not merely the center, reaches z=0 when tilted. All links
and mounts undergo the same rigid transform. Preview width/spacing are still
illustrative, while leg lengths and wheel diameter come from canonical geometry.
Absolute wheel spin remains zero in the body frame; normal contact alone does
not impose rolling without slip.

The table reports world wheel positions, tire/ground gap, body height and roll.
The three plots and velocity/acceleration rows are explicitly **body-frame**
quantities. Local vertical wheel motion is relative to the moving body and must
not be read as world wheel lift. For equal leg poses the world wheel height is
constant and its world vertical speed is zero. A tilted finite cylinder's center
can change height slightly while its lowest surface point remains on the ground.

## Geometry and ROS ownership

`duk_core.modeling.ground_contact` owns the explicit local-to-world placements.
`duk_core.modeling.kinematic_bench` owns body-frame profiles/diagnostics, using the
canonical solver, Jacobian and second derivatives. It does not duplicate the
four-bar equations. `duk_description` owns the ROS adapter, visualization tree,
RViz launch and Qt panel.

A URDF must be a tree. The visualization cuts the C closure joint and publishes
the coupled AB, CD, BCO-relative and wheel-relative angles from solved geometry.
The eight published angles are **not eight independently actuated physical
joints**. Both paths to C coincide geometrically. Do not use this visualization
URDF as a closed-loop dynamics model.

ROS interfaces are scoped to this virtual bench:

| Interface | Content |
| --- | --- |
| `/duk/kinematics/joint_states` | Eight coupled visualization angles |
| `/duk/kinematics/robot_description` | RViz internal tree; world/body transform comes from the contact placement |
| `/duk/kinematics/diagnostics` | Calculated signals and geometry error metrics |
| `/tf`, `/tf_static` | Moving body transform from the preview node; internal frames from robot_state_publisher |

The bench node `/duk/kinematics/bench` accepts atomic ROS parameters
`left_q_deg`, `right_q_deg`, `playing` and `speed`. Pause before changing manual
angles, or include `playing=false` in the same atomic request. Invalid requests
leave the entire last valid state intact. Angle parameters are manual setpoints;
read live angles from diagnostics/joint states during auto playback. Wall time
drives this RViz-only bench; it does not require Gazebo's `/clock`.

## Verification

Inside the sourced simulation environment, with the bench running:

```bash
python tools/environment-check/verify-kinematic-bench.py
```

This script changes only the virtual preview. It checks independent poses,
actual timestamped TF closure at C, wheel-center placement and lowest tire point
at z=0, atomic rejection,
automatic motion, pause and speed validation, then restores the reference pose.
Unit tests independently reconstruct the URDF transforms and visual cylinder
endpoints. A headless Qt test exercises real widgets, drawing, playback and pause.
The [simulation workflow](../../.github/workflows/simulation.yml) includes these
checks; local execution is distinct from a completed remote CI run.

[Static Gazebo preview](simulation-preview.md) · [Mathematical model](../theory/modeling.md).
