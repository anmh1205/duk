# Two-leg RViz kinematic bench

This is a fixed-body geometry inspection tool. Two independent canonical leg
coordinates drive a coupled visualization tree in RViz. It does not run Gazebo,
simulate loads/contact, select a controller, or send motor commands. Unknown
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
- Three plots show world wheel X/Z position and calculated profile derivatives;
  dashed lines are X, solid lines are Z, orange is left and cyan is right.

The selectable interval excludes five degrees at each geometric endpoint. That
reserve, nominal period, speed range and warning threshold are visualization
choices, **not identified motor or mechanical operating limits**. Requests
outside the interval are rejected; they are not silently clipped in the model.

Manual changes are pose inspections, not time trajectories. Their reported rates
are zero and the UI clears history to avoid joining discontinuous poses. Pause
and speed changes are inspection controls, not acceleration-limited motor
commands. Automatic derivatives are analytic within each constant-speed segment.

The body is fixed high enough to clear both wheels for every geometric pose,
using AB+BO+wheel-radius plus 40 mm illustrative clearance. Ground contact is not
imposed and the absolute wheel angles are held at zero. The tire markers make
that convention visible. Body size, spacing and colors reuse the static preview;
they do not become physical mass/CoM/mount measurements.

## Geometry and ROS ownership

`duk_core.modeling.kinematic_bench` owns the profile and diagnostics, using the
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
| `/duk/kinematics/robot_description` | RViz tree, with a fixed world/body mount |
| `/duk/kinematics/diagnostics` | Calculated signals and geometry error metrics |
| `/tf`, `/tf_static` | Moving/fixed visualization frames from robot_state_publisher |

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
actual timestamped TF closure at C and the wheel center, atomic rejection,
automatic motion, pause and speed validation, then restores the reference pose.
Unit tests independently reconstruct the URDF transforms and visual cylinder
endpoints. A headless Qt test exercises real widgets, drawing, playback and pause.
The [simulation workflow](../../.github/workflows/simulation.yml) includes these
checks; local execution is distinct from a completed remote CI run.

[Static Gazebo preview](simulation-preview.md) · [Mathematical model](../theory/modeling.md).
