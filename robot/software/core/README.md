# Robotics Core

`robot/software/core/` contains ROS-independent numerical models, visualization and interactive engineering work for DUK.

## Implemented foundation

| Area | Current scope |
| --- | --- |
| `src/duk_core/modeling/leg_linkage.py` | Canonical robot-frame four-bar kinematics, rigid BCO carrier, workspace, analytic Jacobian and trajectories |
| `src/duk_core/modeling/ground_contact.py` | Explicit rigid world placements: upright planar leg or heave/roll for both finite-width tires on flat ground |
| `src/duk_core/modeling/leg_differential.py` | Analytic second derivatives, time acceleration and physical-branch inverse wheel position |
| `src/duk_core/modeling/kinematic_bench.py` | Body-frame two-leg preview profiles and geometric diagnostics, without physical parameters |
| `src/duk_core/modeling/planar_biped.py` | Candidate planar mass matrix, convective/gravity/damping forces, wheel reactions and optional contact constraints with unidentified symbols |
| `src/duk_core/modeling/mass_properties.py` | Reusable planar mass-property composition for later dynamics |
| `src/duk_core/visualization/leg_linkage.py` | Static Matplotlib mechanism, trajectory, coordinate and Jacobian plots |
| `src/duk_core/visualization/leg_animation.py` | Real-time single-pose animation built on the canonical solver; no duplicated robot equations |
| `examples/duk_leg_kinematics.py` | Numerical inspection of current physical geometry |
| `tests/` | Equation, topology, rigid-body, workspace, Jacobian and notebook validation |
| `notebooks/` | Kinematics and motion workbenches built on the reusable core API |
| `src/duk_core/estimation/` | Complementary pitch and slip-gated velocity replay |
| `src/duk_core/control/` | LQR/MPC, effort feasibility, QP/HQP allocation and task policies |
| `src/duk_core/simulation/` | Numerical plant execution, synthetic observations, scenarios and result traces |

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

The live animation updates one mechanism pose at a time, with the tire tangent
to world ground z=0 and the body origin moving vertically. Canonical poses remain
unchanged. Set `LegAnimationOptions(ground_contact=False)` only for an explicitly
body-frame inspection. History is opt-in.

Inspect the symbolic model without inventing measurements:

```bash
python robot/software/core/examples/duk_planar_model.py
```

Use `--parameters .local/planar-parameters.json` to bind a partial set of identified
values; unknown/null values stay symbolic. See the [parameter template](../../config/physical/duk_planar_parameters.example.json).

## DUK v3 research notebooks

| Study | Notebook | Candidate evaluated |
| --- | --- | --- |
| NB1 | [Kinematics](notebooks/kinematics/duk-leg-kinematics.ipynb) and [motion](notebooks/motion/duk-leg-motion.ipynb) | Canonical four-bar, BCO invariants, derivative regression and world placement |
| NB2 | [Hip loads](notebooks/dynamics/duk-hip-load.ipynb) | Mass composition, static effort and a torque–speed envelope |
| NB3 | [Planar contact](notebooks/dynamics/duk-planar-contact.ipynb) | Compiled 7 DOF dynamics, rigid rolling, compliant slip and sensor replay |
| NB4 | [Fixed-hip balance](notebooks/control/duk-fixed-hip-balance.ipynb) | Wheel-moment pendulum, unanchored/anchored LQR, saturation and traction |
| NB5 | [Height balance](notebooks/control/duk-height-balance.ipynb) | Fixed/scheduled/TV-LQR and MPC with prescribed symmetric hip motion |
| NB6 | [Whole-body allocation](notebooks/control/duk-whole-body.ipynb) | 3D compliant simulation, weighted QP/HQP, feature transitions and supervision |

Install the optional research environment and run the sequence from the repo root.

```bash
python -m pip install -e "./robot/software/core[test,research,dev]"
OPENBLAS_NUM_THREADS=1 python robot/software/core/examples/run_duk_notebooks.py
```

List the studies, or rerun selected notebooks while retaining canonical order:

```bash
python robot/software/core/examples/run_duk_notebooks.py --list
OPENBLAS_NUM_THREADS=1 python robot/software/core/examples/run_duk_notebooks.py \
  --notebook control/duk-height-balance.ipynb \
  --output-dir data/results/duk-v3 --timeout 300
```

Repeat `--notebook` to select several studies. The timeout applies to each code
cell. With `--output-dir`, a failed execution is also saved for inspection, and
the process still exits with an error. Source notebooks are never overwritten.
Each saved notebook embeds a `duk_run` manifest. The output directory also gets
`run-manifest.json` and `source-snapshot.tar.gz`, recording source/input hashes,
the Git state and Python/package versions. The snapshot includes uncommitted
source and input files; a commit hash alone cannot reproduce a dirty checkout.

Use `--output-dir data/results/duk-v3` to save executed notebook copies with plots
and printed outcomes. The runner requires executed copies to stay outside the
source notebook tree. The [experiment inputs](../../../research/experiments/duk-v3/README.md)
separate canonical geometry from synthetic masses, contact and actuator settings.
The numerical and control modules are Python research code. The spatial study
uses thin disks on horizontal supports and a shared lateral rolling row in the allocator.
Independent soft contact forces still act at each wheel in the simulated plant.
The sensor, driver and stop policies are explicit simulation inputs.

### Inspect notebook results

Run all cells in order with a live Python kernel. NB1 includes an angle slider
and a body/world frame switch. The motion study displays analytical velocity
and acceleration of its prescribed sine profile. NB2 separates quantities with
different units into individual panels. NB3–NB6 include compact result tables;
NB4 adds a recovery map using its declared angle, speed and horizon criteria.

NB4–NB6 have case pickers for inspecting cached trials without repeating the
simulation. NB5 marks the final integrated lean; NB6 marks the terminal pose and
shows allocator, supervisor and contact timelines through the actual stop time.
Held wheel-command and feature curves continue through the last applied interval,
including a partial interval. An instantaneous stop is not extended past its
actual stop time. The NB5 comparison shows the declared bounds in all three
trajectory columns.
PASS refers to the criterion printed with that table. A completed simulation
does not by itself establish recovery or satisfy the angle criteria.

The pickers require a live kernel. Executed copies also contain an ordinary
initial figure and result tables that can be read without a widget manager.
Shared presentation helpers live in
[`visualization/notebook.py`](src/duk_core/visualization/notebook.py); they consume
computed results and do not own model equations or controller decisions.

Planar CoM acceleration is available as
`NumericPlanarBiped.center_of_mass_acceleration(configuration, velocity, acceleration)`
in world `(x, z)` order and m/s². NB3 uses this analytic body-kinematics result;
diagnostics do not probe neighbouring configurations outside the linkage branch.
`upper_body_angle_derivatives(q)` supplies analytic first/second derivatives of
the equivalent upper CoM angle along equal left/right hip motion. NB5 uses this
API for its nominal posture path, including valid poses near a geometric boundary.

The spatial experiment applies an impulse at the first command sample at or
after its configured time, consistent with sampled friction and fault changes.
It does not apply impulses at or beyond the terminal horizon.

Simulation entry points are `duk_core.simulation.planar`, `.balance`, `.height`
and `.whole_body`. `HipTrajectory` lives in `duk_core.modeling.trajectories`.
The [core architecture](../../../docs/software/core.md) documents the module
boundaries and import migration from the earlier notebook implementation.

LQR/MPC APIs validate finite, consistent state-space dimensions and symmetric
quadratic costs: state/terminal costs must be positive semidefinite and effort
costs positive definite. Invalid problem data raises `ValueError`; a valid but
infeasible MPC problem returns `None`. MPC supports stages with no force rows
while retaining motor bounds. Linearization accepts array-like callback outputs
and preserves evaluations when a callback reuses its output buffer.

Contact APIs require finite measurements and nonnegative friction. The compliant
law accepts scalar or broadcastable contact batches with one/two tangential
components; tangential forces oppose slip and remain inside the friction cone.
NB3 plots reconstructed forces, tangential dissipation and the slip detector's
confusion matrix over both high- and low-grip sensor replays, with explicit class
counts. These short synthetic trials do not establish long-duration detector
performance. NB5 asserts its declared criteria for all twelve baseline cases.
`PlanarTrace.force_admissible` describes force bounds; it does
not imply sticking. The original `rolling_feasible` field remains available.
Rigid rolling accepts scalar/per-wheel finite nonnegative friction and validates
effort and tolerance before solving; invalid inputs raise `ValueError`.

NB6 records the synthetic plant friction and a copy of the observer's belief at
each sample. Its low-grip chart exposes observation delay and belief updates.
The observed ratio uses the Euclidean tangent-force norm of the plant law,
so rotating the slip direction does not change the inferred coefficient. The
allocator retains its existing inscribed linear friction constraints.

NB6 bounce feedforward differentiates both the sine wave and feature
participation. The existing ramp remains piecewise linear: its rate can jump at
transition breakpoints. Height acceleration describes the regular derivative
between those breakpoints, without an impulsive term or a claim of C² motion.

The [2026-11-11 Notion v3 correction review](../../../research/experiments/duk-v3/notion-v3-review.md)
records the updated sign mapping, conditional contact rank and exact-versus-rounded
pendulum inputs. `WheeledPendulum.upright_characteristics()` reports the local
unstable pole, angle e-folding time and torque-to-axle-position positive zero.
`sample_bench_leg(...).projected_hip_clearance_mm` reports signed 2D A–wheel
clearance, which does not establish CAD collision. NB1/NB2 expose geometric and
static-load cautions; NB4 displays both numerical references and inverse position
response; NB6 demonstrates why rank five is conditional on axle alignment.

## Code organization rules

The research stack is intentionally separated by responsibility:

1. **Modeling** owns equations, topology and invariants.
2. **Visualization** consumes solved poses; it must not duplicate geometry equations.
3. **Animation** updates Matplotlib artists from solved poses; it does not alter the robot model.
4. **Simulation** composes models, controllers and estimators into experiments; those lower layers do not import simulation runners.
5. **Notebooks** compose reusable APIs for investigation; equations that belong to the robot model must be moved into `duk_core` rather than copied into cells.

Comments should explain physical meaning, mathematical intent and non-obvious constraints. They should not narrate trivial Python syntax line by line.

Run `ruff check .`, `ruff format --check .` and `python -m pytest -q -m 'not ros'`
from the repository root. These checks also cover the Python adapters and tools.
See [engineering rules](../../../docs/development/engineering-rules.md) for the
enforced conventions and [testing](../../../docs/development/testing.md) for ROS
and notebook execution.
