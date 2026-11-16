# DUK v3 corrections reconciled on 2026-11-11

Read the [handbook index](https://app.notion.com/p/4131f3cc033281a4b73dcdb38ed41514),
chapters 00–10 and appendices A–B through the connected Notion source. The
relevant chapter edits are timestamped 2026-11-11 11:52 UTC. The default
24-hour version comparison identifies the changed blocks; the current full
pages supply the equations and tables, including blocks rendered as `unknown`
in the change feed. The full page responses contain no `unknown` blocks.
This review updates local code, inputs and notebooks.

## Reconciliation

| Source correction | Core interpretation and notebook evidence |
| --- | --- |
| [01–02: frame and geometry](https://app.notion.com/p/4131f3cc033281a2aae1f8e6bc96d1fc) | Keep the canonical geometry. Mathematical +x is not confirmed physical front/back. NB1 checks the 70°/135° Jacobian examples and displays signed projected A–wheel clearance from `sample_bench_leg`; motion reserves are numerical preview choices. |
| [03: static loads](https://app.notion.com/p/4131f3cc03328196a898faaeb9af6530) | `static_hip_effort` already uses gravity minus external generalized load. NB2 compares the separate 2 N·m-per-hip and 3.5 kg illustrative assumptions without replacing its motor fixture. |
| [04: corrected rolling sign](https://app.notion.com/p/4131f3cc033281f4bd54f24e8f92ce8e) | Canonical rolling remains Ȯₓ + rφ̇ = 0. Existing constraints already use this sign. Regression tests check the transformed velocities, motor work and reconstructed traction. NB3 explains the canonical force-estimator signs. |
| [05: wheel/torque convention](https://app.notion.com/p/4131f3cc033281fdb641fadc2fe1ad72) | Negate both absolute spin and torque when converting to the +x pendulum convention. Relative encoders require body and carrier compensation. Keep the canonical estimator formula Fₓ = (Iφ̈ − τ)/r. |
| [06: CoM lean, constants, pole/zero](https://app.notion.com/p/4131f3cc033281c4a7afcb8b23228bba) | θ_e = θ_G0(q) − θ_CCW. Keep the existing rounded `v2_balance` gain baseline and add the distinct `handbook_linear_reference` input. `WheeledPendulum.upright_characteristics` reports the unstable pole, e-folding time and positive torque→axle-position zero. NB4 displays both parameter sets, retains the independent 7 DOF linearization and shows the linear initial inverse position response. |
| [07: changing posture](https://app.notion.com/p/4131f3cc0332817ca50bd61a2dae6b56) | NB5 distinguishes its fixed-offset state coordinate from the instantaneous equivalent CoM lean used for plots and criteria. The common prescribed-motion plant already includes q̇ and q̈; a frozen gain alone does not supply these dynamics. |
| [04/08: contact rank and height](https://app.notion.com/p/4131f3cc03328156af1dc6b76714fce1) | NB6 demonstrates full contact ranks five at equal hips and six at unequal hips. Its shared lateral allocation row remains an explicit approximation outside the aligned case. The sinusoidal CoM support bound is not applied as an exact body-reference-height constraint. |

## Numerical references and limits

The exact chapter 06 example uses g=9.80665 m/s² and aggregate solid-disk wheel
inertia 0.0016875 kg·m². It gives A₄₃=74.26114244 s⁻² and
B₄=−34.96694531 (rad/s²)/(N·m). The existing rounded example uses g=9.81 and
I_w=0.00169, giving 74.27837626 and −34.96079914 respectively.
The new regression compares each to its own published two-decimal reference.
Pole/zero checks independently use state-space eigenvalues and transfer-function
numerator roots, rather than treating the rounded gain as an equation oracle.

For the exact illustrative example, the unstable pole is 8.61749 rad/s,
the angle e-folding time is 0.116043 s and the positive axle-position zero is
5.29150 rad/s. These characterize this local model and output, not an approved
hardware latency, bandwidth or universal property of every task output.
Neither pendulum fixture includes reflected rotor inertia; no unidentified rotor
inertia or gear ratio is introduced.

Negative projected A–wheel clearance near 130° flags overlap of 2D projections.
It cannot establish physical collision without lateral separation and CAD.
The assembly interval near 69.0°–135.3° is not an approved operating interval.
The 2 N·m static example excludes moving-link gravity and acceleration and is
not a payload rating. The current mass, actuator, contact and sensor inputs
remain synthetic.

Some handbook inventory/status sentences still describe an empty control
directory or notebooks not yet rerun. Those sentences are not adopted as local
implementation facts: the current repository contains the control modules and
executable studies. The corrected equations are reconciled against the actual
implementation and numerical checks. No hardware or deployment claim follows.

## Reproducing the local checks

Run the repository checks documented in [testing](../../../docs/development/testing.md),
including `test_handbook_reference.py` and `test_kinematic_bench.py`.
Execute all seven notebooks using the [notebook runner](../../../robot/software/core/examples/run_duk_notebooks.py)
with `--output-dir data/results/duk-v3-notion-sync` from the repository root.
Generated results and source snapshots belong under `data/`; source notebooks
remain unexecuted. The [research input README](README.md) describes the broader
study assumptions and previous convergence evidence.

The verification run on 2026-11-11 passed 241 tests with one ROS test deselected.
All seven notebooks executed successfully. The live-control checks exercised
5 balance, 12 height and 14 spatial cached cases, plus both pose-slider endpoints
in both coordinate frames. Executed copies match the current notebook cell
sources; committed notebook outputs remain empty. Ruff, Python compilation and
local documentation link checks also passed. These are software checks on
synthetic inputs; deliberately stopped or failed scenario criteria remain visible
in the notebook results.
