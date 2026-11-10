# DUK v3 research inputs

[DUK v3 on Notion](https://app.notion.com/p/4131f3cc033281a4b73dcdb38ed41514) defines the study sequence.
The [notebooks](../../../robot/software/core/README.md#duk-v3-research-notebooks) run it
with the [canonical leg geometry](../../../robot/config/physical/duk_leg_geometry.json)
and [synthetic.json](synthetic.json). The numeric fixture provides illustrative
rigid-body properties, actuator bounds, contact laws, sensors and scenario
settings. The aggregate pendulum numbers in `v2_balance` come from the
[v2 example](https://app.notion.com/p/4131f3cc0332817aa7f9f83fd1ce94a7).
The [physical parameter template](../../../robot/config/physical/duk_planar_parameters.example.json)
records unidentified parameters separately.

The notebooks log input hashes, commit, trial settings and outcomes. Keep
generated outputs under `data/results/`. Motion, controller and fault results
apply to the modeled conditions and numerical tolerances of each study.

NB3 records the event time and state when rigid rolling reaches its force limit
or flight reaches the support. The solver supplies that sample separately from
the requested time grid. [SciPy event outputs](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html)

NB4 reports an empty torque interval as `traction_infeasible` and clears the
simulated delay queue. NB5 checks held torque at the start, midpoint and end
of each step. A rejected step leaves the accepted trajectory at its last
feasible endpoint. NB6 logs the applied command source. A fallback regulates
speed toward zero and pauses optional tasks. It uses normal force shares from
the simulator to estimate the traction available to equal wheel moments.
These checks describe the numerical experiment and its stop policy.

NB6 also constrains interior extrema of the constant-acceleration hip prediction.
An outward velocity near a travel boundary can require braking before the end
of the lookahead. The numerical plant checks travel, fall and prolonged flight
at each integration substep. `terminal_pose` records the final integrated pose,
and `peak_abs_lean_roll` includes those substeps. Controller rows still describe
the state at command issue time. If travel leaves the four-bar assembly domain,
the run stops with `hip_travel` and reports an unavailable terminal pose as NaN.
Rolling odometry accounts for the tilted disk
support geometry using body, carrier and wheel rotation rates.

The [spatial regressions](../../../robot/software/core/tests/test_spatial_biped.py)
check power balance with unequal viscous losses, contact-gap derivatives and
horizontal frame transformations at ten reproducible synthetic states. They
also cover hip travel between prediction endpoints and a fall on the final
integration step. The [control regressions](../../../robot/software/core/tests/test_research_control.py)
check motor work and dissipative slip at eight synthetic states and compare
unconstrained MPC with the Riccati solution. These are equation and software
checks, separate from physical parameter identification.

For the nominal NB6 scenario, a step-refinement check held the 0.015 s controller
period fixed and compared 40 versus 80 integration substeps per command. Both
runs completed 0.9 s with all allocations solved. At controller ticks, the maximum
body-height difference was 8.49e-6 m and the maximum lean difference was 1.94e-5 rad.
The final body-height difference was 1.61e-6 m. This is a comparison of two step
sizes on one synthetic trajectory. To repeat it, call `run_whole_body` with the
same nominal inputs and change only `settings["substeps"]` from 40 to 80.
