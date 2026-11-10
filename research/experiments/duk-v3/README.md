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
