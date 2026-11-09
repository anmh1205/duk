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
