# Research-specific guidance

Read root [AGENTS.md](../AGENTS.md). Scientific experiments must identify
the model revision, robot parameters, initial conditions, seeds (where applicable),
metrics and data provenance. The canonical algorithms remain in
`robot/software/core/src/duk_core/`; research scripts import them.

Do not commit raw rosbag, videos or large experiment output into Git.
Do not claim simulation results demonstrate safety on the physical robot.
