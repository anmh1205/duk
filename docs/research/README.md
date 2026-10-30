# Research and Scientific Evidence

DUK research builds on the canonical robot model and software to investigate design choices, stability, locomotion, control and system performance. Research artifacts must remain reproducible and traceable to model, hardware and software revisions.

## Research areas

- `research/experiments/`: experimental configurations and executable protocols;
- `research/benchmarks/`: shared scenarios, baselines and comparable metrics;
- `research/publications/`: theses, manuscripts, supporting figures and references.

Exploratory model and controller notebooks live beside the reusable core in `robot/software/core/notebooks/`. Scientific research code imports those APIs instead of maintaining parallel dynamics or control implementations.

Follow the [research methodology](methodology.md) and [experiment protocol](../experiments/README.md) when publishing quantitative results.
