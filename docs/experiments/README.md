# Experiment Protocol

## Experiment structure

An experiment in `research/experiments/` should be self-contained in its definition but reuse canonical core and simulation APIs. Each study needs:

- an objective and measurable acceptance criteria;
- a configuration with model/controller revisions, units and scenario parameters;
- an entry point or documented procedure;
- explicit data capture and failure-handling rules;
- analysis code and output metric definitions;
- a summary of results, limitations and conditions for reproduction.

## Execution and data

Write large recordings and temporary results to `data/` or an external artifact store, not the source directory. Keep provenance (commit, environment, platform, configuration, seed and calibration) near the artifact or in a manifest.

Separate raw observations from processed features and published figures. Never silently replace failed trials or discard anomalous runs without recording the exclusion criteria.

## Benchmarks

Benchmark cases under `research/benchmarks/` define comparable operating conditions and evaluation metrics. A new controller should not redefine the benchmark to favor itself. Make any benchmark revision explicit and preserve historical result comparability.

## Safety

Physical experiments require equipment checks, authorization, operational limits, emergency-stop availability and safe shutdown procedures. Simulator outputs do not certify hardware safety.

See [Research methodology](../research/methodology.md).
