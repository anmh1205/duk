# DUK Research

The research area owns scientific experiment definitions, comparative benchmarks and publication materials. It is distinct from the interactive engineering notebooks and reusable algorithms maintained under the robotics core.

## Studies and benchmarks

`experiments/` defines reproducible studies with documented objectives, configuration, initial conditions, measurements and analysis procedures. `benchmarks/` defines shared evaluation conditions that can be reused to compare models and controllers. `publications/` contains thesis/manuscript material and supporting scientific artifacts.

A study should use the canonical `duk_core` API and established robot parameters, rather than maintaining a parallel implementation of kinematics, dynamics or control. Experimental scripts can orchestrate simulations or measurements without becoming the owner of the scientific model.

## Reproducibility

Each experiment needs a precise source revision, model/hardware configuration, software environment, scenario, random seed where applicable, analysis version and recorded metrics. Distinguish simulation-only conclusions from results demonstrated on physical hardware.

Store large recordings and generated result files under ignored local data directories or a managed external artifact store; commit only the configuration and analysis material needed for reproduction.

## Publication quality

Reported results must have traceable source data, clear units, declared uncertainty and fair baselines. Reusable charts and numerical drawing functions remain in core visualization; scientific figures may be generated from recorded experiment data.

See [Research methodology](../docs/research/methodology.md) and [Experiment protocol](../docs/experiments/README.md).
