# Research Methodology and Reproducibility

## Research question and experimental design

Define the scientific objective, hypothesis, model fidelity, control condition and expected observable outcome before comparing algorithms. Distinguish conceptual claims (e.g. stability of a local linear model) from evidence produced by numerical simulation, high-fidelity physics or physical hardware.

## Comparable conditions

Algorithms in a benchmark must use consistent initial conditions, disturbances, constraints, sensor assumptions and evaluation metrics unless a difference is deliberately under study. Report baselines and explain parameter-tuning budgets.

Potential metrics for wheel-legged balance and trajectory tracking include angle and position error, settling time, overshoot, control effort, saturation, disturbance rejection, energy use and failure rate. Select metrics to match the actual hypothesis and measurement capabilities.

## Reproducibility record

Every experiment should preserve:

1. experiment identifier, objective and method;
2. source commit, model/hardware revision and dependencies;
3. physical, controller and estimator parameter set;
4. initial conditions, sampling period, scenario and random seed;
5. measured variables, coordinate frames and data units;
6. result artifacts, analysis scripts and processing versions;
7. exclusions, anomalies, uncertainty and experiment limitations.

## Evidence quality

Separate training/tuning conditions from evaluation conditions where appropriate. Repeat stochastic trials and report distributions rather than only a best case. Physical experiments must document environmental conditions, hardware wear and calibration changes.

An apparent simulated improvement is not equivalent to validated physical stability. Report model assumptions and transfer limitations explicitly.

## Publications

Figures and tables should be reproducible from versioned scripts and traceable data. Manuscripts in `research/publications/` should cite the exact experiment/benchmark definitions that support their claims.

See [Experiment protocol](../experiments/README.md), [Testing](../development/testing.md) and [Physical parameters](../architecture/parameter-governance.md).
