# Engineering Standards

DUK follows consistent rules for correctness, provenance, review and reproducibility across software, mechanical engineering, electronics, firmware, simulation and research. Each artifact must make its intended purpose, authoritative source and verification method clear.

## Ownership and interfaces

Place canonical algorithms in the robotics core, not in notebooks, ROS adapters or experiment runners. Keep editable mechanical and electronic source projects in their hardware directories; treat generated meshes, manufacturing packages and robot descriptions as distinct downstream artifacts. Use shared parameters and documented transforms to maintain consistency rather than independently entering the same dimensions or inertias in multiple models.

For public interfaces, define inputs, outputs, units, coordinate frames, array dimensions, error handling and applicable timing assumptions. Changes to state ordering, connector pinout, message semantics or calibration meaning require coordinated updates to affected consumers and validation.

## Software quality

Modules should expose reusable functions with explicit dependencies and predictable behavior. Avoid implicit file reads, hidden device access, global state or GUI imports during core numerical-module import. Notebook cells may orchestrate calculations and generate figures but must not be the sole location of a reusable equation.

Use tests proportional to risk: numerical invariants and analytic cases for models; message and lifecycle checks for ROS; protocol and fault-state checks for firmware; interface checks for applications. Maintain a traceable failure report when a fix changes expected behavior.

## Mechanical and electrical quality

Mechanical revisions require traceable source CAD, applicable drawings, physical interfaces, tolerances and a clear method of generating downstream files. Electrical revisions require a reviewed schematic, PCB source, electrical and physical connector definitions, power-budget assumptions and manufacturable output.

Hardware changes must identify the corresponding firmware and software interfaces. Do not substitute supplier nominal specifications for assembled-robot calibration without establishing their applicability.

## Scientific quality

Scientific comparisons require common evaluation conditions, relevant baselines and declared metrics. Experiments must identify model revision, configuration, input scenario, random seed where applicable and data-processing steps. Distinguish analytical conclusions, simulation outcomes and measurements on physical hardware.

## Version control and review

Changes should be coherent, scoped to an owning subsystem and accompanied by the documentation they affect. Prefer one reviewable commit or pull request for a logical change rather than multiple scaffolding or repair commits. Preserve source history and do not overwrite user-owned files outside the authorized scope.

Do not commit secrets, machine-specific paths, virtual environments, ROS build/install/log output, temporary plots or bulk sensor recordings. Use `.gitkeep` only for intentionally empty directories, removing it when real files are introduced.

## Safety

Any operation that can energize motors, modify actuator limits, change battery/power configuration or flash embedded hardware requires explicit authorization and a suitable test plan. Simulated stability or software-test success is not evidence that a physical configuration is safe.

See [Contributing](../../CONTRIBUTING.md), [Data contracts](../architecture/data-contracts.md) and [Testing](testing.md).
