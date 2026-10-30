# Physical Parameters and Model Consistency

## Governing principle

Each physical property of DUK requires a known authoritative source, revision, unit, uncertainty or tolerance where relevant, and a controlled publication path to downstream representations. Geometry, inertia, motor constants, encoder calibration and controller gains have different provenance and must not be placed in an undifferentiated parameter file.

## Parameter lifecycle

1. **Establish the source.** Identify CAD-derived nominal geometry, supplier specifications, measured values or identified dynamics. Record the method and source revision.
2. **Review the model.** Document coordinate frames, sign conventions, units and whether a value applies to a specific hardware assembly or revision.
3. **Publish a configuration.** Store validated robot-level nominal parameters in `robot/config/physical/`, controller settings in `robot/config/controllers/`, and device-specific calibration in `robot/config/calibration/`.
4. **Generate or validate consumers.** Python core inputs, URDF/Xacro, Gazebo physics and runtime configuration must read or be checked against the published dataset.
5. **Check consistency.** Validate mass totals, geometric relationships, inertia conventions, link naming, joint limits and sensor/actuator ordering with explicit tolerances.
6. **Retain provenance.** Preserve the originating CAD/measurement revision, schema version and a record of applicable robot hardware.

## Multiple model fidelity levels

A planar balancing model, a multibody dynamic model and a manufacturing CAD assembly are not interchangeable. Each model must state its assumptions, degrees of freedom, neglected effects and applicable operating envelope. Where a parameter is aggregated or fitted, document the transformation from physical sources to the reduced model.

## Calibration

Calibration may differ between individual robots and may change during operation. Protect device-specific calibration data from accidental overwrite, record acquisition conditions and keep reference calibration distinct from nominal design parameters.

## Generated files

Meshes, URDF fragments, manufacturing exports and plotted results are derived artifacts. Any retained generated file needs an identified upstream source and regeneration or verification procedure. Do not edit a generated artifact independently from its authoritative source.
