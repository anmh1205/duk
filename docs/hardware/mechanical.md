# Mechanical Design and CAD

## Asset ownership

`robot/hardware/mechanical/cad/assemblies/` contains editable assembly projects; `cad/parts/` contains editable part models. `drawings/` contains dimensioned drawings and inspection references. `manufacturing/` contains controlled exports such as STEP, STL, DXF or 3MF when a fabrication process requires them.

A mechanical part should have a stable identifier, a revision, units, material and process assumptions, and a clear relationship to its parent assembly. Distinguish design geometry from simplified visualization and collision geometry.

## Assembly and kinematic conventions

Every actuated joint should be documented with axis direction, orientation, travel limits, mechanical hard stops, actuator interface and relevant dimensional tolerances. Link identifiers should be traceable to robotics model names without requiring that CAD internal names exactly match ROS frame IDs.

The mechanical source design is authoritative for nominal geometry; measured assembly dimensions and calibrated joint offsets may override nominal values only through an explicit revisioned configuration.

## Mass properties

Mass, center of mass and inertia tensor must declare the reference frame, origin and units. Document whether properties come from CAD, measurements or system identification. Avoid combining inertias evaluated at different reference points or body configurations without the proper transformations.

A reduced planar model may aggregate multiple components into effective masses and inertias. Record the reduction assumptions so the approximation can be compared with the full robot configuration.

## Exports and derived descriptions

Generate collision and visual meshes from controlled source geometry; validate scale, axis directions, origin and simplification tolerance. URDF/Xacro belongs in `robot/software/ros2_ws/src/duk_description/`. Manufacturing exports must reference their originating CAD revision and intended fabrication process.

## Verification and handoff

Mechanical review includes constraints, clearances, interference detection, fasteners, stress-critical interfaces where applicable, motion range, cable routing and manufacturing feasibility. Handoffs to electronics, firmware and software need reviewed interface dimensions, connector access, sensor alignment and joint conventions.

See [Parameter governance](../architecture/parameter-governance.md) and [Electronics](electronics.md).
