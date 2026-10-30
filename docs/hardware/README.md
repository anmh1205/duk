# Hardware Engineering

DUK hardware includes the mechanical assembly, motors, sensors, electronics, power system and physical interconnects. Hardware assets belong under `robot/hardware/`; release documentation and cross-disciplinary specifications are maintained here.

## Mechanical engineering

[Mechanical design](mechanical.md) defines assembly ownership, editable CAD, part revisions, dimensional drawings, material/process assumptions and manufacturing export traceability. An STL or simplified collision mesh is not a substitute for an editable design source.

## Electronics engineering

[Electronics design](electronics.md) defines board project organization, electrical schematics, PCB layouts, component libraries, power distribution and connector/wiring documentation.

## Interface with software

The robot-description package consumes simplified geometric and inertial data. Modeling and estimation use reviewed physical parameters, not undocumented values copied from CAD exports. Firmware consumes validated connector pinouts, motor/sensor capabilities and protocol contracts.

## Release evidence

A hardware release should identify assembly revision, supplier/BOM data, design checks, component substitutions, measured or estimated mass/inertia where applicable, export versions and fabrication notes. Production-ready fabrication packages require discipline-specific review before ordering or machining.
