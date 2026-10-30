# Electronics and System Wiring

## Board projects

Each PCB project belongs in a dedicated subdirectory of `robot/hardware/electronics/boards/`. Keep its schematic, PCB source, assembly information and controlled fabrication outputs together and clearly versioned. Shared symbols, footprints and component models belong in `electronics/libraries/` when used by multiple designs.

Every board release should identify revision, electrical constraints, applicable BOM, power domains, connector assignments, assembly variants and design-rule/electrical-rule review evidence.

## Power architecture

`electronics/system/power/` describes the system-level energy path: battery and power input, regulation, power distribution, protection, grounding, fuse or current limits and emergency-stop implications. Document voltage/current budget and thermal assumptions for motors, compute, sensors and peripherals.

The power-system overview does not replace a detailed schematic stored with a particular PCB. Use cross-references rather than maintaining two independent schematics with conflicting component values.

## Wiring and connectors

`electronics/system/wiring/` defines cables, harnesses, connector part numbers, pin assignments, shield/ground strategy, wire gauges, polarity and mechanical routing. Pins used by firmware or motor interfaces require stable identifiers and a review process for incompatible changes.

Wiring diagrams must distinguish logical signal names from physical connector locations and orientation. Protect against ambiguous pin numbering and mirrored connector views.

## Interfaces and validation

Hardware–firmware contracts include electrical levels, pull-up/down assumptions, isolation and transport (e.g. CAN, UART, SPI). Sensor and motor data sheets do not establish actual assembled-robot calibration.

Perform power-on checks with defined limits, inspect polarity and short-circuit conditions, and stage actuator activation through safe commissioning procedures. Report measurements and deviations relative to the documented design.

See [Firmware architecture](../software/firmware.md), [Data contracts](../architecture/data-contracts.md) and [Mechanical design](mechanical.md).
