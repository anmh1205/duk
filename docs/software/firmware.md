# Embedded Firmware Architecture

## Ownership and target isolation

`robot/software/firmware/` holds source projects for MCU-based control, sensor interfaces, communication endpoints and other embedded targets. Organize implementation around a concrete board/target only when its toolchain and hardware definitions are established.

Each firmware project must define target MCU and board revision, build tools, flash procedure, pin assignment, communication protocol, startup behavior and recovery or rollback instructions.

## Interface contracts

Firmware and host software share versioned packet/message definitions, scaling, signedness, timing, checksums or integrity mechanisms, command sequencing, timeout behavior and fault-state semantics. Electrical assumptions and pin mappings must be traceable to the electronics design.

## Control and safety

Protect actuators independently of the operator dashboard. Define power-on and power-loss behavior, watchdogs, communication timeout states, fault latching, calibration safety and command limits as appropriate to the hardware. A host heartbeat cannot be assumed sufficient for every failure mode.

## Verification

Compile against a pinned toolchain and perform unit tests where hardware abstraction allows. Target testing requires an explicit bench setup, current limits, emergency-stop arrangements and authorization. Record firmware version with the applicable electronics and mechanical revisions.

See [Electronics](../hardware/electronics.md) and [Engineering rules](../development/engineering-rules.md).
