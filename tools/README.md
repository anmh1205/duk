# Engineering Tools

`tools/` owns reusable utilities that support engineering processes across subsystems. Examples of tool responsibilities include model generation, physical-parameter validation and checks of supported development environments.

## Design expectations

A maintained tool must define its inputs, outputs, supported formats, error handling, dependency requirements and reproducible invocation. Transformations should preserve provenance, scale/units and coordinate-frame conventions. Destructive operations or overwrites require an explicit user-facing policy.

Tools that generate URDF, meshes, drawings or other derived artifacts must identify the authoritative input and produce deterministic output where feasible. Validation tools should report actionable mismatches rather than silently correcting engineering source material.

## Boundary with scripts and CI

`scripts/` is for ad hoc developer helpers. `.github/workflows/` owns CI/CD execution. A workflow may invoke a maintained tool through its documented interface, but a tool does not become a workflow merely because CI calls it.

See [Parameter governance](../docs/architecture/parameter-governance.md), [Engineering rules](../docs/development/engineering-rules.md) and [Testing](../docs/development/testing.md).
