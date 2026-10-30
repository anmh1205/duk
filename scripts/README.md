# Development Scripts

`scripts/` contains short, local helpers used during engineering work: one-off inspections, temporary conversions, quick instrumentation and other ad hoc developer tasks. It is not a second automation framework.

A script may be committed when it is useful for reproducing a small engineering operation and has no hidden credentials or environment-specific assumptions. Keep its invocation and required dependencies understandable.

Move maintained, reusable engineering programs into `tools/`, with a documented interface and appropriate tests. Official CI and release automation belong in `.github/workflows/`; CI must not silently depend on temporary developer scripts.

For repository-wide placement rules, see [Repository layout](../docs/architecture/repository-layout.md).
