# Mathematical Modeling and Control Theory

Theory documentation defines the assumptions, coordinates, symbols and derivations that justify DUK's computational models. Reusable numerical implementations belong in `robot/software/core/src/duk_core/`, not in duplicated equations embedded only in notebooks.

For each model, document generalized coordinates, frames, physical parameters, dynamics/contact assumptions, reduction steps, applicable operating envelope and verification method. Distinguish symbolic derivation, numerical approximation, linearization and measured/system-identified quantities.

Control derivations should specify continuous/discrete time, sampling period, state/reference definitions, actuator limits, stability assumptions and validity range. Cite foundational literature or applicable source material for nontrivial models.

Models not yet derived or verified should not be presented as validated DUK dynamics. See [Core architecture](../software/core.md) and [Data contracts](../architecture/data-contracts.md).
