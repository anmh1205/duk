# Physical configuration

This directory owns established geometry and physical-data templates for DUK.

## Established leg geometry

[`duk_leg_geometry.json`](duk_leg_geometry.json) is the single source of leg
dimensions, topology and coordinate conventions. Lengths use mm and stored angles
use degrees. The core converts angles to radians.

- A is the origin, (0,0).
- +x points right and +y points up; CCW is positive.
- Directed D→A has angle −30°; A→D is its opposite.
- q is the directed CCW angle AB→DA, not a calibrated motor encoder angle.

$$
\theta_{AB}=\theta_{DA}-q,\qquad
D=-AD[\cos\theta_{DA},\sin\theta_{DA}]^T,\qquad
B=AB[\cos\theta_{AB},\sin\theta_{AB}]^T.
$$

C is the intersection of circles (B, BC) and (D, CD), selected by the configured
physical left/right topology. With e=(C−B)/BC and N the CCW 90° operator:

$$
H=B-BH\,e,\qquad O=H+s_O OH\,Ne.
$$

B/C/O are one rigid carrier; H is an auxiliary construction point. BC, BO, CO
and signed B–C–O area are invariant. No solved coordinates, display-time mirror,
solver-frame rotation or post-solution correction are stored in physical config.

## Unidentified planar parameters

[`duk_planar_parameters.example.json`](duk_planar_parameters.example.json) is an
**unidentified template**, not measured robot physics. All entries are `null`.
The symbolic model keeps omitted/null values as variables. Copy it to ignored
`.local/planar-parameters.json` to try identified values locally; never replace an
unknown mass/inertia with zero just to make a simulation run.

The [modeling reference](../../../docs/theory/modeling.md) defines the SI units,
link CoM frames, generalized coordinates and candidate dynamics assumptions.
Preview body dimensions/colors are not evidence for mass, inertia or mount data.
