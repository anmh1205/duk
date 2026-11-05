# Physical Configuration

This directory owns measured, CAD-derived or otherwise established physical parameters used by DUK software models.

## Planar leg frame convention

The planar leg model uses two different frames for two different purposes. They must not be merged or corrected ad hoc in visualization code.

### Robot frame

The robot-frame convention is:

- (+O_x): right;
- (+O_y): up;
- positive rotation: counter-clockwise;
- planar origin for the current leg model: point A;
- reference directed vector: **D→A**;
- directed angle of D→A from (+O_x): **-30°**.

Therefore D is above A.

Because A→D is the opposite directed vector,

[
	heta_{AD}=	heta_{DA}+180^circ=150^circ.
]

The configuration stores only the primary physical statement, `da_angle_deg = -30`. Software derives the A→D direction from it.

### Solver frame

The four-bar numerical solver uses:

- A = (0, 0);
- D = (AD, 0);
- local +x aligned with A→D;
- positive rotation counter-clockwise.

This solver frame is chosen only to simplify loop closure. It is transformed into the robot frame by rotating by the derived A→D angle, currently +150°.

No separate 90° display correction and no independent AD-orientation parameter are part of the model.

## Current leg geometry source

`duk_leg_geometry.json` contains the established linkage lengths, wheel-center offsets, wheel radius and the robot-frame convention. Unresolved values such as the physical four-bar assembly branch remain explicitly unset rather than being guessed.
