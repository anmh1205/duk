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

`duk_leg_geometry.json` contains the established linkage lengths, the rigid BCO wheel-carrier geometry, wheel radius, robot-frame convention and the physical four-bar assembly branch. The CAD reference establishes `assembly_branch = -1`: in this branch B→C has the same topology as the mechanical assembly and the H/O wheel-center construction lies on the wheel side of the linkage. The opposite branch remains available only as a mathematical comparison override.


## Rigid BCO wheel carrier

B, C and O are not three independently solved points. They are fixed material points of one rigid carrier link.

The configuration groups that geometry under `bco_rigid_body`:

- (BC = 42) mm;
- (BH = 119) mm;
- (OH = 55) mm;
- `wheel_side = -1`.

With a body-fixed frame at B whose +x axis points B→C and whose +y axis is the left normal of B→C:

[
B=(0,0),qquad
C=(BC,0),qquad
H=(-BH,0),qquad
O=(-BH,;wheel_sidecdot OH).
]

Therefore the triangle side lengths (BC), (BO) and (CO), plus its signed area/orientation, are invariant through all motion. H is only a construction point; it is not a mechanical joint or additional degree of freedom.
