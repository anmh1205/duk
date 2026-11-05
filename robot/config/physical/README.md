# Physical Configuration

This directory owns established physical parameters and topology for DUK software models.

## Canonical planar robot frame

The leg model uses one physical coordinate frame:

- origin: point A;
- +Ox points right;
- +Oy points up;
- positive rotation is counter-clockwise;
- directed D→A angle is -30° from +Ox.

Only point A is fixed numerically:

A = (0, 0).

All other points are derived.

Point D is computed from AD and the derived direction A→D:

theta_AD = theta_DA + 180° = 150°,

D = A + AD [cos(theta_AD), sin(theta_AD)].

Point B is computed from the current absolute A→B angle alpha:

B = A + AB [cos(alpha), sin(alpha)].

Point C is the physical intersection of the circle centered at B with radius BC and the circle centered at D with radius CD. The physical topology is expressed by named sidedness, not a numeric branch sign:

- B lies on the right side of directed A→D;
- C lies on the right side of directed B→D.

There is no solver-frame-to-robot-frame rotation, display-time mirror or post-solution coordinate correction.

## Rigid BCO wheel carrier

B, C and O are fixed material points on one rigid carrier. H is an auxiliary construction point.

The physical configuration stores:

- BC = 42 mm;
- BH = 119 mm;
- OH = 55 mm;
- O lies on the left side of directed B→C.

For every solved pose, the B→C unit vector is derived from B and C. Then H and O are computed from that moving rigid-body basis. Therefore BC, BO, CO and the signed B-C-O triangle area remain invariant through motion.

## Configuration variable

The current kinematic coordinate is the absolute directed angle of A→B measured from robot +Ox. It is a geometric coordinate only; it is not yet declared to be a calibrated motor encoder angle.

duk_leg_geometry.json stores dimensions, frame convention and named topology. It does not store solved B/C/D/H/O coordinates.
