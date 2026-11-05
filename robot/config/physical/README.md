# Physical Configuration

This directory owns established physical dimensions, frame conventions and topology for DUK software models.

## Canonical planar robot frame

The planar leg uses one physical coordinate frame:

- origin: point A;
- +Ox points right;
- +Oy points up;
- positive rotation is counter-clockwise;
- directed D→A angle is -30° from +Ox.

Only point A is fixed numerically:

[
A=(0,0).
]

No solved coordinate for B, C, D, H or O is stored in configuration.

## Geometry equations

The opposite directed-link angle is derived:

[
	heta_{AD}=	heta_{DA}+pi.
]

Point D is therefore:

[
D=A+AD
egin{bmatrix}
cos	heta_{AD}\
sin	heta_{AD}
end{bmatrix}.
]

The one-degree-of-freedom linkage coordinate is (q), defined as the directed counter-clockwise angle from (overrightarrow{AB}) to (overrightarrow{DA}):

[
q=angle(overrightarrow{AB},overrightarrow{DA}).
]

Hence:

[
	heta_{AB}=	heta_{DA}-q
]

and:

[
B=A+AB
egin{bmatrix}
cos	heta_{AB}\
sin	heta_{AB}
end{bmatrix}.
]

Point C is not stored. It is the physical intersection of:

- the circle centered at B with radius BC;
- the circle centered at D with radius CD.

The physical intersection is selected by named topology: B is on the configured side of directed A→D and C is on the configured side of directed B→D. Numeric branch signs are an implementation detail, not physical configuration.

## Rigid BCO wheel carrier

B, C and O are fixed material points on one rigid carrier. H is an auxiliary construction point.

For every solved pose:

[
mathbf e_{BC}=rac{C-B}{BC}
]

[
H=B-BH,mathbf e_{BC}
]

and, using the unit normal (mathbf n_{BC}) on the configured physical side of B→C,

[
O=H+OH,mathbf n_{BC}.
]

Therefore BC, BO, CO and the signed B-C-O area remain invariant through motion.

## Model boundary

The physical config stores dimensions and relations, not corrected coordinates. There is no display-time mirror, solver-frame rotation, or post-solution point correction in the canonical model.

The coordinate (q) is currently a geometric linkage coordinate. It is not yet declared to be a calibrated motor encoder angle.
