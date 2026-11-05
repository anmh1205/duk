# Robotics Core

> **Status:** Canonical kinematic foundation implemented; dynamic and control layers remain under development.

The robotics core is the ROS-independent mathematical source for DUK geometry and motion primitives.

## Coordinate authority

The planar leg has one canonical robot frame:

- A is the origin;
- +Ox right;
- +Oy up;
- counter-clockwise positive;
- D→A = -30°.

Only A is a fixed coordinate. D, B, C, H and O are derived from dimensions, directed angles and topology.

The model does not use a separate solver frame followed by rotation/reflection. It also does not store corrected display coordinates.

## Kinematic construction

    physical dimensions + named topology
                  |
                  v
              A = (0, 0)
                  |
           +------+------+
           |             |
           v             v
    D from AD,theta  B from AB,alpha
           |             |
           +------v------+
                  |
       circle intersection
         |BC| and |CD|
     + physical side of B→D
                  |
                  v
                  C
                  |
                  v
       rigid BCO body basis
                  |
            +-----+-----+
            v           v
            H           O
                  |
                  v
       pose / trajectory / Jacobian

For the current physical convention:

theta_DA = -30°, theta_AD = 150°.

The current configuration variable alpha is the absolute A→B angle from +Ox.

## Rigid BCO carrier

B, C and O are fixed points of one rigid carrier. BC, BO, CO, signed area and instantaneous rigid-body distance constraints are tested over motion.

## Source locations

- robot/config/physical/duk_leg_geometry.json
- robot/config/physical/README.md
- robot/software/core/src/duk_core/modeling/leg_linkage.py
- robot/software/core/src/duk_core/visualization/leg_linkage.py
- robot/software/core/examples/duk_leg_kinematics.py
- robot/software/core/notebooks/kinematics/duk-leg-kinematics.ipynb
- robot/software/core/notebooks/motion/duk-leg-motion.ipynb
- robot/software/core/tests/

The next layers should reuse this model for inverse kinematics, two-leg coordination, body leveling and later dynamics instead of creating alternate coordinate conventions.
