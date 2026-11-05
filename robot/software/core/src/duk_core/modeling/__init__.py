"""Mathematical models and geometric helpers for DUK."""

from .leg_linkage import (
    BCORigidBodyGeometry,
    KinematicSingularityError,
    LegGeometry,
    LegJacobian,
    LegPose,
    LinkageError,
    Side,
    UnreachableConfigurationError,
    geometry_from_mapping,
    leg_jacobian,
    load_leg_geometry_json,
    sample_leg_trajectory,
    solve_leg_pose,
)
from .mass_properties import (
    CombinedMassProperties,
    EquivalentPendulum2D,
    PlanarRigidBody,
    combine_planar_mass_properties,
    equivalent_pendulum_2d,
)

__all__ = [
    "BCORigidBodyGeometry",
    "CombinedMassProperties",
    "EquivalentPendulum2D",
    "KinematicSingularityError",
    "LegGeometry",
    "LegJacobian",
    "LegPose",
    "LinkageError",
    "PlanarRigidBody",
    "Side",
    "UnreachableConfigurationError",
    "combine_planar_mass_properties",
    "equivalent_pendulum_2d",
    "geometry_from_mapping",
    "leg_jacobian",
    "load_leg_geometry_json",
    "sample_leg_trajectory",
    "solve_leg_pose",
]
