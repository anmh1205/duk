"""Analytical ideal loading and optional illustrative motor envelopes."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LinearTorqueSpeed:
    """Candidate symmetric linear envelope, with torque expressed on the input side."""

    stall_torque: float
    no_load_speed: float
    efficiency: float

    def __post_init__(self):
        if not all(
            math.isfinite(x) and x > 0 for x in (self.stall_torque, self.no_load_speed)
        ):
            raise ValueError(
                "Stall torque and no-load speed must be finite and positive"
            )
        if not 0 < self.efficiency <= 1:
            raise ValueError("Efficiency must be in (0, 1]")

    def joint_limit(self, joint_speed: float) -> float:
        """Input torque already referred to joint gearing, before efficiency."""
        if not math.isfinite(joint_speed):
            raise ValueError("Joint speed must be finite")
        return (
            self.stall_torque
            * self.efficiency
            * max(0.0, 1 - abs(joint_speed) / self.no_load_speed)
        )


def ideal_symmetric_load_kg(
    vertical_jacobian_m: float, joint_torque: float, gravity: float
) -> float:
    """Static centered payload with massless legs, valid away from zero leverage."""
    if not all(math.isfinite(x) for x in (vertical_jacobian_m, joint_torque, gravity)):
        raise ValueError("Ideal load inputs must be finite")
    if abs(vertical_jacobian_m) < 1e-12 or joint_torque < 0 or gravity <= 0:
        raise ValueError(
            "Ideal load requires nonzero leverage, nonnegative torque and positive gravity"
        )
    return 2.0 * joint_torque / (gravity * abs(vertical_jacobian_m))
