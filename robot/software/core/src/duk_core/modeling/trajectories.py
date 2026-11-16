"""Prescribed joint trajectories for numerical studies."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class HipTrajectory:
    """Quintic angle path (rad, seconds) with zero endpoint speed/acceleration."""

    initial: float
    final: float
    duration: float

    def __post_init__(self):
        if (
            not np.isfinite([self.initial, self.final, self.duration]).all()
            or self.duration <= 0
        ):
            raise ValueError(
                "Hip trajectory requires finite endpoints and positive duration"
            )

    def evaluate(self, time: float) -> tuple[float, float, float]:
        t = np.clip(time / self.duration, 0.0, 1.0)
        delta = self.final - self.initial
        position = self.initial + delta * t**3 * (10 - 15 * t + 6 * t**2)
        velocity = delta / self.duration * 30 * t**2 * (1 - t) ** 2
        acceleration = delta / self.duration**2 * 60 * t * (1 - t) * (1 - 2 * t)
        return position, velocity, acceleration
