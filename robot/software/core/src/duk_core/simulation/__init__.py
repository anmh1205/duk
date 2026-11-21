"""Numerical experiment orchestration over models, controllers and estimators.

Import a study module explicitly to load its optional research dependencies.
"""


def sample_times(duration: float, period: float) -> list[float]:
    """Return observation times from zero through duration, with a partial final interval."""
    import numpy as np

    from ._sampled import sample_intervals

    if not np.isfinite([duration, period]).all() or min(duration, period) <= 0:
        raise ValueError("Duration and sample period must be finite and positive")
    _, ends = sample_intervals(duration, period)
    return [0.0, *ends.tolist()]
