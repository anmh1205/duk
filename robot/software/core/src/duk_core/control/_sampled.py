"""Time grids shared by the sampled controller studies."""

from math import ceil, nextafter

import numpy as np


def sample_intervals(duration, period):
    """Include a partial last interval, folding away a one-ULP rounding tail."""
    count = max(1, ceil(nextafter(duration / period, 0.0)))
    starts = np.arange(count) * period
    return starts, np.r_[starts[1:], duration]
