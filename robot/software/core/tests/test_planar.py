"""Unit tests for the generic planar helper (not a DUK robot model)."""

from math import pi

import pytest

from duk_core.modeling.planar import forward_chain


def test_two_link_right_angle() -> None:
    points = forward_chain((1.0, 0.5), (0.0, pi / 2))
    expected = ((0.0, 0.0), (1.0, 0.0), (1.0, 0.5))
    assert len(points) == len(expected)
    for observed, reference in zip(points, expected):
        assert observed == pytest.approx(reference)


def test_relative_joint_angles_accumulate() -> None:
    points = forward_chain((1.0, 1.0), (pi / 2, -pi / 2))
    assert points[-1] == pytest.approx((1.0, 1.0))


@pytest.mark.parametrize(
    ("lengths", "angles"),
    [
        ((), ()),
        ((1.0,), ()),
        ((0.0,), (0.0,)),
        ((-1.0,), (0.0,)),
        ((float("nan"),), (0.0,)),
        ((1.0,), (float("inf"),)),
    ],
)
def test_invalid_input_rejected(lengths, angles) -> None:
    with pytest.raises(ValueError):
        forward_chain(lengths, angles)
