"""Unilateral support, friction cones and passivity under batched input shapes."""

import pytest

np = pytest.importorskip("numpy")

from duk_core.modeling.contact_dynamics import CompliantContact, rigid_rolling
from duk_core.modeling.planar_numeric import PlanarDynamics


def test_rigid_contact_rejects_invalid_force_data_before_solving():
    # Independent contact rows with zero reaction isolate validation from
    # physical infeasibility: negative friction previously passed this state.
    dynamics = PlanarDynamics(
        np.eye(7),
        np.zeros(7),
        np.eye(7)[:, :4],
        np.zeros(2),
        np.eye(7)[[0, 1, 2, 3]],
        np.zeros(4),
        0.0,
        np.zeros(7),
    )
    for friction in (-1.0, np.nan, np.inf, [[0.5, 0.5]]):
        with pytest.raises(ValueError, match="Friction"):
            rigid_rolling(dynamics, np.zeros(4), friction)
    for tolerance in (-1.0, np.nan, np.inf):
        with pytest.raises(ValueError, match="tolerance"):
            rigid_rolling(dynamics, np.zeros(4), 0.5, tolerance=tolerance)
    for effort in (np.zeros(3), [0.0, np.nan, 0.0, 0.0]):
        with pytest.raises(ValueError, match="Effort"):
            rigid_rolling(dynamics, effort, 0.5)
    for friction in (0.0, [0.0, 0.5]):
        assert rigid_rolling(dynamics, np.zeros(4), friction, tolerance=0).admissible


@pytest.mark.parametrize("dimensions", [1, 2])
@pytest.mark.parametrize("seed", range(4))
def test_regularized_contact_never_pulls_or_adds_tangential_energy(dimensions, seed):
    rng = np.random.default_rng(seed)
    gaps = rng.uniform(-0.002, 0.002, (3, 4))
    speed = rng.normal(0, 0.1, (3, 4))
    tangent = rng.normal(size=(3, 4, dimensions))
    friction = rng.uniform(0.0, 1.0, (3, 4))
    law = CompliantContact(1000.0, 10.0, 0.01)
    normal, force = law.forces(gaps, speed, tangent, friction)
    assert np.all(normal >= 0)
    np.testing.assert_array_equal(normal[gaps > 0], 0.0)
    assert np.all(np.sum(force * tangent, axis=-1) <= 1e-12)
    assert np.all(np.linalg.norm(force, axis=-1) <= friction * normal + 1e-12)


def test_contact_accepts_lists_and_broadcasts_one_tangent_across_contacts():
    law = CompliantContact(1000.0, 10.0, 0.01)
    normal, force = law.forces([-0.001, 0.001], [0.0, 0.0], [0.2, 0.0], 0.5)
    np.testing.assert_allclose(normal, [1.0, 0.0])
    np.testing.assert_allclose(force, [[-0.5, 0.0], [0.0, 0.0]], atol=1e-12)


@pytest.mark.parametrize("friction", [-0.1, np.nan, np.inf])
def test_friction_cannot_reverse_dissipation_or_introduce_nonfinite_force(friction):
    with pytest.raises(ValueError, match="nonnegative friction"):
        CompliantContact(1000.0, 10.0, 0.01).forces(-0.001, 0.0, [0.2], friction)


@pytest.mark.parametrize(
    "gaps,normal,tangent",
    [
        ([np.nan], [0.0], [[0.2]]),
        ([-0.001], [np.inf], [[0.2]]),
        ([-0.001], [0.0], [[np.nan]]),
        ([-0.001], [0.0], [0.1, 0.2, 0.3]),
        ([-0.001], [0.0], 0.2),
    ],
)
def test_contact_rejects_nonfinite_or_undefined_tangent_channels(gaps, normal, tangent):
    with pytest.raises(ValueError, match="finite measurements"):
        CompliantContact(1000.0, 10.0, 0.01).forces(gaps, normal, tangent, 0.5)


def test_contact_rejects_incompatible_contact_batches():
    with pytest.raises(ValueError, match="dimensions must broadcast"):
        CompliantContact(1000.0, 10.0, 0.01).forces(
            np.zeros(2), np.zeros(2), np.zeros((3, 2)), 0.5
        )
