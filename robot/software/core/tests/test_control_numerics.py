"""Independent quadratic minimizers and malformed controller problem data."""

from dataclasses import replace

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from duk_core.control.height_balance import (
    HeightSample,
    height_mpc_command,
    height_torque_interval,
)
from duk_core.control.lqr import (
    bryson_weights,
    discrete_lqr,
    finite_horizon_lqr,
    linearize,
)


def scalar_sample(**changes):
    return replace(
        HeightSample(
            (0.0, 0.0, 0.0),
            np.zeros(1),
            0.0,
            np.ones((1, 1)),
            np.ones((1, 1)),
            np.empty(0),
            np.empty((0, 1)),
            np.empty(0),
        ),
        **changes,
    )


def quadratic_minimizer(stages, q, r, terminal, initial):
    """Recover the full quadratic from its values, without a Riccati recursion."""
    count, inputs = len(stages), stages[0][1].shape[1]

    def cost(commands):
        state, value = initial.copy(), 0.0
        for (a, b), effort in zip(stages, commands.reshape(count, inputs)):
            value += state @ q @ state + effort @ r @ effort
            state = a @ state + b @ effort
        return value + state @ terminal @ state

    basis = np.eye(count * inputs)
    zero = cost(np.zeros(count * inputs))
    positive = np.array([cost(unit) for unit in basis])
    negative = np.array([cost(-unit) for unit in basis])
    gradient = (positive - negative) / 2
    hessian = np.empty((count * inputs, count * inputs))
    for i, unit_i in enumerate(basis):
        for j, unit_j in enumerate(basis):
            hessian[i, j] = cost(unit_i + unit_j) - positive[i] - positive[j] + zero
    np.fill_diagonal(hessian, positive + negative - 2 * zero)
    return np.linalg.solve(hessian, -gradient).reshape(count, inputs)


def test_time_varying_multi_input_lqr_matches_the_full_quadratic_minimum():
    stages = [
        (np.array([[1.0, 0.1], [0.0, 0.95]]), np.array([[0.1, 0.0], [0.1, 0.2]])),
        (np.array([[0.9, 0.2], [0.0, 1.1]]), np.array([[0.2, 0.1], [0.0, 0.1]])),
        (np.array([[1.0, 0.3], [-0.1, 0.8]]), np.array([[0.1, 0.0], [0.0, 0.3]])),
    ]
    q, r, terminal = np.diag([2.0, 0.5]), np.diag([0.3, 0.7]), np.diag([3.0, 1.0])
    initial = np.array([0.2, -0.1])
    expected = quadratic_minimizer(stages, q, r, terminal, initial)
    state, commands = initial.copy(), []
    for (a, b), gain in zip(stages, finite_horizon_lqr(stages, q, r, terminal)):
        command = -gain @ state
        commands.append(command)
        state = a @ state + b @ command
    np.testing.assert_allclose(commands, expected, atol=1e-12, rtol=0)


def test_unconstrained_mpc_accepts_empty_force_rows_and_matches_a_full_minimum():
    stages = [
        (np.array([[1.0, 0.2], [0.0, 0.95]]), np.array([[0.02], [0.2]])),
        (np.array([[0.9, 0.1], [-0.1, 1.0]]), np.array([[0.1], [0.15]])),
        (np.array([[1.1, 0.1], [0.0, 0.8]]), np.array([[0.03], [0.3]])),
    ]
    samples = [
        HeightSample(
            (0.0, 0.0, 0.0),
            np.zeros(2),
            0.0,
            a,
            b,
            np.empty(0),
            np.empty((0, 2)),
            np.empty(0),
        )
        for a, b in stages
    ]
    q, r, terminal = np.diag([2.0, 0.5]), np.array([[0.3]]), np.diag([3.0, 1.0])
    error = np.array([0.2, -0.1])
    expected = quadratic_minimizer(stages, q, r, terminal, error)[0, 0]
    actual = height_mpc_command(samples, error, q, r, terminal, 100.0)
    assert actual == pytest.approx(expected, abs=1e-4)


def test_one_step_mpc_obeys_motor_and_force_bounds_with_nonzero_feedforward():
    sample = scalar_sample(
        nominal_effort=0.1,
        constraint_value=np.array([-0.3]),
        constraint_state=np.zeros((1, 1)),
        constraint_effort=np.array([-1.0]),
    )
    # Unconstrained correction is -2/(0.25+2). The force row requires du >= -0.3;
    # the total motor bound requires -0.5 <= du <= 0.3, so the optimum is -0.3.
    command = height_mpc_command(
        [sample], np.ones(1), np.eye(1), np.array([[0.25]]), 2 * np.eye(1), 0.4
    )
    assert command == pytest.approx(-0.3, abs=1e-8)
    assert abs(sample.nominal_effort + command) <= 0.4


@pytest.mark.parametrize("constant", [-1.0, 1.0])
def test_mpc_distinguishes_redundant_and_impossible_constant_constraints(constant):
    sample = scalar_sample(
        constraint_value=np.array([constant]),
        constraint_state=np.zeros((1, 1)),
        constraint_effort=np.zeros(1),
    )
    command = height_mpc_command(
        [sample], np.ones(1), np.eye(1), np.eye(1), np.eye(1), 10.0
    )
    if constant > 0:
        assert command is None
    else:
        assert command == pytest.approx(-0.5, abs=1e-8)


def test_zero_state_cost_on_a_stable_plant_has_a_finite_zero_residual():
    with np.errstate(divide="raise", invalid="raise"):
        design = discrete_lqr([[-1.0]], [[1.0]], [[0.0]], [[1.0]], 0.1)
    assert design.riccati_residual == 0.0
    assert design.gain[0, 0] == 0.0
    assert design.spectral_radius == pytest.approx(np.exp(-0.1))


@pytest.mark.parametrize(
    "q,r,terminal",
    [
        (-np.eye(2), np.eye(1), np.eye(2)),
        (np.eye(2), -np.eye(1), np.eye(2)),
        (np.eye(2), np.zeros((1, 1)), np.eye(2)),
        (np.array([[1.0, 1.0], [0.0, 1.0]]), np.eye(1), np.eye(2)),
        (np.eye(2), np.eye(1), -np.eye(2)),
        (np.eye(3), np.eye(1), np.eye(2)),
        (np.eye(2), np.eye(2), np.eye(2)),
    ],
)
def test_invalid_costs_are_rejected_by_both_finite_horizon_controllers(q, r, terminal):
    with pytest.raises(ValueError):
        finite_horizon_lqr([(np.eye(2), np.ones((2, 1)))], q, r, terminal)
    sample = scalar_sample(
        a=np.eye(2), b=np.ones((2, 1)), constraint_state=np.empty((0, 2))
    )
    with pytest.raises(ValueError):
        height_mpc_command([sample], np.zeros(2), q, r, terminal, 1.0)


@pytest.mark.parametrize("scales", [[], [[1.0, 1.0]], [1e-250], [1e250]])
def test_bryson_scales_must_form_representable_nonempty_diagonal_costs(scales):
    with pytest.raises(ValueError):
        bryson_weights(scales, 1.0)


def test_bryson_scales_match_unit_normalized_costs():
    scales = np.array([0.2, 3.0])
    q, r = bryson_weights(scales, 0.5)
    np.testing.assert_allclose(np.diag(scales) @ q @ np.diag(scales), np.eye(2))
    assert r[0, 0] * 0.5**2 == pytest.approx(1.0)


@pytest.mark.parametrize(
    "base,slope,bound",
    [
        ([-1.0, 1.0], [0.0], 3.0),
        ([[1.0]], [[1.0]], 3.0),
        ([-1.0], [1.0], -1.0),
        ([-1.0], [1.0], np.nan),
    ],
)
def test_affine_torque_interval_cannot_omit_malformed_rows(base, slope, bound):
    with pytest.raises(ValueError):
        height_torque_interval(base, slope, bound)


def test_affine_torque_interval_with_no_force_rows_keeps_the_motor_bound():
    assert height_torque_interval([], [], 3.0) == (-3.0, 3.0)
    assert height_torque_interval([1.0, 0.0], [1.0, -1.0], 3.0) is None


@pytest.mark.parametrize(
    "changes",
    [
        {"a": np.eye(2)},
        {"constraint_value": np.zeros(1)},
        {
            "constraint_value": np.array([np.nan]),
            "constraint_state": np.zeros((1, 1)),
            "constraint_effort": np.zeros(1),
        },
        {"nominal_effort": np.inf},
    ],
)
def test_mpc_rejects_malformed_stage_data(changes):
    with pytest.raises(ValueError):
        height_mpc_command(
            [scalar_sample(**changes)], np.ones(1), np.eye(1), np.eye(1), np.eye(1), 1.0
        )


def test_mpc_requires_a_nonempty_horizon():
    with pytest.raises(ValueError, match="nonempty horizon"):
        height_mpc_command([], np.ones(1), np.eye(1), np.eye(1), np.eye(1), 1.0)


def test_linearization_accepts_array_like_outputs_and_matches_an_affine_map():
    a = np.array([[2.0, -1.0], [0.5, 3.0], [1.0, 0.0]])
    b = np.array([0.5, -2.0, 1.0])
    actual_a, actual_b = linearize(lambda x, u: list(a @ x + b * u), [0.2, -0.1], 0.3)
    np.testing.assert_allclose(actual_a, a, atol=1e-10, rtol=0)
    np.testing.assert_allclose(actual_b[:, 0], b, atol=1e-10, rtol=0)


def test_linearization_preserves_evaluations_from_a_reused_output_buffer():
    a = np.array([[2.0, -1.0], [0.5, 3.0]])
    b = np.array([0.5, -2.0])
    buffer = np.zeros(2)

    def evaluate(x, u):
        buffer[:] = a @ x + b * u
        return buffer

    actual_a, actual_b = linearize(evaluate, [0.2, -0.1], 0.3)
    np.testing.assert_allclose(actual_a, a, atol=1e-10, rtol=0)
    np.testing.assert_allclose(actual_b[:, 0], b, atol=1e-10, rtol=0)


@pytest.mark.parametrize("step", [0.0, -1.0, np.nan, np.inf])
def test_linearization_requires_a_positive_finite_difference_step(step):
    with pytest.raises(ValueError, match="positive step"):
        linearize(lambda x, _: x, np.ones(2), 0.0, step=step)
