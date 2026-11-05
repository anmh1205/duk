"""Symbolic model identities; all numeric mass data here are synthetic fixtures."""

import math
from pathlib import Path

import pytest

sp = pytest.importorskip("sympy")

from duk_core.modeling import (  # noqa: E402
    UnreachableConfigurationError,
    build_planar_biped_model,
    load_leg_geometry_json,
    solve_leg_pose,
)

GEOMETRY = Path(__file__).resolve().parents[3] / "config/physical/duk_leg_geometry.json"
STATE = [0.12, 0.5, 0.1, math.radians(75), math.radians(80), 0.2, -0.3]
VELOCITY = [0.11, -0.07, 0.2, 0.3, -0.25, 1, -0.5]


@pytest.fixture(scope="module")
def model():
    return build_planar_biped_model(load_leg_geometry_json(GEOMETRY))


@pytest.fixture(scope="module")
def values(model):
    # Positive fixture values deliberately do not populate any physical config.
    result = {}
    for name in model.parameters:
        if name.startswith("m_"):
            value = 1.5
        elif name.startswith("I_"):
            value = 0.02
        elif name == "g_m_s2":
            value = 9.81
        elif name.startswith("mu_"):
            value = 0.6
        elif name.startswith("b_"):
            value = 0.1
        elif name.startswith("a_"):
            value = 0
        elif name.endswith("_n_m"):
            value = 0.003
        else:
            value = 0.02
        result[name] = value
    return result


@pytest.fixture(scope="module")
def evaluate(model, values):
    params = tuple(model.parameters.values())
    bound = tuple(values)
    expressions = {
        "M": model.mass_matrix,
        "G": model.gravity,
        "D": model.damping_matrix,
        "B": model.input_matrix,
        "J": model.contact_jacobian,
        "h": model.velocity_bias,
        "V": [model.potential_energy],
    }
    functions = {
        name: sp.lambdify(
            (*model.coordinates, *model.velocities, *params),
            list(expr),
            modules="math",
            cse=True,
            docstring_limit=0,
        )
        for name, expr in expressions.items()
    }

    def numeric(name, state=STATE, velocity=VELOCITY):
        return functions[name](*state, *velocity, *(values[n] for n in bound))

    return numeric


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def test_symbolic_points_match_canonical_solver(model):
    for side, coordinate in (("L", model.coordinates[3]), ("R", model.coordinates[4])):
        expressions = [x for p in model.local_points[side].values() for x in p]
        evaluate = sp.lambdify(
            coordinate, expressions, "math", cse=True, docstring_limit=0
        )
        for angle in (75, 85, 95, 115):
            q = math.radians(angle)
            reference = solve_leg_pose(model.geometry, q)
            actual = evaluate(q)
            for index, point in enumerate(("A", "B", "C", "D", "H", "O")):
                assert actual[2 * index : 2 * index + 2] == pytest.approx(
                    [v / 1000 for v in reference.point(point)], abs=1e-12
                )


def test_mass_matrix_is_symmetric_positive_definite(model, values, evaluate):
    assert model.mass_matrix.shape == (7, 7)
    flat = evaluate("M")
    matrix = [flat[i * 7 : (i + 1) * 7] for i in range(7)]
    factor = [[0.0] * 7 for _ in range(7)]
    for i in range(7):
        for j in range(i + 1):
            assert matrix[i][j] == pytest.approx(matrix[j][i], abs=1e-12)
            residual = matrix[i][j] - sum(factor[i][k] * factor[j][k] for k in range(j))
            if i == j:
                assert residual > 0
                factor[i][j] = math.sqrt(residual)
            else:
                factor[i][j] = residual / factor[j][j]
    total_mass = sum(v for n, v in values.items() if n.startswith("m_"))
    assert matrix[0][0] == pytest.approx(total_mass)
    assert matrix[1][1] == pytest.approx(total_mass)
    assert matrix[5][5] == pytest.approx(values["I_wheel_L_kg_m2"])
    assert matrix[6][6] == pytest.approx(values["I_wheel_R_kg_m2"])


def test_gravity_is_potential_gradient(evaluate, values):
    epsilon = 1e-6
    gradient = []
    for index in range(7):
        left, right = list(STATE), list(STATE)
        left[index] -= epsilon
        right[index] += epsilon
        gradient.append(
            (evaluate("V", right)[0] - evaluate("V", left)[0]) / (2 * epsilon)
        )
    assert evaluate("G") == pytest.approx(gradient, abs=2e-8)
    assert gradient[0] == pytest.approx(0)
    assert gradient[1] == pytest.approx(
        sum(v for n, v in values.items() if n.startswith("m_")) * values["g_m_s2"]
    )


def test_velocity_bias_matches_lagrange_equations(evaluate):
    epsilon = 1e-6
    derivatives = []
    for k in range(7):
        left, right = list(STATE), list(STATE)
        left[k] -= epsilon
        right[k] += epsilon
        derivatives.append(
            [
                (upper - lower) / (2 * epsilon)
                for lower, upper in zip(evaluate("M", left), evaluate("M", right))
            ]
        )
    reference = []
    for i in range(7):
        coriolis = sum(
            (derivatives[k][i * 7 + j] - 0.5 * derivatives[i][j * 7 + k])
            * VELOCITY[j]
            * VELOCITY[k]
            for j in range(7)
            for k in range(7)
        )
        reference.append(coriolis)
    assert evaluate("h") == pytest.approx(reference, abs=2e-8)


def test_relative_wheel_reaction_damping_and_contact_sign(model, evaluate):
    inputs = evaluate("B")
    assert inputs[2 * 4 + 2 : 2 * 4 + 4] == [-1, -1]
    assert inputs[5 * 4 + 2] == 1
    assert inputs[6 * 4 + 3] == 1
    damping = evaluate("D")
    dissipated = sum(
        VELOCITY[i] * damping[i * 7 + j] * VELOCITY[j]
        for i in range(7)
        for j in range(7)
    )
    assert dissipated >= 0
    jacobian = evaluate("J")
    radius = model.geometry.wheel_radius_mm / 1000
    assert jacobian[2 * 7 + 5] == pytest.approx(radius)
    assert jacobian[3 * 7 + 6] == pytest.approx(radius)
    assert jacobian[0 * 7 + 5] == 0
    assert len(model.contact_conditions) == 2


def test_unknown_parameters_stay_symbolic_and_invalid_values_fail(model):
    partial = model.mass_matrix.subs(model.bind_parameters({"m_body_kg": 2}))
    assert model.parameters["m_AB_L_kg"] in partial.free_symbols
    assert model.parameters["I_body_kg_m2"] in partial.free_symbols
    assert len(model.bind_configuration(STATE)) == 7
    wrong = list(STATE)
    wrong[3] = -math.radians(75)
    with pytest.raises(UnreachableConfigurationError):
        model.bind_configuration(wrong)
    for values in ({"m_body_kg": 0}, {"b_q_L_N_m_s": -1}, {"g_m_s2": math.nan}):
        with pytest.raises(ValueError):
            model.bind_parameters(values)
    with pytest.raises(KeyError):
        model.bind_parameters({"unknown_mass": 1})
    assert model.bind_parameters({"mu_L": 0})
    with pytest.raises(ValueError):
        model.bind_parameters({"m_body_kg": True})


def test_parameter_template_has_no_assumed_measurements(model):
    import json

    template = json.loads(
        (GEOMETRY.parent / "duk_planar_parameters.example.json").read_text()
    )
    assert template["model"] == "planar_biped_7dof"
    assert template["parameters"].keys() == model.parameters.keys()
    assert all(value is None for value in template["parameters"].values())


def test_link_angular_rows_match_independent_angle_differences(model):
    delta = 1e-6
    for body in model.bodies:
        if body.name == "body" or body.name.startswith("wheel"):
            continue
        label, side = body.name.split("_")
        index = 3 if side == "L" else 4
        angle = STATE[index]
        previous = solve_leg_pose(model.geometry, angle - delta)
        following = solve_leg_pose(model.geometry, angle + delta)

        def orientation(pose):
            start, end = {
                "AB": (pose.a, pose.b),
                "CD": (pose.d, pose.c),
                "BCO": (pose.b, pose.c),
            }[label]
            return math.atan2(end[1] - start[1], end[0] - start[0])

        difference = (orientation(following) - orientation(previous) + math.pi) % (
            2 * math.pi
        ) - math.pi
        actual = float(
            body.angular_jacobian[index].subs(model.bind_configuration(STATE))
        )
        assert actual == pytest.approx(difference / (2 * delta), abs=1e-8)
        assert body.angular_jacobian[2] == 1


def test_contact_acceleration_bias_matches_constraint_difference(model, values):
    args = (*model.coordinates, *model.velocities, *model.parameters.values())
    evaluate = sp.lambdify(
        args, list(model.contact_acceleration_bias), "math", cse=True, docstring_limit=0
    )
    jacobian = sp.lambdify(
        args, list(model.contact_jacobian), "math", cse=True, docstring_limit=0
    )
    physical = tuple(values.values())
    dt = 1e-6
    previous = [q - dt * rate for q, rate in zip(STATE, VELOCITY)]
    following = [q + dt * rate for q, rate in zip(STATE, VELOCITY)]
    left = jacobian(*previous, *VELOCITY, *physical)
    right = jacobian(*following, *VELOCITY, *physical)
    expected = [
        sum(
            (right[i * 7 + j] - left[i * 7 + j]) / (2 * dt) * VELOCITY[j]
            for j in range(7)
        )
        for i in range(4)
    ]
    assert evaluate(*STATE, *VELOCITY, *physical) == pytest.approx(expected, abs=1e-9)
