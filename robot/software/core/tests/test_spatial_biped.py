"""Spatial dynamics must reduce to the existing sagittal model."""

import json

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")
pytest.importorskip("sympy")

from test_planar_biped import GEOMETRY
from duk_core.modeling import build_planar_biped_model, load_leg_geometry_json
from duk_core.modeling.planar_numeric import NumericPlanarBiped
from duk_core.modeling.spatial_biped import SpatialBiped


@pytest.fixture(scope="module")
def spatial():
    fixture_path = GEOMETRY.parents[3] / "research/experiments/duk-v3/synthetic.json"
    data = json.loads(fixture_path.read_text())
    planar = NumericPlanarBiped(
        build_planar_biped_model(load_leg_geometry_json(GEOMETRY)), data["planar"]
    )
    p = data["spatial"]
    return SpatialBiped(planar, p["lateral_mounts_m"], p["transverse_inertias_kg_m2"])


def test_spatial_sagittal_projection(spatial):
    q = np.r_[0.12, 0.42, 0.13, np.deg2rad([92.0, 101.0]), 0.3, -0.24]
    v = np.array([0.12, -0.03, 0.2, 0.5, -0.4, -0.7, 0.6])
    planar = spatial.planar.evaluate(q, v)
    state, lift = spatial.sagittal_state(q, v)
    model = spatial.evaluate(state)
    np.testing.assert_allclose(lift.T @ model.mass @ lift, planar.mass, atol=1e-11)
    np.testing.assert_allclose(lift.T @ model.actuation, planar.input, atol=1e-11)
    np.testing.assert_allclose(
        model.kinematics.contact[:, 0, :] @ lift, planar.contact[2:], atol=1e-11
    )
    np.testing.assert_allclose(
        model.kinematics.contact[:, 2, :] @ lift, planar.contact[:2], atol=1e-11
    )
    np.testing.assert_allclose(model.kinematics.gaps, planar.gaps, atol=1e-11)
    dt = 1e-5
    _, before = spatial.sagittal_state(q - dt * v, v)
    _, after = spatial.sagittal_state(q + dt * v, v)
    drift = (after - before) @ v / (2 * dt)
    np.testing.assert_allclose(
        lift.T @ (model.bias + model.mass @ drift), planar.bias, atol=1e-8
    )
    assert np.linalg.eigvalsh(model.mass)[0] > 0


def test_spatial_task_rates_match_finite_differences(spatial):
    q = np.r_[0.12, 0.42, 0.13, np.deg2rad([92.0, 101.0]), 0.3, -0.24]
    state, _ = spatial.sagittal_state(q, [0.12, -0.03, 0.2, 0.5, -0.4, -0.7, 0.6])
    v = state.velocity.copy()
    v[3:6] += [0.07, 0.01, -0.14]
    step = 1e-5
    before = spatial.kinematics(state.advance(v, -step)).tasks
    after = spatial.kinematics(state.advance(v, step)).tasks
    np.testing.assert_allclose(
        spatial.kinematics(state).task_jacobian @ v,
        (after - before) / (2 * step),
        atol=1e-8,
    )


def test_free_spatial_motion_preserves_energy_to_first_order(spatial):
    from duk_core.modeling.spatial_biped import SpatialState
    from scipy.spatial.transform import Rotation

    q = np.r_[0.0, 0.5, 0.10, np.deg2rad([94.0, 101.0]), 0.3, -0.24]
    initial, _ = spatial.sagittal_state(q, [0.1, -0.05, 0.16, 0.4, -0.3, -0.6, 0.7])
    rotation = Rotation.from_rotvec([0.13, -0.12, 0.18]).as_matrix()
    v = initial.velocity + np.array(
        [0.0, -0.1, 0.0, 0.2, 0.0, -0.15, 0.0, 0.0, 0.0, 0.0]
    )
    state = SpatialState(initial.position, rotation, initial.joints, v)
    dynamics = spatial.evaluate(state)
    acceleration = np.linalg.solve(dynamics.mass, -dynamics.bias)

    def energy(pose):
        terms = spatial.evaluate(pose)
        return 0.5 * pose.velocity @ terms.mass @ pose.velocity + (
            spatial.gravity * spatial.masses @ terms.kinematics.centers[:, 2]
        )

    step = 1e-5
    plus = state.advance(v, step)
    minus = state.advance(v, -step)
    plus = SpatialState(
        plus.position, plus.rotation, plus.joints, v + step * acceleration
    )
    minus = SpatialState(
        minus.position, minus.rotation, minus.joints, v - step * acceleration
    )
    assert (energy(plus) - energy(minus)) / (2 * step) == pytest.approx(0.0, abs=1e-8)


def test_hierarchy_preserves_first_task_and_rejects_invalid_limits(spatial):
    from duk_core.control.whole_body import AccelerationTask, allocate

    q = np.r_[0.0, 0.3, 0.0, np.deg2rad([95.0, 95.0]), 0.0, 0.0]
    state, _ = spatial.sagittal_state(q, np.zeros(7))
    terms = spatial.evaluate(state)
    row = terms.kinematics.task_jacobian[0]
    tasks = [
        AccelerationTask("first", row, 0.2, 1.0),
        AccelerationTask("second", row, -0.2, 1.0),
    ]
    limits = dict(
        friction=np.full(2, 0.6),
        effort_bounds=np.array([5.0, 5.0, 1.5, 1.5]),
        hip_limits=np.deg2rad([78.0, 125.0]),
        hip_speed_limit=7.0,
        lookahead=0.08,
        velocity_stabilization=7.0,
        deadline_s=2.0,
    )
    result = allocate(
        terms, state, tasks, ("rolling", "rolling"), method="hierarchical", **limits
    )
    assert result.status == "solved"
    assert row @ result.solution[:10] == pytest.approx(0.2, abs=2e-6)
    assert result.residual < 2e-6
    impossible = allocate(
        terms,
        state,
        tasks,
        ("rolling", "rolling"),
        method="weighted",
        **(limits | {"hip_limits": (2.8, 2.9)}),
    )
    assert impossible.status == "infeasible" and impossible.effort is None


@pytest.mark.parametrize("fault", ["sensor", "deadline"])
def test_fault_discards_drive_request_before_a_partial_final_step(
    spatial, monkeypatch, fault
):
    from duk_core.control import whole_body_experiment
    from duk_core.modeling.hip_load import LinearTorqueSpeed

    def unexpected_allocation(*args, **kwargs):
        pytest.fail("A rejected sample must not reach the allocator")

    monkeypatch.setattr(whole_body_experiment, "allocate", unexpected_allocation)
    fixture_path = GEOMETRY.parents[3] / "research/experiments/duk-v3/synthetic.json"
    fixture = json.loads(fixture_path.read_text())
    settings = fixture["whole_body"] | {"substeps": 2}
    scenario = settings["scenario"] | {
        "duration_s": 0.007,
        "feature_start_s": 0.0,
        "initial_lean_deg": 0.0,
        "fault": fault,
        "fault_start_s": 0.0,
        "fault_end_s": 0.02,
        "drive_speed_m_s": 0.1,
        "drive_end_s": 0.02,
    }
    limits = fixture["hip_envelope"]
    motor = LinearTorqueSpeed(
        limits["stall_torque_N_m"], limits["no_load_speed_rad_s"], limits["efficiency"]
    )
    trace = whole_body_experiment.run_whole_body(
        spatial,
        motor,
        fixture["v2_balance"],
        settings,
        scenario,
        method="weighted",
        priority="balance_first",
    )
    assert trace.requested_reference[0, 2] == 0.1
    np.testing.assert_allclose(trace.effective_reference[0, 1:], 0.0)
    np.testing.assert_allclose(trace.feature_amount[0], 0.0)
    np.testing.assert_allclose(trace.applied_effort[0, 2:], 0.0, atol=1e-10)
    if fault == "sensor":
        assert trace.command_source == ("stop",)
        assert trace.solver_status == ("skipped_sensor",)
        assert trace.terminal_time == 0.0
    else:
        assert trace.command_source == ("balance_fallback",)
        assert trace.terminal_time == pytest.approx(0.007)
