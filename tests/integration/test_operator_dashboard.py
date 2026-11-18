"""Verify HTTP inputs reach the real core and return unmodified simulation traces."""

import json
import math
import os
import subprocess
import sys
from dataclasses import fields
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from duk_core.modeling.leg_linkage import (
    leg_jacobian,
    load_leg_geometry_json,
    solve_leg_pose,
)

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def dashboard():
    pytest.importorskip("scipy")
    python_path = [str(ROOT / "robot/software/core/src")]
    if inherited := os.environ.get("PYTHONPATH"):
        python_path.append(inherited)
    with subprocess.Popen(
        [
            sys.executable,
            str(ROOT / "applications/operator-dashboard/server.py"),
            "--port",
            "0",
        ],
        cwd=ROOT,
        env=os.environ
        | {"OPENBLAS_NUM_THREADS": "1", "PYTHONPATH": os.pathsep.join(python_path)},
        stdout=subprocess.PIPE,
        text=True,
    ) as server:
        try:
            line = server.stdout.readline()
            assert line.startswith("DUK Simulation Lab: ")
            yield line.strip().split(" ", 3)[-1]
        finally:
            server.terminate()
            server.wait(timeout=10)


def api(server, path, payload=None):
    request = Request(server + "/api/" + path)
    if payload is not None:
        request.add_header("Content-Type", "application/json")
        request.data = json.dumps(payload).encode()
    with urlopen(request, timeout=10) as response:
        return json.load(response)


def test_dashboard_serves_app_and_canonical_asymmetric_poses(dashboard):
    with urlopen(dashboard, timeout=10) as response:
        assert "<title>DUK Simulation Lab</title>" in response.read().decode()
    config = api(dashboard, "config")
    lower, upper = config["kinematics"]["bounds_deg"]
    response = api(dashboard, "kinematics", {"left_deg": lower, "right_deg": upper})
    geometry = load_leg_geometry_json(
        ROOT / "robot/config/physical/duk_leg_geometry.json"
    )
    for leg, degree in zip(response["legs"], (lower, upper)):
        direct = solve_leg_pose(geometry, math.radians(degree))
        assert leg["q_deg"] == degree
        assert leg["points_mm"]["O"] == pytest.approx(direct.o)
        assert leg["wheel_jacobian_mm_rad"] == pytest.approx(
            leg_jacobian(geometry, direct).d_o
        )
        assert leg["closure_error_mm"] < 1e-9


def test_dashboard_replays_nb4_and_preserves_early_stop(dashboard):
    import numpy as np

    from duk_core.control.lqr import bryson_weights, discrete_lqr
    from duk_core.modeling.wheeled_pendulum import WheeledPendulum
    from duk_core.simulation.balance import simulate_balance

    config = api(dashboard, "config")
    dry = api(dashboard, "balance", config["presets"]["dry"])
    low = api(dashboard, "balance", {"friction": 0.3})
    limited = api(dashboard, "balance", config["presets"]["low_grip_limited"])
    assert [case["trace"]["outcome"] for case in (dry, low, limited)] == [
        "completed",
        "lost_contact",
        "completed",
    ]
    assert 0 < low["trace"]["time"][-1] < config["duration_s"]
    assert limited["trace"]["time"][-1] == config["duration_s"]
    assert np.max(np.abs(limited["trace"]["slip"])) < np.max(
        np.abs(low["trace"]["slip"])
    )

    fixture = json.loads(
        (ROOT / "research/experiments/duk-v3/synthetic.json").read_text()
    )
    base = fixture["v2_balance"]
    study = fixture["balance_experiment"]
    model = WheeledPendulum(
        **{field.name: base[field.name] for field in fields(WheeledPendulum)}
    )
    a, b = model.linearization()
    q, r = bryson_weights(base["state_scales"], base["torque_scale_N_m"])
    period = base["sample_period_s"]
    first = discrete_lqr(a, b, q, r, period)
    alternate = discrete_lqr(a[1:, 1:], b[1:], q[1:, 1:], r, period)
    alternative_inputs = {
        "controller": "balance",
        "feedback": "gated",
        "delay_samples": 1,
        "noise_seed": 7,
        "pitch_noise": 0.0008,
        "friction": 0.3,
        "traction_budget": 0.25,
    }
    other = api(dashboard, "balance", alternative_inputs)
    assert other == api(dashboard, "balance", alternative_inputs)
    for run, design, anchor in ((low, first, 0.0), (other, alternate, None)):
        setup = run["configuration"]
        direct = simulate_balance(
            model,
            design.gain,
            math.radians(study["initial_lean_deg"]),
            period=period,
            duration=study["duration_s"],
            friction=0.3,
            motor_bound=base["total_torque_limit_N_m"],
            slip_scale=study["slip_scale_m_s"],
            fall_angle=math.radians(study["fall_angle_deg"]),
            feedback=setup["feedback"],
            traction_budget=setup["traction_budget"],
            anchor=anchor,
            delay_samples=setup["delay_samples"],
            noise_seed=setup["noise_seed"],
            accel_noise=study["acceleration_noise_m_s2"],
            pitch_noise=setup["pitch_noise"],
            slip_threshold=study["slip_threshold_m_s"],
        )
        np.testing.assert_allclose(run["linear_model"]["gain"], design.gain[0])
        assert run["trace"]["outcome"] == direct.outcome
        assert run["trace"]["limit_reason"] == list(direct.limit_reason)
        for field in (
            "time",
            "state",
            "requested",
            "limited",
            "applied",
            "slip",
            "normal",
        ):
            np.testing.assert_allclose(run["trace"][field], getattr(direct, field))


@pytest.mark.parametrize(
    ("path", "payload"),
    [
        ("balance", {"controller": "mpc"}),
        ("balance", {"feedback": "unsupported"}),
        ("balance", {"delay_samples": 1.5}),
        ("balance", {"friction": -1}),
        ("kinematics", {"left_deg": 45, "right_deg": 95}),
    ],
)
def test_dashboard_rejects_incompatible_inputs(dashboard, path, payload):
    with pytest.raises(HTTPError) as failure:
        api(dashboard, path, payload)
    assert failure.value.code == 400
    with failure.value as response:
        assert json.load(response)["error"]
