"""Acceptance from scenarios through core studies, saved runs and ROS playback."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time
from dataclasses import asdict, replace

import numpy as np
import pytest
from workbench.inputs import LEFT_GRIP, PROFILE, ROOT, Inputs, read_object
from workbench.records import RunRecord, Signal, decimal, event
from workbench.session import Workbench, check_replay
from workbench.spatial import spatial_model

from duk_core.simulation.whole_body import run_whole_body


@pytest.fixture(scope="module")
def session():
    return Workbench(Inputs())


@pytest.fixture(scope="module")
def left_grip(session):
    return session.run(read_object(LEFT_GRIP))


@pytest.fixture(scope="module")
def short_nb6_record(session):
    return session.run({"study": "NB6", "duration_s": 0.015})


@pytest.fixture
def small_record():
    return RunRecord(
        "NB3",
        {"study": "NB3"},
        {},
        [0.0, 1.0],
        {"force": Signal("N", "left.wheel", [0, 1])},
        "completed",
        [event("completed", None, 1.0)],
        {},
    )


def test_replay_validates_signal_metadata_and_uses_tolerance_for_event_time(
    small_record,
):
    end = 1.0 + 1e-9
    check_replay(
        small_record,
        replace(
            small_record, time_s=[0.0, end], events=[event("completed", None, end)]
        ),
    )
    force = small_record.signals["force"]
    with pytest.raises(ValueError, match="diagnostics"):
        check_replay(small_record, replace(small_record, metadata={"gain": [1.0]}))
    for change, message in (
        ({"unit": "N·m"}, "signal metadata"),
        ({"frame": "body"}, "signal metadata"),
        ({"values": [False, True]}, "beyond tolerance"),
    ):
        with pytest.raises(ValueError, match=message):
            check_replay(
                small_record,
                replace(small_record, signals={"force": replace(force, **change)}),
            )


def test_run_file_rejects_invalid_signal_and_event_metadata(small_record, tmp_path):
    path = tmp_path / "bad-run.json"
    payload = {"schema_version": 1, **asdict(small_record)}
    payload["signals"]["force"]["scope"] = 4
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="signal metadata"):
        RunRecord.load(path)
    payload["signals"]["force"]["scope"] = "left.wheel"
    payload["events"][0].pop("requested_s")
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="Invalid run event"):
        RunRecord.load(path)


def test_signal_plot_keeps_missing_components_and_rejects_incompatible_shapes():
    signal = Signal("N", "left.wheel", [[1.0, None], None, [2.0, 3.0]])
    np.testing.assert_equal(
        signal.plot_components(), [[1.0, np.nan, 2.0], [np.nan, np.nan, 3.0]]
    )
    assert replace(signal, values=[[1.0], [2.0, 3.0]]).plot_components() is None
    assert replace(signal, values=[[["a"]], [[1.0]]]).plot_components() is None


def test_profile_integer_literals_do_not_bypass_nb6_number_validation(tmp_path):
    profile = read_object(PROFILE)
    profile["whole_body"]["scenario"]["friction_change_s"] = 5
    alternate = tmp_path / "synthetic.json"
    alternate.write_text(json.dumps(profile))
    inputs = Inputs(alternate)
    with pytest.raises(ValueError, match="friction_change_s"):
        inputs.whole_body_configuration({"friction_change_s": -1})
    assert (
        inputs.whole_body_configuration({"friction_change_s": 0.25})[
            "friction_change_s"
        ]
        == 0.25
    )


def test_left_grip_parity_and_comparison(session, left_grip, tmp_path):
    config = left_grip.configuration
    inputs = session.inputs
    model = spatial_model(inputs, config)
    trace = run_whole_body(
        model,
        inputs.motor,
        inputs.fixture["v2_balance"],
        inputs.fixture["whole_body"],
        config,
        method="weighted",
        priority="balance_first",
    )
    assert left_grip.time_s[:-1] == trace.time.tolist()
    assert left_grip.time_s[-1] == trace.terminal_time == config["duration_s"]
    np.testing.assert_allclose(
        left_grip.signals["speed_estimation_error"].values[:-1],
        trace.true_and_estimated_speed[:, 1] - trace.true_and_estimated_speed[:, 0],
    )
    np.testing.assert_allclose(
        left_grip.signals["yaw_rate_tracking_error"].values[:-1],
        trace.effective_reference[:, 1] - trace.pose_rate[:, 3],
    )
    np.testing.assert_allclose(
        left_grip.signals["position"].values[-1], trace.terminal_state.position
    )
    at_change = next(
        i for i, t in enumerate(left_grip.time_s) if t >= config["friction_change_s"]
    )
    np.testing.assert_allclose(trace.actual_friction[at_change - 1], [0.65, 0.65])
    np.testing.assert_allclose(trace.actual_friction[at_change], [0.13, 0.65])
    for index, side in enumerate(("left", "right")):
        for key, value in (
            ("wheel_center", trace.wheel_centers[:, index]),
            ("normal_force", trace.force[:, index, 2]),
            ("tangential_force", trace.force[:, index, :2]),
            ("hip_applied", trace.applied_effort[:, index]),
        ):
            np.testing.assert_allclose(
                left_grip.signals[f"{side}_{key}"].values[:-1], value
            )
        assert left_grip.signals[side + "_contact_mode"].values[:-1] == [
            m[index] for m in trace.contact_modes
        ]
        assert (
            left_grip.signals[side + "_contact_mode"].values[-1]
            == trace.terminal_contact_modes[index]
        )
        assert (
            left_grip.signals[side + "_hip_preceding_effort"].values[-1]
            == trace.terminal_integrated_effort[index]
        )
        np.testing.assert_allclose(
            np.array(left_grip.signals[side + "_O_world"].values) / 1000,
            left_grip.signals[side + "_wheel_center"].values,
            atol=1e-12,
        )
    assert max(left_grip.signals["left_slip_speed"].values) > max(
        left_grip.signals["right_slip_speed"].values
    )
    advanced = session.run(config | {"controller": "hierarchical"})
    assert advanced.time_s == left_grip.time_s
    assert advanced.stop_reason == "completed"
    assert (
        advanced.signals["left_friction"].values
        == left_grip.signals["left_friction"].values
    )
    assert not np.allclose(
        advanced.signals["left_hip_applied"].values[:-1],
        left_grip.signals["left_hip_applied"].values[:-1],
    )
    assert session.equations(left_grip, at_change, "body")["M"][0][0] > 0
    assert session.equations(left_grip, at_change, "left.linkage.O")["O"][
        "dp/dq [mm/rad]"
    ]
    wheel = session.equations(left_grip, at_change, "left.wheel")
    np.testing.assert_allclose(wheel["J v [m/s]"], trace.contact_speed[at_change, 0])
    record = tmp_path / "run.json"
    left_grip.save(record)
    loaded = RunRecord.load(record)
    csv_path = tmp_path / "signals.csv"
    loaded.export_csv(csv_path)
    assert "left_slip_speed [m/s]" in csv_path.read_text().splitlines()[0]
    assert loaded == left_grip


@pytest.mark.parametrize(
    ("case", "source", "outcome"),
    [
        ("sensor_lost", "stop", "sensor"),
        ("late", "balance_fallback", "no_valid_command"),
        ("infeasible", "balance_fallback", "no_valid_command"),
    ],
)
def test_supervisor_evidence(session, case, source, outcome):
    run = session.run({"study": "NB6", "case": case, "duration_s": 0.42})
    assert run.stop_reason == outcome
    fault = next(item for item in run.events if item["name"].startswith("Lỗi"))
    index = run.time_s.index(fault["actual_s"])
    assert run.signals["command_source"].values[index] == source
    if outcome == "sensor":
        assert run.time_s[-1] == fault["actual_s"]
        assert run.signals["odometry_speed"].values[-1] is None
        assert run.signals["sensor_valid"].values[-1] is False
    else:
        assert run.signals["command_source"].values[-1] == "stop"
        assert run.time_s[-1] > fault["actual_s"]


def test_linkage_asymmetry_inverse_and_static_load(session):
    base = {"study": "NB1–2", "left_q_deg": 95.0, "right_q_deg": 95.0}
    pose = session.run(base)
    adjusted = session.run(
        base
        | {
            "left_q_deg": 105.0,
            "joint_rate_rad_s": [4.0, 0.0],
            "force_left_N": [12.0, 40.0],
            "support_levels_m": [0.006, -0.006],
        }
    )
    for key in ("right_O", "right_hip_static_effort", "right_hip_limit"):
        assert pose.signals[key].values == adjusted.signals[key].values
    assert pose.signals["left_O"].values != adjusted.signals["left_O"].values
    assert (
        pose.signals["left_hip_static_effort"].values
        != adjusted.signals["left_hip_static_effort"].values
    )
    assert (
        adjusted.signals["left_hip_limit"].values[0]
        < pose.signals["left_hip_limit"].values[0]
    )
    for side in ("left", "right"):
        assert abs(adjusted.signals[side + "_gap"].values[0]) < 1e-10
        assert adjusted.signals[side + "_carrier_error"].values[0] < 1e-9
    inverse = session.run(
        base
        | {"mode": "inverse", "inverse_target_mm": adjusted.signals["left_O"].values[0]}
    )
    assert 105.0 == pytest.approx(inverse.metadata["inverse_solutions_deg"][0])
    assert decimal(1e-12) == "< 0,000001"


def test_prescribed_world_wheel_derivatives(session):
    run = session.run(
        {
            "study": "NB1–2",
            "mode": "trajectory",
            "left_final_deg": 108.0,
            "right_final_deg": 100.0,
            "support_levels_m": [0.006, -0.006],
            "period_s": 0.001,
        }
    )
    for side in ("left", "right"):
        index = len(run.time_s) // 2
        centers_mm = 1000 * np.asarray(run.signals[side + "_wheel_center"].values)
        span = centers_mm[index - 1 : index + 2]
        dt = run.time_s[index + 1] - run.time_s[index]
        velocity = (span[2] - span[0]) / (2 * dt)
        accel = (span[2] - 2 * span[1] + span[0]) / dt**2
        np.testing.assert_allclose(
            run.signals[side + "_wheel_velocity_world"].values[index],
            velocity,
            atol=0.001,
        )
        np.testing.assert_allclose(
            run.signals[side + "_wheel_accel_world"].values[index], accel, atol=0.001
        )


def test_static_load_sweep_preserves_speed_and_partial_sample_interval(session):
    run = session.run(
        {
            "study": "NB1–2",
            "mode": "load_sweep",
            "left_q_deg": 95,
            "left_final_deg": 105,
            "joint_rate_rad_s": [2, -3],
            "duration_s": 0.14,
            "period_s": 0.05,
        }
    )
    np.testing.assert_allclose(run.time_s, [0, 0.05, 0.1, 0.14], atol=1e-15)
    assert run.signals["left_hip_rate"].values[0] == 0
    np.testing.assert_allclose(
        run.signals["left_hip_limit"].values, session.inputs.motor.joint_limit(2)
    )
    np.testing.assert_allclose(
        run.signals["right_hip_limit"].values, session.inputs.motor.joint_limit(-3)
    )
    assert (
        run.signals["left_hip_static_effort"].values[0]
        != run.signals["left_hip_static_effort"].values[-1]
    )
    inspected = session.equations(run, 2, "left.linkage.BC")
    assert "A" not in inspected
    assert inspected["BC"]["Chiều dài [mm]"] == pytest.approx(
        session.inputs.geometry.bc_mm
    )
    np.testing.assert_allclose(
        inspected["BC"]["Vectơ trong hệ chân [mm]"],
        np.subtract(run.signals["left_C"].values[2], run.signals["left_B"].values[2]),
    )


def test_nb3_off_grid_grip_event_and_signed_nb6_inputs(session):
    switch = 0.0123
    run = session.run(
        {
            "study": "NB3",
            "case": "rolling",
            "friction_change_s": switch,
            "friction_after": [0, 0.65],
        }
    )
    assert run.stop_reason == "rolling_limit" and run.time_s[-1] == switch
    assert event("Đổi độ bám", switch, switch) in run.events
    unreached = session.run(
        {"study": "NB3", "case": "flight", "friction_change_s": 1.0}
    )
    assert event("Đổi độ bám", 1, None) in unreached.events
    run = session.run(
        {
            "study": "NB6",
            "case": "drive_release",
            "drive_speed_m_s": -0.03,
            "turn_rate_rad_s": -0.1,
            "duration_s": 0.02,
        }
    )
    assert run.signals["requested_speed"].values[0] == -0.03
    assert run.signals["requested_yaw_rate"].values[0] == -0.1


def test_nb4_reproduction_uses_loaded_profile_bytes(tmp_path):
    path = tmp_path / "alternate.json"
    profile = read_object(PROFILE)
    path.write_text(json.dumps(profile))
    session = Workbench(Inputs(path))
    profile["v2_balance"]["radius_m"] *= 2
    path.write_text(json.dumps(profile))
    run = session.run({"study": "NB4", "feedback": "truth"})
    assert (
        run.metadata["parameters"]["plant"]["radius_m"]
        == session.inputs.fixture["v2_balance"]["radius_m"]
    )
    assert run.signals["speed_estimation_error"].values[:-1] == [0.0] * (
        len(run.time_s) - 1
    )
    assert run.signals["speed_estimation_error"].values[-1] is None
    saved = tmp_path / "run.json"
    run.save(saved)
    session.verify_replay(RunRecord.load(saved))


def test_sagittal_contact_termination_and_height_policies(session):
    roll, slide, landing, limit = [
        session.run({"study": "NB3", "case": case})
        for case in ("rolling", "slip", "flight", "rolling_limit")
    ]
    assert max(abs(s) for s in roll.signals["left_slip_speed"].values) < 1e-7
    assert max(abs(s) for s in slide.signals["left_slip_speed"].values) > 0.05
    assert landing.stop_reason == "touchdown"
    assert landing.time_s[-1] == pytest.approx(
        math.sqrt(0.2 / session.inputs.planar.parameters["g_m_s2"]), abs=1e-7
    )
    assert limit.stop_reason == "rolling_limit" and 0 < limit.time_s[-1] < 0.2
    touching = session.run({"study": "NB3", "case": "flight", "drop_height_m": 0})
    assert touching.stop_reason == "touchdown"
    assert touching.time_s == [0.0]
    options = {"study": "NB5", "case": "slow", "duration_s": 0.2}
    mpc = session.run(options | {"controller": "mpc"})
    fixed = session.run(options | {"controller": "fixed"})
    assert mpc.time_s == fixed.time_s
    assert mpc.signals["linear_a"].values[0]
    assert mpc.signals["applied"].values != fixed.signals["applied"].values
    for i, effort in enumerate(mpc.signals["applied"].values[:-1]):
        low, high = mpc.signals["torque_interval"].values[i]
        assert low <= effort <= high
        assert max(mpc.signals["force_constraints"].values[i]) <= 1e-8


@pytest.mark.parametrize(
    "options",
    [
        {"study": "NB6", "controller": "position"},
        {"study": "NB6", "friction_after": [0.5, -0.1]},
        {"study": "NB6", "support_levels_m": [0.5]},
        {"study": "NB3", "controller": "weighted"},
        {"study": "NB4", "controller": "mpc"},
    ],
)
def test_rejects_incompatible_studies_and_commands(session, options):
    with pytest.raises(ValueError):
        session.run(options)


def test_nb6_multiple_changes_resolve_on_command_ticks(session):
    changes = [
        {"time_s": 0, "field": "drive_speed_m_s", "value": 0.08},
        {"time_s": 0.010, "field": "friction_left", "value": 0.3},
        {"time_s": 0.012, "field": "friction_right", "value": 0.5},
        {"time_s": 0.018, "field": "turn_rate_rad_s", "value": -0.14},
        {"time_s": 0.020, "field": "drive_speed_m_s", "value": 0},
        {"time_s": 0.031, "field": "friction_left", "value": 0.55},
        {"time_s": 0.041, "field": "level_hold", "value": True},
        {"time_s": 0.045, "field": "fault", "value": "deadline"},
        {"time_s": 0.055, "field": "fault", "value": "none"},
        {"time_s": 0.073, "field": "friction_left", "value": 0.2},
    ]
    options = read_object(LEFT_GRIP) | {"duration_s": 0.073, "schedule": changes}
    record = session.run(options)
    assert record.stop_reason == "completed"
    assert record.time_s == [0.0, 0.015, 0.030, 0.045, 0.060, 0.073]
    scheduled = [item for item in record.events if item["name"].startswith("schedule.")]
    assert len(scheduled) == len(changes)
    assert [item["requested_s"] for item in scheduled] == [
        change["time_s"] for change in changes
    ]
    assert [item["actual_s"] for item in scheduled] == [
        0,
        0.015,
        0.015,
        0.030,
        0.030,
        0.045,
        0.045,
        0.045,
        0.060,
        None,
    ]
    assert record.signals["left_friction"].values == [0.65, 0.3, 0.3, 0.55, 0.55, None]
    assert record.signals["right_friction"].values == [0.65, 0.5, 0.5, 0.5, 0.5, None]
    assert record.signals["requested_speed"].values == [0.08, 0.08, 0, 0, 0, None]
    assert record.signals["requested_yaw_rate"].values == [
        0,
        0,
        -0.14,
        -0.14,
        -0.14,
        None,
    ]
    assert not record.signals["feature_intent"].values[2][1]
    assert record.signals["feature_intent"].values[3][1]
    assert record.signals["solver_status"].values[3] == "deadline"
    assert record.signals["solver_status"].values[4] != "deadline"
    assert not any(
        item["name"] == "Đổi độ bám" or item["name"] == "Nhiễu động vận tốc"
        for item in record.events
    )


def test_nb6_schedule_rejects_conflicting_ticks_and_negative_grip(session):
    options = {"study": "NB6", "duration_s": 0.073}
    first = {"time_s": 0.016, "field": "friction_left", "value": 0.25}
    conflict = first | {"time_s": 0.02, "value": 0.3}
    with pytest.raises(ValueError, match="dòng 2.*cùng mẫu lệnh"):
        session.run(options | {"schedule": [first, conflict]})
    with pytest.raises(ValueError, match="độ bám phải không âm"):
        session.run(options | {"schedule": [first | {"value": -0.1}]})


def test_nb6_step_preserves_full_horizon_and_event_phases(session):
    options = {
        "study": "NB6",
        "duration_s": 0.049,
        "schedule": [
            {"time_s": 0.004, "field": "friction_left", "value": 0.3},
            {"time_s": 0.019, "field": "drive_speed_m_s", "value": 0.08},
        ],
    }
    experiment = session.begin_nb6(options)
    runner = experiment.runner
    assert runner.time_s == 0.0
    assert not runner.has_samples
    with pytest.raises(ValueError, match="before reading"):
        experiment.record()

    assert runner.advance() == 0.015
    first = experiment.record()
    assert first.stop_reason == "paused"
    assert first.time_s == [0.0, 0.015]
    pre_event_velocity = first.signals["state_velocity"].values[-1][0]
    assert first.events[0]["actual_s"] is None
    assert first.signals["left_hip_applied"].values[-1] is None
    assert first.signals["left_hip_preceding_effort"].values[-1] is not None

    assert runner.advance() == 0.03
    partial = experiment.record(cancelled=True)
    assert partial.stop_reason == "cancelled"
    assert [entry["actual_s"] for entry in partial.events[:2]] == [0.015, None]
    assert first.time_s == [0.0, 0.015]
    with pytest.raises(ValueError, match="chỉ dùng để phát lại"):
        session.verify_replay(partial)

    runner.run_to_end()
    assert runner.finished
    final = experiment.record()
    assert final.time_s[-1] == 0.049
    assert [entry["actual_s"] for entry in final.events[:2]] == [0.015, 0.03]
    assert runner.advance() == 0.049
    check_replay(session.run(options), final)

    stopped = session.begin_nb6(
        options
        | {
            "schedule": [
                {"time_s": 0.01, "field": "fault", "value": "sensor"},
                {
                    "time_s": 0.004,
                    "field": "velocity_impulse_m_s",
                    "value": [0.02, 0, 0],
                },
                options["schedule"][1],
            ]
        }
    )
    assert stopped.runner.advance() == 0.015
    assert stopped.runner.advance() == 0.015
    assert stopped.runner.finished
    stopped_record = stopped.record()
    assert stopped_record.stop_reason == "sensor"
    assert stopped_record.signals["sensor_valid"].values == [True, False]
    assert [entry["actual_s"] for entry in stopped_record.events[:3]] == [
        0.015,
        0.015,
        None,
    ]
    assert (
        stopped_record.signals["state_velocity"].values[-1][0] - pre_event_velocity
    ) == pytest.approx(0.02)


@pytest.mark.ros
def test_qt_nb6_step_pause_resume_and_cancel(
    session, short_nb6_record, tmp_path, monkeypatch
):
    pytest.importorskip("python_qt_binding")
    rclpy = pytest.importorskip("rclpy")
    from std_msgs.msg import Float64
    from workbench.qt_panel import W, WorkbenchWindow
    from workbench.viewer import Viewer

    options = {
        "study": "NB6",
        "duration_s": 0.043,
        "schedule": [{"time_s": 0.012, "field": "friction_left", "value": 0.3}],
    }
    app = W.QApplication.instance() or W.QApplication([])
    window = WorkbenchWindow(
        session, short_nb6_record.configuration, short_nb6_record, rviz=False
    )
    observed_times = []
    node = None

    def until(condition):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            app.processEvents()
            rclpy.spin_once(node, timeout_sec=0.02)
            if condition():
                return
            time.sleep(0.01)
        pytest.fail("Timed out waiting for the interactive NB6 run")

    try:
        window.viewer = Viewer(session.inputs, headless=True)
        node = rclpy.create_node("test_nb6_interactive")
        node.create_subscription(
            Float64,
            "/duk/workbench/sample_time",
            lambda msg: observed_times.append(msg.data),
            10,
        )
        window.viewer_timer.start()
        window.show()
        window.scenario.set_configuration(options)
        window.prepare_button.click()
        until(lambda: window.interactive is not None and window.pending is None)
        assert window.interactive.runner.time_s == 0
        assert not window.scenario.isEnabled()
        assert window.run is short_nb6_record
        window.step_button.click()
        until(lambda: window.pending is None and window.run.stop_reason == "paused")
        assert window.time.text() == "0,015 s"
        assert window.run.events[0]["actual_s"] is None
        until(lambda: bool(observed_times) and observed_times[-1] == 0.015)

        window.continue_button.click()
        assert window.continue_button.text() == "Tạm dừng tính"
        window.continue_button.click()
        until(lambda: window.pending is None and window.time.text() == "0,03 s")
        assert window.run.events[0]["actual_s"] == 0.015
        assert window.run.stop_reason == "paused"
        window.seek(0.0)
        assert window.time.text() == "0 s"
        assert window.interactive.runner.time_s == 0.03

        window.continue_button.click()
        until(lambda: window.interactive is None and window.pending is None)
        assert window.time.text() == "0,043 s"
        assert window.run.stop_reason == "completed"
        until(lambda: observed_times[-1] == 0.043)

        window.prepare_button.click()
        until(lambda: window.interactive is not None and window.pending is None)
        window.step_button.click()
        window.cancel_button.click()
        until(lambda: window.interactive is None and window.pending is None)
        assert window.run.stop_reason == "cancelled"
        assert window.run.time_s[-1] == 0.015
        assert window.run.events[0]["actual_s"] is None
        path = tmp_path / "cancelled.json"
        monkeypatch.setattr(
            W.QFileDialog, "getSaveFileName", lambda *_: (str(path), "")
        )
        window.save_run()
        assert RunRecord.load(path).stop_reason == "cancelled"
        assert window.scenario.isEnabled()
    finally:
        if node is not None:
            node.destroy_node()
        window.close()


@pytest.mark.ros
def test_qt_creates_schedule_and_cli_replays_same_file(
    session, short_nb6_record, tmp_path, monkeypatch
):
    pytest.importorskip("python_qt_binding")
    from workbench.qt_panel import ROLE, QtCore, W, WorkbenchWindow
    from workbench.qt_schedule import ScheduleDialog

    app = W.QApplication.instance() or W.QApplication([])
    before = short_nb6_record
    window = WorkbenchWindow(session, before.configuration, before, rviz=False)
    scope = window.scenario
    errors = []

    def edit_rows():
        dialog = app.activeModalWidget()
        try:
            assert isinstance(dialog, ScheduleDialog)
            dialog.add_row()
            dialog.table.item(0, 0).setText("0,004")
            field = dialog.table.cellWidget(0, 1)
            field.setCurrentIndex(field.findData("friction_left"))
            dialog.table.cellWidget(0, 2).setText("0,25")
            dialog.add_row({"time_s": 0.019, "field": "drive_speed_m_s", "value": 0.12})
            dialog.accept()
            assert not dialog.error.text()
        except Exception as exc:
            errors.append(exc)
            if isinstance(dialog, ScheduleDialog):
                dialog.reject()

    try:
        window.show()
        window.new_button.click()
        assert scope.schedule_mode.isChecked()
        assert window.run is before
        assert "chưa chạy" in window.draft_notice.text()
        scope.findChild(W.QLineEdit, "duration_s").setText("0,043")
        QtCore.QTimer.singleShot(0, edit_rows)
        scope.schedule_button.click()
        assert not errors
        requested = scope.configuration()
        assert requested["schedule"][0]["value"] == 0.25
        assert scope.findChild(W.QLineEdit, "friction_change_s").isHidden()
        scenario_path = tmp_path / "scenario.json"
        monkeypatch.setattr(
            W.QFileDialog,
            "getSaveFileName",
            lambda *_args: (str(scenario_path), "JSON (*.json)"),
        )
        window.save_scenario()
        assert read_object(scenario_path) == requested

        window.run_button.click()
        deadline = time.monotonic() + 20
        while window.pending is not None and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.01)
        run = window.run
        assert run is not before
        assert run.configuration == requested
        grip = next(
            window.events.topLevelItem(i)
            for i in range(window.events.topLevelItemCount())
            if window.events.topLevelItem(i).text(0).startswith("Độ bám bánh trái")
        )
        assert grip.data(0, ROLE) == 0.015
        assert grip.text(1) == "0,004 / 0,015"
        window.seek_event(grip, 0)
        assert window.time.text() == "0,015 s"
        assert "Kịch bản của kết quả" in window.draft_notice.text()

        result_path = tmp_path / "cli.json"
        subprocess.run(
            [
                sys.executable,
                str(ROOT / "applications/operator-dashboard/qt_workbench.py"),
                "--scenario",
                str(scenario_path),
                "--output",
                str(result_path),
            ],
            cwd=ROOT,
            check=True,
            capture_output=True,
            timeout=60,
            env=os.environ
            | {
                "PYTHONPATH": os.pathsep.join(
                    filter(
                        None,
                        (
                            str(ROOT / "robot/software/core/src"),
                            os.environ.get("PYTHONPATH"),
                        ),
                    )
                )
            },
        )
        check_replay(run, RunRecord.load(result_path))

        window.set_run(before)
        window.set_comparison(run)
        window.seek(run.time_s[-1])
        assert window.time.text() == "0,043 s · Đối chiếu"
        row = next(
            i
            for i in range(window.signals.rowCount())
            if window.signals.item(i, 0).data(ROLE) == "height"
        )
        assert window.signals.item(row, 1).text() == "—"
        assert window.signals.item(row, 2).text() != "—"
    finally:
        window.close()


@pytest.mark.ros
def test_viewer_start_failure_cleans_up_ros_and_publisher(session, monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    from workbench.viewer import Viewer

    started = []
    original_start = Viewer.start

    def missing_rviz(viewer, package, *args):
        if package == "rviz2":
            raise LookupError("RViz package unavailable")
        original_start(viewer, package, *args)
        started.extend(viewer.processes)

    monkeypatch.setattr(Viewer, "start", missing_rviz)
    try:
        with pytest.raises(LookupError, match="RViz package unavailable"):
            Viewer(session.inputs)
        assert started and all(proc.poll() is not None for proc in started)
        assert not rclpy.ok()
    finally:
        for proc in started:
            if proc.poll() is None:
                proc.terminate()
                proc.wait(timeout=5)
        if rclpy.ok():
            rclpy.shutdown()


@pytest.mark.ros
def test_qt_run_button_and_final_ros_tf(session, monkeypatch, tmp_path):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("python_qt_binding")
    from rclpy.time import Time
    from std_msgs.msg import Float64
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import MarkerArray
    from workbench.qt_panel import ROLE, W, WorkbenchWindow, shown
    from workbench.ui_text import wheel_readout
    from workbench.viewer import Viewer, replay_frame

    app = W.QApplication.instance() or W.QApplication([])
    window = WorkbenchWindow(
        session, {"study": "NB6", "case": "nominal", "duration_s": 0.03}, rviz=False
    )
    viewer = Viewer(session.inputs, headless=True)
    window.viewer = viewer
    node = rclpy.create_node("test_workbench_replay")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    stamps, markers = [], []
    node.create_subscription(
        Float64, "/duk/workbench/sample_time", lambda msg: stamps.append(msg.data), 10
    )
    node.create_subscription(MarkerArray, "/duk/workbench/markers", markers.append, 10)

    def until(condition):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            app.processEvents()
            rclpy.spin_once(node, timeout_sec=0.02)
            if condition():
                return
        pytest.fail("Timed out waiting for Qt/ROS replay")

    try:
        window.show()
        until(lambda: window.run is not None)
        window.scenario.set_configuration({"study": "NB6"})
        assert window.scenario.configuration()["case"] == "nominal"
        window.scenario.set_configuration(
            {"study": "NB6", "case": "nominal", "duration_s": 1, "friction": 0.5}
        )
        window.scenario.findChild(W.QLineEdit, "duration_s").setText("0,5")
        edited = window.scenario.configuration()
        assert edited["duration_s"] == 0.5
        assert edited["friction"] == [0.5, 0.5]
        with pytest.raises(ValueError, match="Bộ điều khiển không hợp lệ"):
            window.scenario.set_configuration(
                {"study": "NB5", "controller": "weighted"}
            )
        assert window.scenario.configuration() == edited
        window.scenario.findChild(W.QLineEdit, "duration_s").setText("NaN")
        with pytest.raises(ValueError, match="Thời lượng"):
            window.scenario.configuration()
        window.scenario.set_configuration(
            session.defaults("NB6", "left_grip") | {"duration_s": 0.45}
        )
        window.run_button.click()
        until(lambda: window.run.configuration["case"] == "left_grip")
        run = window.run
        assert shown(["Active", "Paused"]) == "[ Đang hoạt động / Đang tạm dừng ]"
        assert shown("Bánh trái · sliding") == "Bánh trái · Trượt"
        event_titles = [
            window.events.topLevelItem(i).text(0)
            for i in range(window.events.topLevelItemCount())
        ]
        assert "Bánh trái · Trượt" in event_titles
        index = run.time_s.index(0.39)
        window.slider.setValue(index)
        assert window.time.text() == "0,39 s"
        left_readout = window.wheel_columns["left"].text()
        assert left_readout == "\n".join(wheel_readout(run, index, "left"))
        assert "Lực pháp tuyến" in left_readout
        assert "Mô-men hông" in left_readout
        left = window.root.child(1)
        for item, channel in (
            (left.child(1), "left_slip_speed"),
            (left.child(2), "left_hip_applied"),
            (window.root.child(0), "height"),
        ):
            window.objects.setCurrentItem(item)
            row = next(
                i
                for i in range(window.signals.rowCount())
                if window.signals.item(i, 0).data(ROLE) == channel
            )
            assert window.signals.item(row, 1).text() == shown(
                run.signals[channel].values[index]
            )
            assert window.time.text() == "0,39 s"
        partial = dict(run.signals)
        partial.pop("orientation_xyzw")
        assert replay_frame(replace(run, signals=partial), index) is None
        partial = dict(run.signals)
        partial.pop("left_tangential_force")
        assert (
            replay_frame(replace(run, signals=partial), index).wheels[0].force is None
        )
        window.viewer_timer.start()
        contact = run.signals["left_contact_point"].values[index]

        def matching_arrow():
            if not markers:
                return None
            return next(
                (
                    marker
                    for marker in markers[-1].markers
                    if marker.id == 7
                    and [marker.points[1].x, marker.points[1].y, marker.points[1].z]
                    == pytest.approx(contact)
                ),
                None,
            )

        until(
            lambda: (
                stamps
                and stamps[-1] == 0.39
                and markers
                and matching_arrow() is not None
            )
        )
        arrow = matching_arrow()
        assert (
            arrow.points[1].x,
            arrow.points[1].y,
            arrow.points[1].z,
        ) == pytest.approx(contact)
        assert arrow.points[1].z - arrow.points[0].z == pytest.approx(
            0.004 * run.signals["left_normal_force"].values[index]
        )
        assert not any(marker.text for marker in markers[-1].markers)
        window.slider.setValue(window.slider.maximum())
        assert window.time.text() == "0,45 s"
        assert "Mô-men bánh ở bước trước" in window.wheel_columns["left"].text()

        def matches_terminal_tf():
            for side in ("left", "right"):
                frame = f"duk_workbench/{side}_wheel_frame"
                if not buffer.can_transform("world", frame, Time()):
                    return False
                position = buffer.lookup_transform(
                    "world", frame, Time()
                ).transform.translation
                if not np.allclose(
                    [position.x, position.y, position.z],
                    run.signals[side + "_wheel_center"].values[-1],
                    atol=1e-8,
                    rtol=0,
                ):
                    return False
            return stamps[-1] == run.time_s[-1]

        until(matches_terminal_tf)
        monkeypatch.setattr(
            W.QInputDialog, "getItem", lambda *a, **kw: ("HQP theo thứ tự", True)
        )
        window.compare_button.click()
        until(lambda: window.comparison is not None)
        assert window.comparison.configuration == run.configuration | {
            "controller": "hierarchical"
        }
        assert len(window.axes.get_legend_handles_labels()[0]) == 2
        # Switch to an alternative profile from the same panel that ran the acceptance case.
        window.viewer_timer.stop()
        window.viewer = None
        path = tmp_path / "motor.json"
        profile = read_object(PROFILE)
        profile["hip_envelope"]["stall_torque_N_m"] *= 0.5
        path.write_text(json.dumps(profile))
        window.scenario.set_configuration({"study": "NB1–2", "left_q_deg": 100})
        window.run_with_profile(path)
        until(lambda: window.run.study == "NB1–2")
        assert window.run.configuration["left_q_deg"] == 100
        assert window.run.provenance["profile_name"] == "motor.json"
        assert window.run.signals["left_hip_limit"].values[0] == pytest.approx(
            session.inputs.motor.joint_limit(0) / 2
        )
    finally:
        listener.unregister()
        node.destroy_node()
        window.close()
        viewer.close()
