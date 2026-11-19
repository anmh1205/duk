"""Acceptance from scenarios through core studies, saved runs and ROS playback."""

from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, replace

import numpy as np
import pytest
from workbench.inputs import LEFT_GRIP, PROFILE, Inputs, read_object
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
    assert session.verify_replay(loaded).stop_reason == "completed"


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
        window.scenario.set_configuration(
            session.defaults("NB6", "left_grip") | {"duration_s": 0.45}
        )
        window.run_button.click()
        until(lambda: window.run.configuration["case"] == "left_grip")
        run = window.run
        index = run.time_s.index(0.39)
        window.slider.setValue(index)
        assert window.time.text() == "0,39 s"
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
        until(
            lambda: (
                stamps
                and stamps[-1] == 0.39
                and markers
                and any(m.text == "NB6 · 0,39 s" for m in markers[-1].markers)
            )
        )
        arrow = next(m for m in markers[-1].markers if m.id == 7)
        assert (
            arrow.points[0].x,
            arrow.points[0].y,
            arrow.points[0].z,
        ) == pytest.approx(run.signals["left_contact_point"].values[index])
        window.slider.setValue(window.slider.maximum())
        assert window.time.text() == "0,45 s"

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
