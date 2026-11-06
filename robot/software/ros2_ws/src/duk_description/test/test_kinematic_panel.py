"""Exercise real Qt widget signals with a headless canvas, not physical hardware."""

import math

import pytest


def test_panel_widgets_drive_the_same_bench_and_pause(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    import rclpy
    from duk_description.kinematic_node import KinematicNode
    from duk_description.kinematic_panel import run_panel
    from python_qt_binding import QtCore, QtWidgets

    rclpy.init(args=[])
    node = KinematicNode()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    errors = []
    observed = {}

    def guarded(action):
        def callback():
            try:
                action()
            except Exception as exc:
                errors.append(exc)
                app.quit()

        return callback

    def panel():
        return next(
            w
            for w in app.topLevelWidgets()
            if w.windowTitle().startswith("DUK — Bàn thử")
        )

    def manual():
        view = panel()
        view.angles[0].setValue(82)
        assert node.bench.samples[0].pose.linkage_angle_rad == pytest.approx(
            math.radians(82)
        )
        assert (
            node.bench.samples[1].pose.linkage_angle_rad
            != node.bench.samples[0].pose.linkage_angle_rad
        )
        view.play.click()
        assert node.bench.playing
        QtCore.QTimer.singleShot(800, guarded(pause))

    def pause():
        view = panel()
        assert node.bench.samples[0].pose.linkage_angle_rad > math.radians(82)
        view.play.click()
        observed["frozen"] = tuple(s.pose.o for s in node.bench.samples)
        assert not node.bench.playing
        view.speed.setValue(2)
        assert node.bench.speed == 2
        QtCore.QTimer.singleShot(400, guarded(finish))

    def finish():
        assert tuple(s.pose.o for s in node.bench.samples) == observed["frozen"]
        view = panel()
        assert len(view.history) > 1
        assert view.table.item(9, 1).text()
        assert len(view.lines[0].get_xdata()) > 1
        view.close()

    def timeout():
        raise TimeoutError("Panel smoke test stalled")

    QtCore.QTimer.singleShot(700, guarded(manual))
    watchdog = QtCore.QTimer()
    watchdog.setSingleShot(True)
    watchdog.timeout.connect(guarded(timeout))
    watchdog.start(15000)
    try:
        run_panel(node)
        if errors:
            raise errors[0]
        assert "frozen" in observed
    finally:
        watchdog.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
