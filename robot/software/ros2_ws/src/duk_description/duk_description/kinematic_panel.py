"""Qt/Matplotlib inspection panel; ROS and the numerical bench own the state."""

import math
import time
from collections import deque

import rclpy
from rclpy.parameter import Parameter


def run_panel(node):
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
    from matplotlib.figure import Figure
    from python_qt_binding import QtCore, QtWidgets

    class Panel(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("DUK — Bàn thử động học hai chân")
            self.resize(850, 1000)
            self.setStyleSheet("""
                QWidget {font-family: 'DejaVu Sans'; font-size: 12px; color: #172a3a; background: #f5f7fa;}
                QGroupBox {font-weight: bold; border: 1px solid #d8dfe8; border-radius: 6px; margin-top: 10px; padding: 10px;}
                QGroupBox::title {subcontrol-origin: margin; left: 10px;}
                QPushButton {background: #e5eaf1; border: 0; padding: 9px 14px; border-radius: 5px;}
                QPushButton#play {background: #e77a20; color: white; font-weight: bold;}
                QDoubleSpinBox {background: white; padding: 4px;}
                QTableWidget {background: white; gridline-color: #e3e8ef;}
            """)
            layout = QtWidgets.QVBoxLayout(self)
            title = QtWidgets.QLabel("DUK  /  ĐỘNG HỌC HAI CHÂN")
            title.setStyleSheet("font-size: 19px; font-weight: bold; padding: 4px;")
            layout.addWidget(title)
            layout.addWidget(
                QtWidgets.QLabel(
                    "Hai bánh tiếp xúc sàn · Thân đổi cao độ / nghiêng theo hình học"
                )
            )
            controls = QtWidgets.QGroupBox("Tư thế và chuyển động thử")
            grid = QtWidgets.QGridLayout(controls)
            self.sliders, self.angles = [], []
            for row, label in enumerate(("Chân trái qL", "Chân phải qR")):
                grid.addWidget(QtWidgets.QLabel(label), row, 0)
                slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
                slider.setRange(
                    math.ceil(math.degrees(node.bench.bounds[0]) * 100),
                    math.floor(math.degrees(node.bench.bounds[1]) * 100),
                )
                spin = QtWidgets.QDoubleSpinBox()
                spin.setRange(slider.minimum() / 100, slider.maximum() / 100)
                spin.setDecimals(2)
                spin.setSuffix(" °")
                slider.valueChanged.connect(
                    lambda value, side=row: self.manual(side, value / 100)
                )
                spin.valueChanged.connect(
                    lambda value, side=row: self.manual(side, value)
                )
                grid.addWidget(slider, row, 1, 1, 3)
                grid.addWidget(spin, row, 4)
                self.sliders.append(slider)
                self.angles.append(spin)
            self.play = QtWidgets.QPushButton("Chạy tự động")
            self.play.setObjectName("play")
            self.play.clicked.connect(
                lambda: self.apply({"playing": not node.bench.playing})
            )
            reset = QtWidgets.QPushButton("Về 75°")
            reset.clicked.connect(self.reset)
            self.speed = QtWidgets.QDoubleSpinBox()
            self.speed.setRange(0.25, 2)
            self.speed.setSingleStep(0.25)
            self.speed.setValue(1)
            self.speed.setSuffix(" ×")
            self.speed.valueChanged.connect(lambda value: self.apply({"speed": value}))
            grid.addWidget(self.play, 2, 0, 1, 2)
            grid.addWidget(reset, 2, 2)
            grid.addWidget(QtWidgets.QLabel("Tốc độ profile"), 2, 3)
            grid.addWidget(self.speed, 2, 4)
            lower, upper = map(
                math.degrees, node.geometry.geometric_linkage_angle_limits_rad()
            )
            grid.addWidget(
                QtWidgets.QLabel(
                    f"Biên hình học: {lower:.2f}–{upper:.2f}° · Dải thử chừa 5° mỗi đầu; không phải giới hạn motor"
                ),
                3,
                0,
                1,
                5,
            )
            layout.addWidget(controls)
            self.status = QtWidgets.QLabel()
            self.status.setWordWrap(True)
            layout.addWidget(self.status)
            self.body_status = QtWidgets.QLabel()
            layout.addWidget(self.body_status)
            labels = [
                "q (°)",
                "dq/dt (rad/s)",
                "d²q/dt² (rad/s²)",
                "Tâm bánh X (mm)",
                "Tâm bánh Z world (mm)",
                "Vx gắn thân (mm/s)",
                "Vz gắn thân (mm/s)",
                "Ax gắn thân (mm/s²)",
                "Az gắn thân (mm/s²)",
                "Sai số khép kín (mm)",
                "Sai số chiều dài BCO (mm)",
                "Sai số 2×diện tích BCO (mm²)",
                "Còn tới biên hình học (°)",
                "|sin góc BC/CD| (0 = kỳ dị)",
                "Khe hở bánh / sàn (mm)",
            ]
            self.table = QtWidgets.QTableWidget(len(labels), 3)
            self.table.setHorizontalHeaderLabels(
                ["Đại lượng tính từ mô hình", "Trái", "Phải"]
            )
            self.table.verticalHeader().hide()
            self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
            self.table.horizontalHeader().setSectionResizeMode(
                0, QtWidgets.QHeaderView.Stretch
            )
            self.table.horizontalHeader().setSectionResizeMode(
                1, QtWidgets.QHeaderView.ResizeToContents
            )
            self.table.horizontalHeader().setSectionResizeMode(
                2, QtWidgets.QHeaderView.ResizeToContents
            )
            for row, label in enumerate(labels):
                self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(label))
                for column in (1, 2):
                    self.table.setItem(row, column, QtWidgets.QTableWidgetItem("0"))
                self.table.setRowHeight(row, 22)
            self.table.setMinimumHeight(360)
            layout.addWidget(self.table)
            self.figure = Figure(figsize=(8, 3.5), dpi=90, layout="constrained")
            self.figure.suptitle("Tọa độ và đạo hàm trong hệ gắn thân", fontsize=9)
            self.axes = self.figure.subplots(3, 1, sharex=True)
            self.lines = []
            for axis, label in zip(
                self.axes,
                (
                    "Vị trí\n(mm)",
                    "Vận tốc\n(mm/s)",
                    "Gia tốc\n(mm/s²)",
                ),
            ):
                axis.set_ylabel(label, fontsize=8)
                axis.grid(alpha=0.2)
                axis.tick_params(labelsize=7)
                for side, color in (("Trái", "#df791e"), ("Phải", "#008eaa")):
                    for component, style in (("X", "--"), ("Z", "-")):
                        self.lines.append(
                            axis.plot(
                                [],
                                [],
                                style,
                                color=color,
                                linewidth=1.2,
                                label=f"{side} {component}",
                            )[0]
                        )
            self.axes[0].legend(loc="upper left", ncol=4, fontsize=7)
            self.axes[-1].set_xlabel("Thời gian xem (s)", fontsize=8)
            self.canvas = FigureCanvasQTAgg(self.figure)
            layout.addWidget(self.canvas, 1)
            layout.addWidget(
                QtWidgets.QLabel(
                    "Đồ thị: hệ gắn thân. Bảng: vị trí world và khe hở sàn. Chưa mô phỏng lực hay lăn không trượt."
                )
            )
            self.history = deque(maxlen=200)
            self.pose_revision = node.bench.pose_revision
            self.last_plot = 0.0
            self.last_error = ""
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.refresh)
            self.timer.start(50)

        def apply(self, values):
            result = node.set_parameters_atomically(
                [Parameter(name, value=value) for name, value in values.items()]
            )
            self.last_error = "" if result.successful else result.reason
            return result.successful

        def manual(self, side, value):
            name = "left_q_deg" if side == 0 else "right_q_deg"
            if self.apply({name: float(value), "playing": False}):
                # A manual pose jump is not a time trajectory; do not join it on a plot.
                self.history.clear()

        def reset(self):
            if self.apply({"left_q_deg": 75.0, "right_q_deg": 75.0, "playing": False}):
                self.history.clear()

        def refresh(self):
            if not rclpy.ok():
                self.close()
                return
            rclpy.spin_once(node, timeout_sec=0)
            if self.pose_revision != node.bench.pose_revision:
                self.history.clear()
                self.pose_revision = node.bench.pose_revision
            samples = node.bench.samples
            ground = node.ground
            self.body_status.setText(
                f"Thân cao {ground.body_height_mm:.2f} mm · Nghiêng ngang {math.degrees(ground.roll_rad):+.2f}° · Sàn z=0"
            )
            for side, sample in enumerate(samples):
                side_name = "left" if side == 0 else "right"
                world_o = ground.leg_point(sample.pose, side_name, "O")
                angle = math.degrees(sample.pose.linkage_angle_rad)
                for widget, value in (
                    (self.sliders[side], round(angle * 100)),
                    (self.angles[side], angle),
                ):
                    widget.blockSignals(True)
                    widget.setValue(value)
                    widget.blockSignals(False)
                values = [
                    angle,
                    sample.q_dot_rad_s,
                    sample.q_ddot_rad_s2,
                    sample.pose.o[0],
                    world_o[2],
                    *sample.wheel_velocity_mm_s,
                    *sample.wheel_acceleration_mm_s2,
                    sample.closure_error_mm,
                    sample.carrier_error_mm,
                    sample.carrier_area_error_mm2,
                    math.degrees(sample.boundary_margin_rad),
                    sample.loop_sine,
                    ground.ground_gap_mm(sample.pose, side_name),
                ]
                for row, value in enumerate(values):
                    self.table.item(row, side + 1).setText(
                        f"{value:.2e}" if row in (9, 10, 11) else f"{value:.4f}"
                    )
            self.play.setText("Tạm dừng" if node.bench.playing else "Chạy tự động")
            self.speed.blockSignals(True)
            self.speed.setValue(node.bench.speed)
            self.speed.blockSignals(False)
            near = [
                name
                for name, sample in zip(("trái", "phải"), samples)
                if sample.boundary_margin_rad < 2 * node.bench.reserve_rad
            ]
            self.status.setText(
                self.last_error
                or (
                    "Chú ý: chân " + ", ".join(near) + " gần biên hình học."
                    if near
                    else "Hình học hợp lệ · BCO giữ nguyên kích thước · Hai bánh chạm sàn"
                )
            )
            self.status.setStyleSheet(
                "color: #a64d10;" if self.last_error or near else "color: #167047;"
            )
            row = [node.bench.wall_time_s]
            for sample in samples:
                row.extend(
                    [
                        sample.pose.o[0],
                        sample.pose.o[1],
                        *sample.wheel_velocity_mm_s,
                        *sample.wheel_acceleration_mm_s2,
                    ]
                )
            self.history.append(row)
            if time.monotonic() - self.last_plot > 0.4 and len(self.history) > 1:
                times = [row[0] for row in self.history]
                for axis_index, axis in enumerate(self.axes):
                    for side in range(2):
                        for component in range(2):
                            index = 1 + side * 6 + axis_index * 2 + component
                            self.lines[axis_index * 4 + side * 2 + component].set_data(
                                times, [r[index] for r in self.history]
                            )
                    axis.relim()
                    axis.autoscale_view()
                    axis.set_xlim(max(0, times[-1] - 10), max(10, times[-1]))
                self.canvas.draw_idle()
                self.last_plot = time.monotonic()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = Panel()
    panel.show()
    app.exec_()
