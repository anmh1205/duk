"""Qt widgets for scenario setup, synchronized replay and study inspection."""

from __future__ import annotations

import bisect
from concurrent.futures import Future, ThreadPoolExecutor
from functools import partial
from pathlib import Path
from time import monotonic

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.ticker import FuncFormatter
from python_qt_binding import QtCore, QtWidgets

from .inputs import INTEGER_SCENARIO_FIELDS, NB6_LEGACY_TIMED_FIELDS, Inputs, merged
from .kinematics import POINTS, SEGMENTS
from .qt_schedule import ScheduleDialog, editor_text
from .records import RunRecord, decimal, finite_number, save_json
from .session import CONTROLLER_LABELS, CONTROLLERS, STUDIES, Workbench
from .spatial import SpatialRun
from .ui_text import (
    OPTIONS,
    SCHEDULE_LABELS,
    case_label,
    configuration_value,
    field_label,
    frame_label,
    label,
    parameter_label,
    shown,
    unit_label,
    wheel_readout,
)

W = QtWidgets
ROLE = QtCore.Qt.UserRole
COLOR = ("#166c67", "#ac4e30")


def combo(options: dict[str, str]) -> W.QComboBox:
    widget = W.QComboBox()
    for key, title in options.items():
        widget.addItem(title, key)
    return widget


def button(text: str, action) -> W.QPushButton:
    widget = W.QPushButton(text)
    widget.clicked.connect(action)
    return widget


def details_tree(tree: W.QTreeWidget, data: dict) -> None:
    tree.clear()

    def insert(parent, key, value):
        matrix = isinstance(value, list) and value and isinstance(value[0], list)
        item = W.QTreeWidgetItem(
            [
                parameter_label(key),
                "" if isinstance(value, dict) or matrix else shown(value),
            ]
        )
        item.setToolTip(0, item.text(0))
        item.setToolTip(1, item.text(1))
        if parent:
            parent.addChild(item)
        else:
            tree.addTopLevelItem(item)
        if isinstance(value, dict):
            for child, entry in value.items():
                insert(item, child, entry)
            item.setExpanded(True)
        elif matrix:
            for index, entry in enumerate(value, start=1):
                insert(item, str(index), entry)

    for key, value in data.items():
        insert(None, key, value)


class ScenarioForm(W.QWidget):
    """A generated form backed by the exact accepted scenario keys."""

    edited = QtCore.Signal()

    def __init__(self, session: Workbench, initial: dict) -> None:
        super().__init__()
        self.session = session
        self.editor = {}
        stack = W.QVBoxLayout(self)
        title = W.QLabel("Kịch bản")
        title.setObjectName("section")
        stack.addWidget(title)
        self.study = combo(STUDIES)
        self.case = W.QComboBox()
        stack.addWidget(self.study)
        stack.addWidget(self.case)
        self.schedule_mode = W.QCheckBox("Dùng lịch tùy chỉnh NB6")
        self.schedule_mode.toggled.connect(self.toggle_schedule)
        self.schedule_button = button("Lịch sự kiện…", self.edit_schedule)
        stack.addWidget(self.schedule_mode)
        stack.addWidget(self.schedule_button)
        self.scroll = W.QScrollArea()
        self.scroll.setWidgetResizable(True)
        stack.addWidget(self.scroll, 1)
        self.study.currentIndexChanged.connect(self.change_study)
        self.case.currentIndexChanged.connect(self.change_case)
        self.set_configuration(initial)

    def set_configuration(self, options: dict) -> None:
        study = options.get("study")
        if not isinstance(study, str) or study not in STUDIES:
            raise ValueError("Tệp phải chỉ định một bài thử hợp lệ từ NB1 đến NB6.")
        index = self.study.findData(study)
        case = options.get("case")
        if "case" in options and case not in self.session.cases(study):
            raise ValueError("Kịch bản trong tệp không thuộc bài thử đã chọn.")
        defaults = self.session.defaults(study, options.get("case"))
        configuration = merged(defaults, options)
        if study == "NB6":
            if configuration["schedule"] is not None:
                configuration = self.session.inputs.whole_body_configuration(
                    configuration
                )
            for name in ("friction", "friction_after"):
                if type(configuration[name]) in (int, float):
                    configuration[name] = [configuration[name]] * 2
        for name, default in defaults.items():
            value = configuration[name]
            if type(default) is bool and type(value) is not bool:
                raise ValueError(f"{field_label(name)} phải có giá trị bật hoặc tắt.")
            if name in OPTIONS and (
                not isinstance(value, str) or value not in self.choices(study, name)
            ):
                raise ValueError(
                    f"{field_label(name)} không hợp lệ cho bài thử đã chọn."
                )
            if isinstance(default, list) and (
                not isinstance(value, (list, tuple))
                or len(value) != len(default)
                or not all(map(finite_number, value))
            ):
                raise ValueError(
                    f"{field_label(name)} phải gồm {len(default)} giá trị số hữu hạn."
                )
        self.study.blockSignals(True)
        self.study.setCurrentIndex(index)
        self.study.blockSignals(False)
        self.set_cases(study, configuration.get("case"))
        self.set_editors(configuration, defaults)

    def set_cases(self, study: str, selected: str | None) -> None:
        self.case.blockSignals(True)
        self.case.clear()
        for name in self.session.cases(study):
            self.case.addItem(case_label(study, name), name)
        self.case.setCurrentIndex(max(0, self.case.findData(selected)))
        self.case.setVisible(self.case.count() > 0)
        self.case.blockSignals(False)

    def change_study(self) -> None:
        self.set_cases(self.study.currentData(), None)
        self.change_case()

    def change_case(self) -> None:
        defaults = self.session.defaults(
            self.study.currentData(), self.case.currentData()
        )
        self.set_editors(defaults, defaults)

    @staticmethod
    def choices(study: str, name: str) -> dict[str, str]:
        choices = OPTIONS[name]
        if name == "controller":
            return {key: choices[key] for key in CONTROLLERS[study]}
        if name == "mode":
            keys = (
                ("pose", "trajectory", "load_sweep", "inverse")
                if study == "NB1–2"
                else ("rolling", "compliant", "flight")
            )
            return {key: choices[key] for key in keys}
        return choices

    def set_editors(self, configuration: dict, defaults: dict) -> None:
        self.editor.clear()
        body = W.QWidget()
        layout = W.QFormLayout(body)
        layout.setRowWrapPolicy(W.QFormLayout.WrapAllRows)
        layout.setFieldGrowthPolicy(W.QFormLayout.AllNonFixedFieldsGrow)
        self.scroll.setWidget(body)
        for name, value in configuration.items():
            if name in ("study", "case", "schedule"):
                continue
            default = defaults[name]
            if name in OPTIONS:
                widget = combo(self.choices(configuration["study"], name))
                widget.setCurrentIndex(widget.findData(value))
            elif isinstance(default, bool):
                widget = W.QCheckBox()
                widget.setChecked(value)
            else:
                widget = W.QLineEdit()
                widget.setText(editor_text(value))
                if isinstance(default, list):
                    widget.setToolTip("Các thành phần cách nhau bằng dấu /.")
                if default is None:
                    widget.setPlaceholderText(
                        "Không đổi" if name == "friction_change_s" else "Không áp dụng"
                    )
            widget.setObjectName(name)
            title = field_label(name)
            if isinstance(default, list) and name in (
                "friction",
                "friction_after",
                "support_levels_m",
            ):
                title += " · trái / phải"
            layout.addRow(title, widget)
            self.editor[name] = (widget, default)
            if isinstance(widget, W.QComboBox):
                changed = widget.currentIndexChanged
            elif isinstance(widget, W.QCheckBox):
                changed = widget.toggled
            else:
                changed = widget.textChanged
            changed.connect(lambda *_args: self.edited.emit())
        if configuration["study"] == "NB1–2":
            self.editor["mode"][0].currentIndexChanged.connect(self.kinematic_fields)
            self.kinematic_fields()
        scheduled = configuration.get("schedule")
        self.schedule = scheduled if scheduled is not None else []
        self.schedule_mode.blockSignals(True)
        self.schedule_mode.setChecked(scheduled is not None)
        self.schedule_mode.blockSignals(False)
        self.toggle_schedule()

    def toggle_schedule(self) -> None:
        nb6 = self.study.currentData() == "NB6"
        self.schedule_mode.setVisible(nb6)
        self.schedule_button.setVisible(nb6)
        enabled = self.schedule_mode.isChecked()
        self.schedule_button.setText(
            f"Sửa lịch · {len(self.schedule)} mốc…" if enabled else "Lịch sự kiện…"
        )
        if nb6:
            layout = self.scroll.widget().layout()
            for name in NB6_LEGACY_TIMED_FIELDS:
                widget = self.editor[name][0]
                widget.setVisible(not enabled)
                layout.labelForField(widget).setVisible(not enabled)
        self.edited.emit()

    def edit_schedule(self) -> None:
        dialog = ScheduleDialog(
            self,
            self.schedule,
            lambda entries: self.configuration(schedule_override=entries),
        )
        if dialog.exec_():
            self.schedule = dialog.entries
            self.schedule_mode.setChecked(True)
            self.toggle_schedule()

    def kinematic_fields(self) -> None:
        mode = self.editor["mode"][0].currentData()
        has_path = mode in ("trajectory", "load_sweep")
        for name in ("left_final_deg", "right_final_deg", "duration_s", "period_s"):
            self.editor[name][0].setEnabled(has_path)
        for name in ("inverse_target_mm", "inverse_side"):
            self.editor[name][0].setEnabled(mode == "inverse")
        self.editor["joint_accel_rad_s2"][0].setEnabled(not has_path)
        rate = self.editor["joint_rate_rad_s"][0]
        rate.setEnabled(mode != "trajectory")
        rate.setToolTip(
            "Tốc độ dùng để tra đường bao mô-men tại mỗi góc."
            if mode == "load_sweep"
            else "Vận tốc hông trái / phải."
        )

    def configuration(self, *, schedule_override=None) -> dict:
        selection = {"study": self.study.currentData()}
        if self.case.count():
            selection["case"] = self.case.currentData()
        if schedule_override is not None:
            selection["schedule"] = schedule_override
        elif self.schedule_mode.isChecked():
            selection["schedule"] = self.schedule
        for name, (widget, default) in self.editor.items():
            if "schedule" in selection and name in NB6_LEGACY_TIMED_FIELDS:
                continue
            if isinstance(widget, W.QCheckBox):
                value = widget.isChecked()
            elif isinstance(widget, W.QComboBox):
                value = widget.currentData()
            else:
                try:
                    text = widget.text().strip().replace(",", ".")
                    if isinstance(default, list):
                        value = [float(item.strip()) for item in text.split("/")]
                        if len(value) != len(default) or not all(
                            map(finite_number, value)
                        ):
                            raise ValueError()
                    elif name in INTEGER_SCENARIO_FIELDS:
                        value = int(text)
                    else:
                        value = float(text) if text or default is not None else None
                        if value is not None and not finite_number(value):
                            raise ValueError()
                except ValueError as exc:
                    raise ValueError(
                        f"Giá trị không hợp lệ cho {field_label(name)}."
                    ) from exc
            selection[name] = value
        if selection["study"] == "NB6":
            return self.session.inputs.whole_body_configuration(selection)
        return selection


class WorkbenchWindow(W.QMainWindow):
    def __init__(
        self,
        session: Workbench,
        configuration: dict,
        saved: RunRecord | None = None,
        *,
        rviz: bool = True,
    ) -> None:
        super().__init__()
        self.session = session
        self.run: RunRecord | None = None
        self.comparison: RunRecord | None = None
        self.timeline: list[float] = []
        self.interactive: SpatialRun | None = None
        self.continuous = False
        self.cancel_requested = False
        self.viewer = None
        self.worker = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="duk-simulation"
        )
        self.pending: Future | None = None
        self.setWindowTitle("DUK | Bàn thử mô phỏng")
        self.resize(1360, 830)
        self.setStyleSheet(
            "QMainWindow{background:#f5f7f8} QLabel#section{font-size:17px;font-weight:600;color:#183943}"
            "QLabel#heading{font-size:22px;font-weight:650;color:#183943}"
            "QPushButton{padding:5px 9px} QTreeWidget,QTableWidget{alternate-background-color:#f0f5f6}"
        )
        central = W.QWidget()
        self.setCentralWidget(central)
        layout = W.QVBoxLayout(central)
        header = W.QHBoxLayout()
        heading = W.QLabel("DUK   /   Bàn thử mô phỏng")
        heading.setObjectName("heading")
        header.addWidget(heading)
        self.profile = W.QLabel(
            session.inputs.parameter_info["profile_name"] + " · Tham số tổng hợp"
        )
        header.addWidget(self.profile)
        self.profile_button = button("Chạy với bộ tham số…", self.open_profile)
        header.addWidget(self.profile_button)
        header.addStretch()
        self.viewer_button = button("Mở RViz", self.open_viewer)
        header.addWidget(self.viewer_button)
        layout.addLayout(header)
        columns = W.QSplitter()
        layout.addWidget(columns, 1)
        left = W.QWidget()
        tools = W.QVBoxLayout(left)
        self.scenario = ScenarioForm(session, configuration)
        tools.addWidget(self.scenario, 1)
        self.draft_notice = W.QLabel("Cấu hình chờ chạy")
        self.draft_notice.setWordWrap(True)
        tools.addWidget(self.draft_notice)
        self.scenario.edited.connect(self.mark_draft)
        self.new_button = button("Kịch bản mới", self.new_scenario)
        tools.addWidget(self.new_button)
        self.run_button = button("Chạy mô phỏng", self.run_form)
        self.run_button.setObjectName("run")
        self.run_button.setStyleSheet(
            "background:#176d66;color:white;font-weight:600;padding:9px"
        )
        tools.addWidget(self.run_button)
        self.prepare_button = button("Chạy từng bước NB6", self.prepare_interactive)
        tools.addWidget(self.prepare_button)
        self.step_controls = W.QWidget()
        step_row = W.QHBoxLayout(self.step_controls)
        step_row.setContentsMargins(0, 0, 0, 0)
        self.step_button = button("Tính 1 bước", self.advance_interactive)
        self.continue_button = button("Chạy tiếp", self.toggle_interactive)
        self.cancel_button = button("Hủy ca", self.cancel_interactive)
        for control in (self.step_button, self.continue_button, self.cancel_button):
            step_row.addWidget(control)
        tools.addWidget(self.step_controls)
        self.open_scenario_button = button("Nạp kịch bản JSON…", self.open_scenario)
        tools.addWidget(self.open_scenario_button)
        tools.addWidget(button("Lưu kịch bản JSON…", self.save_scenario))
        columns.addWidget(left)

        middle = W.QWidget()
        content = W.QVBoxLayout(middle)
        self.status = W.QLabel("Đang chuẩn bị mô phỏng…")
        self.status.setWordWrap(True)
        content.addWidget(self.status)
        toolbar = W.QHBoxLayout()
        self.signal = W.QComboBox()
        self.signal.setSizeAdjustPolicy(
            W.QComboBox.AdjustToMinimumContentsLengthWithIcon
        )
        self.signal.setMinimumContentsLength(16)
        self.signal.setToolTip("Chọn tín hiệu để vẽ.")
        self.signal.currentIndexChanged.connect(self.draw_plot)
        toolbar.addWidget(self.signal, 1)
        self.compare_button = button("So sánh điều khiển…", self.compare_controller)
        self.compare_button.setEnabled(False)
        toolbar.addWidget(self.compare_button)
        content.addLayout(toolbar)

        self.figure = Figure(facecolor="#ffffff", layout="constrained")
        self.axes = self.figure.add_subplot()
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.mpl_connect("button_press_event", self.plot_seek)
        content.addWidget(self.canvas, 1)
        self.wheel_readouts = W.QGroupBox("Tiếp xúc và mô-men tại mẫu đang phát")
        wheel_layout = W.QHBoxLayout(self.wheel_readouts)
        self.wheel_columns = {}
        for wheel_side, title, color in (
            ("left", "Bánh trái", "#a74717"),
            ("right", "Bánh phải", "#135b89"),
        ):
            column = W.QVBoxLayout()
            heading = W.QLabel(title)
            heading.setStyleSheet(f"font-weight: 600; color: {color}")
            column.addWidget(heading)
            values = W.QLabel()
            values.setTextFormat(QtCore.Qt.PlainText)
            values.setWordWrap(True)
            column.addWidget(values)
            wheel_layout.addLayout(column, 1)
            self.wheel_columns[wheel_side] = values
        self.wheel_readouts.hide()
        content.addWidget(self.wheel_readouts)
        player = W.QHBoxLayout()
        self.play_button = button("Phát", self.toggle_play)
        self.play_button.setToolTip("Phát lại dữ liệu đã tính.")
        self.play_button.setEnabled(False)
        player.addWidget(self.play_button)
        self.slider = W.QSlider(QtCore.Qt.Horizontal)
        self.slider.setObjectName("time_slider")
        self.slider.setRange(0, 0)
        self.slider.valueChanged.connect(self.update_cursor)
        player.addWidget(self.slider, 1)
        self.time = W.QLabel("0 s")
        self.time.setMinimumWidth(115)
        player.addWidget(self.time)
        content.addLayout(player)
        files = W.QHBoxLayout()
        results = W.QToolButton()
        results.setText("Tệp kết quả")
        results.setPopupMode(W.QToolButton.InstantPopup)
        menu = W.QMenu(results)
        self.open_result_action = menu.addAction("Mở kết quả…", self.open_run)
        self.save_result_action = menu.addAction("Lưu kết quả…", self.save_run)
        self.export_result_action = menu.addAction("Xuất CSV…", self.export_csv)
        self.compare_result_action = menu.addAction(
            "Đối chiếu tệp…", self.open_comparison
        )
        self.verify_result_action = menu.addAction("Chạy lại và kiểm tra", self.verify)
        menu.aboutToShow.connect(self.update_result_actions)
        results.setMenu(menu)
        files.addWidget(results)
        files.addStretch()
        content.addLayout(files)
        columns.addWidget(middle)

        inspect = W.QWidget()
        side = W.QVBoxLayout(inspect)
        self.objects = W.QTreeWidget()
        self.objects.setHeaderLabel("Đối tượng")
        self.populate_objects()
        self.objects.currentItemChanged.connect(self.object_changed)
        side.addWidget(self.objects, 1)
        self.frame = combo(
            {"all": "Mọi hệ tọa độ", "body": "Hệ thân / cơ cấu", "world": "Hệ thế giới"}
        )
        self.frame.currentIndexChanged.connect(self.object_changed)
        side.addWidget(self.frame)
        self.tabs = W.QTabWidget()
        self.signals = W.QTableWidget(0, 3)
        self.signals.setHorizontalHeaderLabels(["Tín hiệu", "Giá trị", "Đối chiếu"])
        self.signals.setColumnHidden(2, True)
        self.signals.horizontalHeader().setSectionResizeMode(0, W.QHeaderView.Stretch)
        self.signals.setColumnWidth(1, 116)
        self.signals.setColumnWidth(2, 116)
        self.signals.verticalHeader().hide()
        self.signals.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        self.signals.setAlternatingRowColors(True)
        self.signals.cellDoubleClicked.connect(self.pick_signal)
        self.equations = W.QTreeWidget()
        self.equations.setHeaderLabels(["Phương trình / biến", "Giá trị"])
        self.events = W.QTreeWidget()
        self.events.setHeaderLabels(["Sự kiện", "Yêu cầu / thực tế (s)"])
        self.events.itemActivated.connect(self.seek_event)
        self.parameters = W.QTreeWidget()
        self.parameters.setHeaderLabels(["Tham số", "Giá trị"])
        for name, widget in (
            ("Tín hiệu", self.signals),
            ("Phương trình", self.equations),
            ("Sự kiện", self.events),
            ("Tham số", self.parameters),
        ):
            self.tabs.addTab(widget, name)
        self.tabs.currentChanged.connect(self.update_cursor)
        side.addWidget(self.tabs, 2)
        columns.addWidget(inspect)
        for index, stretch in enumerate((0, 1, 0)):
            columns.setStretchFactor(index, stretch)
        columns.setSizes([300, 660, 400])

        self.play_timer = QtCore.QTimer(self)
        self.play_timer.setInterval(25)
        self.play_timer.timeout.connect(self.play_tick)
        self.viewer_timer = QtCore.QTimer(self)
        self.viewer_timer.setInterval(80)
        self.viewer_timer.timeout.connect(self.publish_frame)
        self.work_timer = QtCore.QTimer(self)
        self.work_timer.setInterval(30)
        self.work_timer.timeout.connect(self.finish_work)
        self.work_timer.start()
        self.scenario.study.currentIndexChanged.connect(self.update_run_controls)
        self.update_run_controls()
        if rviz:
            QtCore.QTimer.singleShot(0, self.open_viewer)
        if saved:
            self.set_run(saved)
        else:
            self.submit(partial(session.run, configuration), self.set_run)

    def populate_objects(self) -> None:
        def node(parent, name, scope):
            item = W.QTreeWidgetItem(parent, [name])
            item.setData(0, ROLE, scope)
            return item

        self.root = node(self.objects, "Toàn robot", "robot")
        node(self.root, "Thân", "body")
        for side, name in (("left", "Chân trái"), ("right", "Chân phải")):
            limb = node(self.root, name, side)
            node(limb, "Khớp hông", side + ".hip")
            node(limb, "Bánh", side + ".wheel")
            linkage = node(limb, "Cơ cấu bốn khâu", side + ".linkage")
            for part in (*SEGMENTS, *POINTS):
                node(
                    linkage,
                    part if len(part) > 1 else "Điểm " + part,
                    side + ".linkage." + part,
                )
            limb.setExpanded(True)
        node(self.root, "Cặp bánh (NB4/NB5)", "wheel_pair")
        node(self.root, "Điều khiển", "control")
        self.root.setExpanded(True)
        self.objects.setCurrentItem(self.root)

    @property
    def scope(self) -> str:
        return self.objects.currentItem().data(0, ROLE)

    def submit(self, operation, callback, *, interactive: bool = False) -> None:
        if self.pending is not None or (self.interactive and not interactive):
            return
        self.pause()
        self.finished = callback
        self.pending_interactive = interactive
        self.pending = self.worker.submit(operation)
        self.update_run_controls()
        if not interactive:
            self.status.setText("Đang tính toán…")

    def finish_work(self) -> None:
        if self.pending is None or not self.pending.done():
            return
        future, self.pending = self.pending, None
        interactive = self.pending_interactive
        try:
            self.finished(future.result())
        except (
            ValueError,
            RuntimeError,
            OSError,
            KeyError,
            TypeError,
            IndexError,
        ) as exc:
            if interactive:
                self.clear_interactive()
            self.status.setText(str(exc))
            W.QMessageBox.warning(self, "Bàn thử mô phỏng", str(exc))
        finally:
            self.update_run_controls()

    def update_run_controls(self, *_args) -> None:
        active = self.interactive is not None
        idle = self.pending is None
        editable = idle and not active
        for control in (
            self.scenario,
            self.new_button,
            self.run_button,
            self.profile_button,
            self.open_scenario_button,
            self.prepare_button,
        ):
            control.setEnabled(editable)
        self.compare_button.setEnabled(
            editable and self.run is not None and self.run.study in CONTROLLERS
        )
        self.prepare_button.setVisible(
            not active and self.scenario.study.currentData() == "NB6"
        )
        self.step_controls.setVisible(active)
        can_step = active and idle and not self.continuous and not self.cancel_requested
        self.step_button.setEnabled(can_step)
        self.continue_button.setEnabled(
            can_step or (self.continuous and not self.cancel_requested)
        )
        self.continue_button.setText(
            "Tạm dừng tính" if self.continuous else "Chạy tiếp"
        )
        self.cancel_button.setEnabled(active and not self.cancel_requested)

    def prepare_interactive(self) -> None:
        try:
            options = self.scenario.configuration()
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        self.status.setText("Đang chuẩn bị ca NB6…")
        self.submit(
            partial(self.session.begin_nb6, options),
            self.interactive_ready,
            interactive=True,
        )

    def interactive_ready(self, run: SpatialRun) -> None:
        self.interactive = run
        self.status.setText("NB6 · Sẵn sàng tính bước đầu ở 0 s")
        self.draft_notice.setText(
            "Kịch bản đã chốt · đang xem kết quả trước"
            if self.run
            else "Kịch bản đã chốt"
        )

    def advance_interactive(self) -> None:
        if self.interactive is not None and self.pending is None:
            action = "Đang tính" if self.continuous else "Đang tính một bước"
            self.status.setText(
                f"NB6 · {action} · {decimal(self.interactive.runner.time_s)} s"
            )
            self.submit(
                self.interactive.runner.advance,
                self.interactive_advanced,
                interactive=True,
            )

    def toggle_interactive(self) -> None:
        if self.continuous:
            self.continuous = False
            self.status.setText("Sẽ tạm dừng sau bước tính đang chạy…")
        elif (
            self.interactive is not None
            and self.pending is None
            and not self.cancel_requested
        ):
            self.continuous = True
            self.advance_interactive()
        self.update_run_controls()

    def interactive_advanced(self, _time_s: float) -> None:
        if (
            self.interactive.runner.finished
            or self.cancel_requested
            or not self.continuous
        ):
            self.collect_interactive()
            return
        self.advance_interactive()

    def collect_interactive(self) -> None:
        self.submit(
            partial(self.interactive.record, cancelled=self.cancel_requested),
            self.show_interactive,
            interactive=True,
        )

    def show_interactive(self, record: RunRecord) -> None:
        if self.cancel_requested and record.stop_reason == "paused":
            self.collect_interactive()
            return
        if record.stop_reason != "paused":
            self.clear_interactive()
        self.set_run(record, follow_end=True)
        if record.stop_reason == "paused":
            self.draft_notice.setText("Ca NB6 tạm dừng · có thể chạy tiếp")

    def cancel_interactive(self) -> None:
        if self.interactive is None:
            return
        self.continuous = False
        self.cancel_requested = True
        self.status.setText("Đang chốt ca tại biên bước tính…")
        if self.pending is None:
            if self.interactive.runner.has_samples:
                self.collect_interactive()
            else:
                self.clear_interactive()
                self.mark_draft()
                self.status.setText("Đã hủy trước khi tính bước đầu.")
        self.update_run_controls()

    def clear_interactive(self) -> None:
        self.interactive = None
        self.continuous = False
        self.cancel_requested = False

    def mark_draft(self) -> None:
        self.draft_notice.setText(
            "Kịch bản chưa chạy · đang xem kết quả trước"
            if self.run
            else "Kịch bản chưa chạy"
        )

    def update_result_actions(self) -> None:
        ready = self.pending is None and self.interactive is None
        available = self.run is not None
        self.open_result_action.setEnabled(ready)
        self.save_result_action.setEnabled(available)
        self.export_result_action.setEnabled(available)
        self.compare_result_action.setEnabled(ready and available)
        self.verify_result_action.setEnabled(
            ready and available and self.run.stop_reason not in ("paused", "cancelled")
        )

    def new_scenario(self) -> None:
        study = self.scenario.study.currentData()
        configuration = self.session.defaults(study)
        if study == "NB6":
            configuration["schedule"] = []
        self.scenario.set_configuration(configuration)

    def run_form(self) -> None:
        try:
            options = self.scenario.configuration()
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        self.submit(partial(self.session.run, options), self.set_run)

    def set_run(self, run: RunRecord, *, follow_end: bool = False) -> None:
        self.pause()
        self.scenario.set_configuration(run.configuration)
        self.run, self.comparison = run, None
        self.draft_notice.setText("Kịch bản của kết quả đang xem")
        self.wheel_readouts.setVisible(
            any(
                name in run.signals
                for name in ("left_normal_force", "left_hip_static_effort")
            )
        )
        self.signals.setColumnHidden(2, True)
        self.set_timeline(run.time_s[-1] if follow_end else run.time_s[0])
        self.update_run_controls()
        details_tree(
            self.parameters,
            {
                "Bộ tham số": run.provenance.get("profile_name"),
                "Nguồn": run.provenance.get("profile"),
                "Phiên bản Git": run.provenance.get("git_commit"),
                "Điều kiện": {
                    key: configuration_value(run.study, key, value)
                    for key, value in run.configuration.items()
                },
                **run.metadata,
            },
        )
        self.events.clear()
        for event in sorted(
            run.events,
            key=lambda item: (
                item["actual_s"] if item["actual_s"] is not None else float("inf")
            ),
        ):
            name = event["name"]
            channel = name.removeprefix("schedule.")
            if name.startswith("schedule.") and channel in SCHEDULE_LABELS:
                value = event.get("value")
                if channel == "fault" and isinstance(value, str):
                    value = OPTIONS["fault"].get(value, value)
                elif type(value) is bool:
                    value = "Bật" if value else "Tắt"
                name = f"{SCHEDULE_LABELS[channel]} · {shown(value)}"
            item = W.QTreeWidgetItem(
                self.events,
                [
                    shown(name),
                    f"{decimal(event['requested_s'])} / {decimal(event['actual_s'])}",
                ],
            )
            item.setData(0, ROLE, event["actual_s"])
            item.setToolTip(0, item.text(0))
        status = f"{self.run_label(run)} · {shown(run.stop_reason)} · {decimal(run.time_s[-1])} s"
        if not self.session.matches_profile(run):
            status += " · Cần bộ tham số gốc để xem RViz và phương trình"
        self.status.setText(status)
        self.object_changed()

    def set_timeline(self, keep: float) -> None:
        self.timeline = self.run.time_s
        if self.comparison:
            self.timeline = sorted(set(self.timeline).union(self.comparison.time_s))
        self.slider.blockSignals(True)
        self.slider.setRange(0, len(self.timeline) - 1)
        self.seek(keep)
        self.slider.blockSignals(False)
        self.play_button.setEnabled(len(self.timeline) > 1)

    @staticmethod
    def index_at(run: RunRecord, time: float) -> int | None:
        if run.time_s[0] <= time <= run.time_s[-1]:
            return bisect.bisect_right(run.time_s, time) - 1
        return None

    def displayed_sample(self, time: float) -> tuple[RunRecord | None, int | None]:
        for run in (self.run, self.comparison):
            if run is not None:
                index = self.index_at(run, time)
                if index is not None:
                    return run, index
        return None, None

    @staticmethod
    def run_label(run: RunRecord) -> str:
        control = run.configuration.get("controller")
        return CONTROLLER_LABELS.get(
            control, case_label(run.study, run.configuration.get("case"))
        )

    def scoped_signals(self, run: RunRecord) -> dict:
        scope = self.scope
        frame = self.frame.currentData()
        base, separator, part = scope.rpartition(".")
        related = set()
        if separator and part in SEGMENTS:
            related.update(base + "." + point for point in part)
        if scope in ("left.linkage", "right.linkage"):
            related.add(scope.replace(".linkage", ".hip"))

        def visible_frame(signal):
            if frame == "all":
                return True
            if signal.frame.startswith("world"):
                return frame == "world"
            if signal.frame in ("body", "body x/z", "leg x/z", "carrier"):
                return frame == "body"
            return True

        return {
            key: value
            for key, value in run.signals.items()
            if (
                scope == "robot"
                or value.scope == scope
                or value.scope.startswith(scope + ".")
                or value.scope in related
            )
            and visible_frame(value)
        }

    def object_changed(self, *_args) -> None:
        if self.run is None:
            return
        current = self.signal.currentData()
        self.signal.blockSignals(True)
        self.signal.clear()
        candidates = self.scoped_signals(self.run)
        for name, signal in candidates.items():
            if signal.plot_components() is not None:
                unit = unit_label(signal.unit)
                self.signal.addItem(
                    f"{label(name)} ({unit})" if unit else label(name), name
                )
        if self.signal.findData(current) < 0:
            preferred = next(
                (
                    name
                    for name in (
                        "height",
                        "lean",
                        "model_speed",
                        "left_slip_speed",
                        "right_slip_speed",
                        "left_hip_applied",
                        "right_hip_applied",
                    )
                    if self.signal.findData(name) >= 0
                ),
                None,
            )
            current = preferred
        self.signal.setCurrentIndex(max(0, self.signal.findData(current)))
        self.signal.blockSignals(False)
        self.draw_plot()
        self.update_cursor()

    def draw_plot(self, *_args) -> None:
        self.axes.clear()
        key = self.signal.currentData()
        if not self.run:
            return
        component_count = None
        for index, run in enumerate((self.run, self.comparison)):
            if not run or key not in run.signals:
                continue
            signal = run.signals[key]
            if not signal.same_quantity(self.run.signals[key]):
                continue
            axes = signal.plot_components()
            if axes is None:
                continue
            if index == 0:
                component_count = len(axes)
            elif len(axes) != component_count:
                continue
            for component, values in enumerate(axes):
                times = run.time_s
                if signal.held and len(values) > 1 and np.isnan(values[-1]):
                    values[-1] = values[-2]
                name = self.run_label(run)
                if self.comparison and self.run_label(
                    self.comparison
                ) == self.run_label(self.run):
                    name += " · " + ("Gốc", "Đối chiếu")[index]
                if len(axes) > 1:
                    name = f"{name} · {component + 1}"
                self.axes.plot(
                    times,
                    values,
                    linewidth=1.6,
                    color=COLOR[index],
                    alpha=1 - component / (1.6 * len(axes)),
                    label=name,
                    drawstyle="steps-post"
                    if signal.held or any(type(v) is bool for v in signal.values)
                    else "default",
                    marker="o" if len(times) == 1 else None,
                    linestyle=("-", "--", ":", "-.")[component % 4],
                )
        self.axes.set_xlabel("Thời gian (s)")
        if key:
            self.axes.set_ylabel(unit_label(self.run.signals[key].unit))
        for axis in (self.axes.xaxis, self.axes.yaxis):
            axis.set_major_formatter(
                FuncFormatter(lambda value, _pos: decimal(value, 6))
            )
        self.axes.grid(alpha=0.20)
        if self.axes.get_legend_handles_labels()[0]:
            self.axes.legend(loc="best", fontsize="small")
        for index, run in enumerate((self.run, self.comparison)):
            if run is not None:
                for e in run.events:
                    if e["actual_s"] is not None:
                        self.axes.axvline(
                            e["actual_s"], color=COLOR[index], linewidth=0.7, alpha=0.25
                        )
        self.cursor = self.axes.axvline(
            self.timeline[self.slider.value()], linewidth=1.15, color="#273a42"
        )
        self.canvas.draw_idle()

    def update_cursor(self, *_args) -> None:
        if self.run is None:
            return
        time = self.timeline[self.slider.value()]
        run, index = self.displayed_sample(time)
        suffix = " · Đối chiếu" if run is not None and run is self.comparison else ""
        self.time.setText(f"{decimal(time)} s{suffix}")
        if not self.wheel_readouts.isHidden():
            for side, values in self.wheel_columns.items():
                values.setText(
                    "\n".join(wheel_readout(run, index, side)) if run else "—"
                )
        if hasattr(self, "cursor"):
            self.cursor.set_xdata([time, time])
            self.canvas.draw_idle()
        if self.tabs.currentWidget() == self.equations:
            try:
                data = (
                    self.session.equations(run, index, self.scope)
                    if run
                    else {"Mẫu": "Không có dữ liệu tại mốc này."}
                )
            except (ValueError, KeyError, TypeError, IndexError) as exc:
                data = {"Mẫu": str(exc)}
            details_tree(self.equations, data)
        if self.tabs.currentWidget() == self.signals:
            self.update_signals(time)
        self.publish_frame()

    def update_signals(self, time: float) -> None:
        values = self.scoped_signals(self.run)
        self.signals.setRowCount(len(values))
        index = self.index_at(self.run, time)
        compare_index = (
            self.index_at(self.comparison, time) if self.comparison else None
        )
        for row, (name, value) in enumerate(values.items()):
            unit = unit_label(value.unit)
            title = W.QTableWidgetItem(
                f"{label(name)} [{unit}]" if unit else label(name)
            )
            title.setData(ROLE, name)
            if value.frame:
                title.setToolTip(title.text() + "\n" + frame_label(value.frame))
            else:
                title.setToolTip(title.text())
            self.signals.setItem(row, 0, title)
            sample = W.QTableWidgetItem(
                shown(value.values[index] if index is not None else None)
            )
            sample.setToolTip(sample.text())
            self.signals.setItem(row, 1, sample)
            if self.comparison:
                comparison = self.comparison.signals.get(name)
                matching = comparison is not None and value.same_quantity(comparison)
                datum = (
                    comparison.values[compare_index]
                    if matching and compare_index is not None
                    else None
                )
                sample = W.QTableWidgetItem(shown(datum))
                sample.setToolTip(sample.text())
                self.signals.setItem(row, 2, sample)

    def open_viewer(self) -> None:
        if self.viewer:
            return
        try:
            from .viewer import Viewer

            self.viewer = Viewer(self.session.inputs)
            self.viewer_timer.start()
            self.publish_frame()
        except (ImportError, LookupError, OSError, RuntimeError) as exc:
            self.viewer_button.setText("Mở RViz")
            self.viewer_button.setToolTip(str(exc))
            self.status.setText("RViz chưa kết nối · " + str(exc))
            return

    def publish_frame(self) -> None:
        if not self.viewer:
            return
        run, index = (
            self.displayed_sample(self.timeline[self.slider.value()])
            if self.timeline
            else (None, None)
        )
        if run is not None and not self.session.matches_profile(run):
            run = None
        try:
            complete = self.viewer.send(run, 0 if index is None else index)
            self.viewer_button.setText(
                "RViz đã kết nối" if complete else "RViz · tư thế chưa khả dụng"
            )
        except (RuntimeError, ValueError, KeyError, TypeError) as exc:
            self.viewer_timer.stop()
            self.viewer.close()
            self.viewer = None
            self.viewer_button.setText("Mở RViz")
            self.viewer_button.setToolTip(str(exc))

    def seek(self, time: float) -> None:
        if self.timeline:
            self.slider.setValue(max(0, bisect.bisect_right(self.timeline, time) - 1))

    def plot_seek(self, event) -> None:
        if event.inaxes == self.axes and event.xdata is not None:
            self.seek(event.xdata)

    def seek_event(self, item, _column) -> None:
        time = item.data(0, ROLE)
        if time is not None:
            self.seek(time)

    def pick_signal(self, row, _column) -> None:
        index = self.signal.findData(self.signals.item(row, 0).data(ROLE))
        if index >= 0:
            self.signal.setCurrentIndex(index)

    def pause(self) -> None:
        self.play_timer.stop()
        self.play_button.setText("Phát")

    def toggle_play(self) -> None:
        if self.play_timer.isActive():
            self.pause()
        elif len(self.timeline) > 1:
            if self.slider.value() == self.slider.maximum():
                self.slider.setValue(0)
            self.play_start = monotonic() - self.timeline[self.slider.value()]
            self.play_button.setText("Tạm dừng")
            self.play_timer.start()

    def play_tick(self) -> None:
        self.seek(monotonic() - self.play_start)
        if self.slider.value() == self.slider.maximum():
            self.pause()

    def compare_controller(self) -> None:
        if (
            not self.run
            or self.run.study not in CONTROLLERS
            or self.pending is not None
        ):
            return
        if not self.session.matches_profile(self.run):
            self.status.setText("Mở lại với bộ tham số gốc để chạy so sánh.")
            return
        names = {
            CONTROLLER_LABELS[key]: key
            for key in CONTROLLERS[self.run.study]
            if key != self.run.configuration["controller"]
        }
        label_text, ok = W.QInputDialog.getItem(
            self,
            "Đối chiếu điều khiển",
            "Dùng cùng kịch bản và giá trị khởi tạo bộ sinh nhiễu",
            list(names),
            editable=False,
        )
        if ok:
            self.submit(
                partial(
                    self.session.run,
                    self.run.configuration | {"controller": names[label_text]},
                ),
                self.set_comparison,
            )

    def set_comparison(self, run: RunRecord) -> None:
        if not self.run or self.run.study != run.study:
            raise ValueError("Chọn kết quả từ cùng mô hình để so sánh.")
        self.pause()
        current_time = self.timeline[self.slider.value()]
        self.comparison = run
        self.set_timeline(current_time)
        self.signals.setColumnHidden(2, False)
        self.status.setText(
            f"{self.run_label(self.run)} · {shown(self.run.stop_reason)} ({decimal(self.run.time_s[-1])} s)"
            f"   |   {self.run_label(run)} · {shown(run.stop_reason)} ({decimal(run.time_s[-1])} s)"
        )
        self.draw_plot()
        self.update_cursor()

    def verify(self) -> None:
        if self.run is not None:
            self.submit(
                partial(self.session.verify_replay, self.run), self.verification_passed
            )

    def verification_passed(self, _run: RunRecord) -> None:
        self.status.setText(
            "Tái lập khớp trong dung sai · tuyệt đối < 0,00000001 · tương đối < 0,0000001"
        )

    def read_file(self, title, callback) -> None:
        if self.pending is not None or self.interactive is not None:
            return
        path, _ = W.QFileDialog.getOpenFileName(self, title, "", "JSON (*.json)")
        if path:
            try:
                callback(Path(path))
            except (ValueError, OSError, KeyError, TypeError) as exc:
                self.status.setText(str(exc))
                W.QMessageBox.warning(self, title, str(exc))

    def open_scenario(self) -> None:
        from .inputs import read_object

        self.read_file(
            "Nạp kịch bản",
            lambda path: self.scenario.set_configuration(read_object(path)),
        )

    def open_profile(self) -> None:
        self.read_file("Chạy với bộ tham số tổng hợp", self.run_with_profile)

    def run_with_profile(self, path: Path) -> None:
        options = self.scenario.configuration()
        study, case = options["study"], options.get("case")
        defaults = self.session.defaults(study, case)
        overrides = {
            key: value for key, value in options.items() if value != defaults[key]
        }
        current = self.run

        def simulate():
            session = Workbench(Inputs(path))
            # A matching saved record already contains its fully resolved inputs.
            config = (
                options
                if current and session.matches_profile(current)
                else session.defaults(study, case) | overrides
            )
            return session, session.run(config)

        def show(result):
            session, run = result
            reopen = self.viewer is not None
            if reopen:
                self.viewer_timer.stop()
                self.viewer.close()
                self.viewer = None
            self.session = self.scenario.session = session
            self.profile.setText(
                session.inputs.parameter_info["profile_name"] + " · Tham số tổng hợp"
            )
            self.set_run(run)
            if reopen:
                self.open_viewer()

        self.submit(simulate, show)

    def open_run(self) -> None:
        self.read_file("Mở kết quả", lambda path: self.set_run(RunRecord.load(path)))

    def open_comparison(self) -> None:
        self.read_file(
            "Mở kết quả đối chiếu",
            lambda path: self.set_comparison(RunRecord.load(path)),
        )

    def write_file(self, title, extension, action) -> None:
        path, _ = W.QFileDialog.getSaveFileName(
            self, title, "", f"{extension.upper()} (*.{extension})"
        )
        if path:
            try:
                action(Path(path))
                self.status.setText(f"Đã lưu {Path(path).name}")
            except (ValueError, OSError) as exc:
                self.status.setText(str(exc))

    def save_run(self) -> None:
        if self.run:
            self.write_file("Lưu kết quả và kịch bản", "json", self.run.save)

    def save_scenario(self) -> None:
        try:
            configuration = self.scenario.configuration()
        except ValueError as exc:
            self.status.setText(str(exc))
            return
        self.write_file(
            "Lưu kịch bản", "json", lambda path: save_json(path, configuration)
        )

    def export_csv(self) -> None:
        if self.run:
            self.write_file("Xuất tín hiệu", "csv", self.run.export_csv)

    def closeEvent(self, event) -> None:
        self.viewer_timer.stop()
        self.work_timer.stop()
        self.pause()
        if self.viewer:
            self.viewer.close()
        self.worker.shutdown(wait=False, cancel_futures=True)
        super().closeEvent(event)


def run_qt(
    session: Workbench, configuration: dict, saved: RunRecord | None, *, rviz: bool
) -> None:
    app = W.QApplication([])
    window = WorkbenchWindow(session, configuration, saved, rviz=rviz)
    window.show()
    app.exec_()
