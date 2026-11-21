"""Edit the time-stamped inputs of an NB6 scenario."""

from collections.abc import Callable

import numpy as np
from python_qt_binding import QtWidgets as W

from duk_core.simulation.whole_body_schedule import SCHEDULE_FIELDS

from .ui_text import OPTIONS, SCHEDULE_LABELS


def editor_text(value) -> str:
    if value is None:
        return ""
    if isinstance(value, (list, tuple)):
        return " / ".join(map(editor_text, value))
    if type(value) is float:
        return np.format_float_positional(value, trim="-").replace(".", ",")
    return str(value)


class ScheduleDialog(W.QDialog):
    def __init__(self, parent, entries: list[dict], validate: Callable[[list], dict]):
        super().__init__(parent)
        self.setWindowTitle("Lịch sự kiện NB6")
        self.resize(730, 410)
        self.validate = validate
        self.entries = entries
        layout = W.QVBoxLayout(self)
        hint = W.QLabel(
            "Mỗi dòng áp ở mẫu điều khiển đầu tiên từ thời điểm yêu cầu. "
            "Lịch thay các mốc tiến, quay, đổi bám, nhiễu và lỗi của ca mẫu. "
            "Lệnh tiến và quay bắt đầu ở 0. Bật tác vụ độ cao để áp dụng độ lệch cao độ đặt."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.table = W.QTableWidget(0, 3)
        self.table.setObjectName("schedule_table")
        self.table.setHorizontalHeaderLabels(["Thời điểm (s)", "Tác động", "Giá trị"])
        self.table.horizontalHeader().setSectionResizeMode(1, W.QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(2, W.QHeaderView.Stretch)
        self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows)
        layout.addWidget(self.table)
        actions = W.QHBoxLayout()
        add = W.QPushButton("Thêm mốc")
        add.clicked.connect(lambda _checked: self.add_row())
        remove = W.QPushButton("Xóa mốc đã chọn")
        remove.clicked.connect(self.remove_selected)
        remove.setEnabled(False)
        self.table.itemSelectionChanged.connect(
            lambda: remove.setEnabled(bool(self.table.selectionModel().selectedRows()))
        )
        actions.addWidget(add)
        actions.addWidget(remove)
        actions.addStretch()
        layout.addLayout(actions)
        self.error = W.QLabel()
        self.error.setWordWrap(True)
        layout.addWidget(self.error)
        buttons = W.QDialogButtonBox(W.QDialogButtonBox.Ok | W.QDialogButtonBox.Cancel)
        buttons.button(W.QDialogButtonBox.Ok).setText("Áp dụng lịch")
        buttons.button(W.QDialogButtonBox.Cancel).setText("Hủy")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        for entry in entries:
            self.add_row(entry)

    def add_row(self, entry: dict | None = None) -> None:
        entry = entry or {}
        row = self.table.rowCount()
        self.table.insertRow(row)
        when = W.QTableWidgetItem(editor_text(entry.get("time_s")))
        self.table.setItem(row, 0, when)
        field = W.QComboBox()
        for key, label in SCHEDULE_LABELS.items():
            field.addItem(label, key)
        field.setCurrentIndex(field.findData(entry.get("field", "drive_speed_m_s")))
        self.table.setCellWidget(row, 1, field)
        self.set_value_editor(row, field.currentData(), entry.get("value"))
        field.currentIndexChanged.connect(lambda _index: self.change_field(field))
        self.table.setCurrentCell(row, 0)

    def change_field(self, field: W.QComboBox) -> None:
        for row in range(self.table.rowCount()):
            if self.table.cellWidget(row, 1) is field:
                self.set_value_editor(row, field.currentData())
                break

    def set_value_editor(self, row: int, field: str, value=None) -> None:
        kind = SCHEDULE_FIELDS[field]
        if kind in ("fault", "boolean"):
            editor = W.QComboBox()
            options = (
                OPTIONS["fault"] if kind == "fault" else {True: "Bật", False: "Tắt"}
            )
            for key, label in options.items():
                editor.addItem(label, key)
            editor.setCurrentIndex(max(0, editor.findData(value)))
        else:
            editor = W.QLineEdit(editor_text(value))
            editor.setPlaceholderText(
                "x / y / z trong hệ thế giới" if kind == "vector" else "Nhập giá trị"
            )
        self.table.setCellWidget(row, 2, editor)

    def remove_selected(self) -> None:
        selected = self.table.selectionModel().selectedRows()
        for index in sorted(selected, key=lambda index: index.row(), reverse=True):
            self.table.removeRow(index.row())

    def changes(self) -> list[dict]:
        entries = []
        for row in range(self.table.rowCount()):
            field = self.table.cellWidget(row, 1).currentData()
            editor = self.table.cellWidget(row, 2)
            try:
                when = float(self.table.item(row, 0).text().replace(",", "."))
                if isinstance(editor, W.QComboBox):
                    value = editor.currentData()
                else:
                    raw = editor.text().replace(",", ".")
                    value = (
                        [float(part) for part in raw.split("/")]
                        if SCHEDULE_FIELDS[field] == "vector"
                        else float(raw)
                    )
            except ValueError as exc:
                raise ValueError(
                    f"Lịch NB6 · dòng {row + 1} cần thời điểm và giá trị hợp lệ."
                ) from exc
            entries.append({"time_s": when, "field": field, "value": value})
        return entries

    def accept(self) -> None:
        try:
            self.entries = self.validate(self.changes())["schedule"]
        except ValueError as exc:
            self.error.setText(str(exc))
            return
        super().accept()
