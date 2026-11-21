"""Validate time-stamped inputs to the sampled whole-body experiment."""

from collections.abc import Sequence
from dataclasses import dataclass
from math import isfinite
from typing import Any

import numpy as np

SCHEDULE_FIELDS = {
    "drive_speed_m_s": "signed",
    "turn_rate_rad_s": "signed",
    "height_offset_m": "signed",
    "friction_left": "nonnegative",
    "friction_right": "nonnegative",
    "velocity_impulse_m_s": "vector",
    "angular_velocity_impulse_rad_s": "vector",
    "fault": "fault",
    "bounce": "boolean",
    "level_hold": "boolean",
    "anchor": "boolean",
    "heading_hold": "boolean",
}
FAULT_MODES = ("none", "sensor", "deadline", "infeasible")


@dataclass(frozen=True)
class ScheduledChange:
    """A requested input and the candidate command tick, if one remains."""

    time_s: float
    field: str
    value: float | bool | str | tuple[float, float, float]
    tick: int | None


def _number(value: Any) -> float:
    if type(value) not in (int, float):
        raise ValueError("cần giá trị số hữu hạn")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError("cần giá trị số hữu hạn") from exc
    if not isfinite(value):
        raise ValueError("cần giá trị số hữu hạn")
    return value


def _value(kind: str, value: Any):
    if kind == "vector":
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError("cần ba thành phần x, y, z trong hệ thế giới")
        return tuple(map(_number, value))
    if kind == "fault":
        if type(value) is not str or value not in FAULT_MODES:
            raise ValueError("loại lỗi giả lập không hợp lệ")
        return value
    if kind == "boolean":
        if type(value) is not bool:
            raise ValueError("cần giá trị bật hoặc tắt")
        return value
    value = _number(value)
    if kind == "nonnegative" and value < 0:
        raise ValueError("độ bám phải không âm")
    return value


def compile_schedule(
    entries: list, times: Sequence[float]
) -> tuple[ScheduledChange, ...]:
    """Bind edits to the first command sample at or after their requested time.

    Equal-tick changes to different fields are independent. A field may only
    change once at a tick. Rows beyond the last command sample remain in the
    record with no candidate tick, even at the terminal integration time.
    """
    if not isinstance(entries, list):
        raise ValueError("Lịch NB6 phải là danh sách các mốc.")
    times = np.asarray(times)
    changes = []
    occupied = {}
    for row, entry in enumerate(entries, start=1):
        try:
            if not isinstance(entry, dict) or entry.keys() != {
                "time_s",
                "field",
                "value",
            }:
                raise ValueError("cần thời điểm, tác động và giá trị")
            time = _number(entry["time_s"])
            if time < 0:
                raise ValueError("thời điểm phải không âm")
            field = entry["field"]
            if not isinstance(field, str) or field not in SCHEDULE_FIELDS:
                raise ValueError("tác động không được hỗ trợ")
            value = _value(SCHEDULE_FIELDS[field], entry["value"])
            tick = int(np.searchsorted(times, time))
            if tick < len(times):
                key = (tick, field)
                if key in occupied:
                    raise ValueError(
                        f"trùng tác động với dòng {occupied[key]} tại cùng mẫu lệnh"
                    )
                occupied[key] = row
            else:
                tick = None
        except ValueError as exc:
            raise ValueError(f"Lịch NB6 · dòng {row} {exc}.") from exc
        changes.append(ScheduledChange(time, field, value, tick))
    return tuple(changes)
