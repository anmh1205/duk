"""Common scenario dispatch, mathematical inspection and reproducibility checking."""

from __future__ import annotations

from math import isclose
from typing import Any

from duk_core.modeling.leg_linkage import LinkageError

from .inputs import Inputs
from .kinematics import (
    POINTS,
    SEGMENTS,
    kinematic_defaults,
    kinematic_record,
    linkage_equations,
)
from .records import REPLAY_TOLERANCE, RunRecord, plain
from .sagittal import HEIGHT_POLICIES, PLANAR_PRESETS, SagittalStudies, planar_equations
from .spatial import SpatialRun, spatial_equations, spatial_record

STUDIES = {
    "NB6": "NB6 · Toàn thân không gian",
    "NB1–2": "NB1–2 · Động học và tải hông",
    "NB3": "NB3 · Tiếp xúc phẳng 7 bậc tự do",
    "NB4": "NB4 · Cân bằng hông cố định",
    "NB5": "NB5 · Điều khiển theo độ cao",
}
CONTROLLERS = {
    "NB6": ("weighted", "hierarchical"),
    "NB4": ("position", "balance"),
    "NB5": HEIGHT_POLICIES,
}
CONTROLLER_LABELS = {
    "weighted": "QP có trọng số",
    "hierarchical": "HQP theo thứ tự",
    "position": "LQR giữ vị trí",
    "balance": "LQR giữ thăng bằng",
    "fixed": "LQR cố định",
    "scheduled": "LQR theo góc hông",
    "tv_lqr": "LQR biến thiên theo thời gian",
    "mpc": "MPC có ràng buộc",
}


class Workbench:
    def __init__(self, inputs: Inputs) -> None:
        self.inputs = inputs
        self.sagittal = SagittalStudies(inputs)

    def cases(self, study: str) -> tuple[str, ...]:
        if study == "NB6":
            return ("left_grip", *self.inputs.fixture["whole_body"]["cases"])
        if study == "NB3":
            return PLANAR_PRESETS
        if study == "NB4":
            return tuple(self.sagittal.balance.presets)
        if study == "NB5":
            return ("slow", "fast", "stationary")
        return ()

    def defaults(self, study: str, case: str | None = None) -> dict:
        if study == "NB6":
            return self.inputs.whole_body_defaults(case or "nominal")
        if study == "NB1–2":
            return kinematic_defaults(self.inputs)
        return self.sagittal.defaults(study, case)

    def run(self, options: dict) -> RunRecord:
        study = options.get("study")
        if not isinstance(study, str) or study not in STUDIES:
            raise ValueError("Choose one of the workbench studies NB1–NB6")
        if study == "NB6":
            return spatial_record(self.inputs, options)
        if study == "NB1–2":
            return kinematic_record(self.inputs, options)
        return self.sagittal.run(study, options)

    def begin_nb6(self, options: dict) -> SpatialRun:
        """Prepare NB6 without advancing its simulation clock."""
        if options.get("study") != "NB6":
            raise ValueError("Chế độ chạy từng bước hiện hỗ trợ NB6.")
        return SpatialRun(self.inputs, options)

    def matches_profile(self, run: RunRecord) -> bool:
        return all(
            run.provenance.get(key) == self.inputs.parameter_info[key]
            for key in ("profile_sha256", "geometry_sha256")
        )

    def verify_replay(self, original: RunRecord) -> RunRecord:
        if original.stop_reason in ("paused", "cancelled"):
            raise ValueError(
                "Bản ghi dừng giữa ca chỉ dùng để phát lại. "
                "Chọn kịch bản để chạy đủ thời lượng."
            )
        if not self.matches_profile(original):
            raise ValueError("Select the original geometry and parameter profile first")
        if original.provenance.get("code") != self.inputs.provenance()["code"]:
            raise ValueError(
                "Source files differ from the saved run. Use its recorded revision."
            )
        repeated = self.run(original.configuration)
        check_replay(original, repeated)
        return repeated

    def equations(self, run: RunRecord, index: int, scope: str) -> dict:
        if not self.matches_profile(run):
            return {
                "Nguồn": "Mở lại với bộ tham số gốc để tính các ma trận tại mẫu này."
            }
        side, _, component = scope.partition(".")
        if side in ("left", "right"):
            if run.study == "NB4":
                return {
                    "Mô hình": "NB4 gộp hai bánh và giữ cố định hông. Chọn Cặp bánh để xem mô hình tuyến tính."
                }
            q = run.signals.get(side + "_hip_angle")
            if q and (component not in ("hip", "wheel") or run.study == "NB1–2"):
                part = component.removeprefix("linkage.")
                if component in ("hip", "wheel"):
                    part = "O"
                try:
                    data = linkage_equations(
                        self.inputs,
                        q.values[index],
                        part if part in (*POINTS, *SEGMENTS) else None,
                    )
                except LinkageError:
                    return {"Trạng thái": "Cơ cấu vượt miền giải tại mẫu dừng."}
                if run.study == "NB1–2" and component == "hip":
                    data.update(
                        {
                            "Tải tĩnh": "τ = ∂V/∂q − J_Oᵀ F, với J_O tính bằng m/rad",
                            "Đường bao": "τ_max = η τ_stall max(0, 1 − |q̇|/ω₀)",
                            "Tải tại O [N]": run.signals[
                                side + "_axle_external_load"
                            ].values[index],
                            "Mô-men tĩnh [N·m]": run.signals[
                                side + "_hip_static_effort"
                            ].values[index],
                            "Mô-men cho phép [N·m]": run.signals[
                                side + "_hip_limit"
                            ].values[index],
                        }
                    )
                return plain(data)
        if run.study == "NB6":
            return spatial_equations(self.inputs, run, index, scope)
        if run.study in ("NB3", "NB5"):
            return planar_equations(self.inputs, run, index, scope)
        if run.study == "NB4":
            return run.metadata["linear_model"]
        return {
            key: run.metadata[key]
            for key in ("assumptions", "geometric_bounds_deg", "inverse_solutions_deg")
        }


def _same(a: Any, b: Any) -> bool:
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(
            _same(value, b[key]) for key, value in a.items()
        )
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if type(a) in (int, float) and type(b) in (int, float):
        return isclose(
            a, b, abs_tol=REPLAY_TOLERANCE["atol"], rel_tol=REPLAY_TOLERANCE["rtol"]
        )
    return type(a) is type(b) and a == b


def check_replay(original: RunRecord, repeated: RunRecord) -> None:
    if original.study != repeated.study or original.stop_reason != repeated.stop_reason:
        raise ValueError("Replay changed the study or its stopping reason")
    if not _same(original.time_s, repeated.time_s):
        raise ValueError("Replay changed the time or sample count")
    if not _same(original.events, repeated.events):
        raise ValueError("Replay changed an event")
    if not _same(original.metadata, repeated.metadata):
        raise ValueError("Replay changed the study parameters or model diagnostics")
    if original.signals.keys() != repeated.signals.keys():
        raise ValueError("Replay changed the recorded signals")
    for name, trace in original.signals.items():
        other = repeated.signals[name]
        if not trace.same_quantity(other):
            raise ValueError(f"Replay changed the signal metadata for {name}")
        if name != "solver_seconds" and not _same(trace.values, other.values):
            raise ValueError(f"Replay differs beyond tolerance for {name}")
