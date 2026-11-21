"""Vietnamese display text. Persisted identifiers and numbers stay in the run schema."""

from typing import Any

from .records import RunRecord, decimal
from .session import CONTROLLER_LABELS

CASE_LABELS = {
    "left_grip": "Giảm độ bám bánh trái",
    "nominal": "Danh định",
    "split_level": "Hai mặt đỡ lệch",
    "late": "Bộ giải trễ",
    "brief_late": "Bộ giải trễ một mẫu",
    "infeasible": "Lệnh không khả thi",
    "sensor_lost": "Mất cảm biến",
    "low_grip": "Giảm độ bám hai bánh",
    "competing": "Giới hạn mô-men",
    "drive_release": "Chạy rồi ngắt tốc độ",
    "one_wheel": "Nhấc một bánh",
    "brief_flight": "Bay ngắn",
    "extended_flight": "Bay kéo dài",
    "rolling": "Lăn cứng",
    "compliant": "Tiếp xúc đàn hồi",
    "slip": "Trượt",
    "flight": "Rơi và chạm đất",
    "rolling_limit": "Mất điều kiện lăn",
    "dry": "Nền khô",
    "low_grip_limited": "Ít bám, có giới hạn",
    "slow": "Nâng chậm",
    "fast": "Nâng nhanh",
    "stationary": "Giữ độ cao",
}
FIELD_LABELS = {
    "controller": "Bộ điều khiển",
    "period_s": "Chu kỳ mẫu (s)",
    "duration_s": "Thời lượng (s)",
    "substeps": "Bước tích phân mỗi mẫu",
    "initial_q_deg": "Góc hông ban đầu (°)",
    "initial_lean_deg": "Độ nghiêng ban đầu (°)",
    "friction": "Độ bám ban đầu",
    "friction_after": "Độ bám sau thay đổi",
    "friction_change_s": "Thời điểm đổi độ bám (s)",
    "support_levels_m": "Cao độ mặt đỡ (m)",
    "sensor_seed": "Giá trị khởi tạo bộ sinh nhiễu",
    "noise_seed": "Giá trị khởi tạo bộ sinh nhiễu",
    "fault": "Lỗi giả lập",
    "fault_start_s": "Bắt đầu lỗi (s)",
    "fault_end_s": "Kết thúc lỗi (s)",
    "feedback": "Nguồn phản hồi",
    "priority": "Thứ tự nhiệm vụ",
    "mode": "Chế độ",
    "inverse_side": "Chân giải nghịch",
    "inverse_target_mm": "Tâm bánh đích x/z (mm)",
    "force_left_N": "Tải ngoài bánh trái x/z (N)",
    "force_right_N": "Tải ngoài bánh phải x/z (N)",
    "joint_rate_rad_s": "Vận tốc góc hông trái / phải (rad/s)",
    "joint_accel_rad_s2": "Gia tốc góc hông trái / phải (rad/s²)",
    "left_q_deg": "Góc trái (°)",
    "right_q_deg": "Góc phải (°)",
    "left_final_deg": "Góc cuối trái (°)",
    "right_final_deg": "Góc cuối phải (°)",
    "height_offset_m": "Độ cao đặt so với gốc (m)",
    "drive_speed_m_s": "Vận tốc đặt (m/s)",
    "turn_rate_rad_s": "Tốc độ quay đặt (rad/s)",
    "level_hold": "Cân bằng ngang",
    "heading_hold": "Giữ hướng",
    "anchor": "Về điểm neo",
    "delay_samples": "Trễ lệnh (mẫu)",
    "trajectory": "Quỹ đạo",
    "max_step_s": "Bước tích phân tối đa (s)",
    "initial_axle_m_s": "Vận tốc trục ban đầu (m/s)",
    "drop_height_m": "Độ cao thả ban đầu (m)",
    "total_wheel_torque_N_m": "Mô-men tổng hai bánh (N·m)",
    "wheel_torque_ramp_N_m_s": "Tốc độ tăng mô-men tổng (N·m/s)",
    "accel_noise_m_s2": "Độ lệch chuẩn nhiễu gia tốc (m/s²)",
    "encoder_noise_rad_s": "Độ lệch chuẩn nhiễu encoder (rad/s)",
    "accel_noise": "Độ lệch chuẩn nhiễu gia tốc (m/s²)",
    "pitch_noise": "Độ lệch chuẩn nhiễu góc nghiêng (rad)",
    "traction_budget": "Độ bám giả định để giới hạn lệnh",
    "lift_s": "Thời gian nâng (s)",
    "target_q_deg": "Góc hông đích (°)",
    "total_wheel_limit_N_m": "Giới hạn mô-men tổng hai bánh (N·m)",
    "mpc_horizon": "Chân trời MPC (mẫu)",
    "feature_start_s": "Bắt đầu nhiệm vụ phụ (s)",
    "bounce_off_s": "Kết thúc dao động độ cao (s)",
    "bounce_amplitude_m": "Biên độ dao động độ cao (m)",
    "bounce_hz": "Tần số dao động độ cao (Hz)",
    "turn_end_s": "Kết thúc lệnh quay (s)",
    "drive_end_s": "Kết thúc lệnh vận tốc (s)",
    "reference_capacity": "Hệ số giới hạn tham chiếu",
    "hip_capacity": "Hệ số giới hạn mô-men hông",
    "impulse_at_s": "Thời điểm nhiễu động (s)",
    "velocity_impulse_m_s": "Bước đổi vận tốc x/y/z (m/s)",
    "angular_velocity_impulse_rad_s": "Bước đổi vận tốc góc x/y/z (rad/s)",
}
OPTIONS = {
    "controller": CONTROLLER_LABELS,
    "priority": {"balance_first": "Thăng bằng trước", "legs_first": "Chân trước"},
    "feedback": {
        "truth": "Trạng thái mô hình",
        "odometry": "Vận tốc suy từ bánh xe",
        "gated": "Ước lượng phát hiện trượt",
    },
    "fault": {
        "none": "Không lỗi",
        "sensor": "Mất cảm biến",
        "deadline": "Bộ giải trễ",
        "infeasible": "Bài toán vô nghiệm",
    },
    "inverse_side": {"left": "Trái", "right": "Phải"},
    "mode": {
        "pose": "Tư thế và đạo hàm",
        "trajectory": "Quỹ đạo đặt",
        "load_sweep": "Quét góc và tải hông tĩnh",
        "inverse": "Động học nghịch",
        "rolling": "Lăn cứng",
        "compliant": "Đàn hồi",
        "flight": "Bay",
    },
}
SIGNAL_LABELS = {
    "height": "Độ cao thân",
    "lean": "Góc nghiêng so với phương thẳng đứng",
    "pitch": "Góc nghiêng dọc",
    "roll": "Góc nghiêng ngang",
    "yaw": "Hướng quay",
    "position": "Vị trí thân",
    "center_of_mass": "Tâm khối lượng",
    "model_speed": "Vận tốc mô hình",
    "estimated_speed": "Vận tốc ước lượng",
    "odometry_speed": "Vận tốc suy từ bánh xe",
    "accelerometer": "Gia tốc cảm biến",
    "speed_estimation_error": "Sai số vận tốc ước lượng",
    "speed_tracking_error": "Sai số bám vận tốc",
    "yaw_rate_tracking_error": "Sai số bám tốc độ quay",
    "height_tracking_error": "Sai số bám độ cao",
    "yaw_rate": "Tốc độ quay",
    "torque_interval": "Khoảng mô-men khả thi",
    "force_constraints": "Phần dư ràng buộc lực thực tế",
    "normal_force": "Lực pháp tuyến",
    "tangential_force": "Lực tiếp tuyến",
    "slip_speed": "Tốc độ trượt",
    "gap": "Khe hở",
    "friction": "Độ bám mô hình",
    "friction_belief": "Độ bám ước lượng",
    "contact_mode": "Tiếp xúc",
    "wheel_center": "Tâm bánh",
    "hip_angle": "Góc hông",
    "hip_applied": "Mô-men hông áp dụng",
    "hip_static_effort": "Mô-men hông tĩnh",
    "hip_limit": "Mô-men hông cho phép",
    "envelope_rate": "Tốc độ khảo sát đường bao",
    "hip_saturation": "Bão hòa mô-men hông",
    "hip_requested": "Mô-men hông yêu cầu",
    "hip_preceding_effort": "Mô-men hông giữ ở bước trước",
    "wheel_applied": "Mô-men bánh áp dụng",
    "wheel_requested": "Mô-men bánh yêu cầu",
    "wheel_limit": "Mô-men bánh cho phép",
    "wheel_preceding_effort": "Mô-men bánh giữ ở bước trước",
    "command_source": "Nguồn lệnh",
    "supervisor": "Giám sát",
    "orientation_xyzw": "Tư thế thân (quaternion x/y/z/w)",
    "state_joints": "Trạng thái góc khớp",
    "state_velocity": "Vận tốc tổng quát",
    "state_7dof": "Tọa độ tổng quát (7 bậc tự do)",
    "rate_7dof": "Vận tốc tổng quát (7 bậc tự do)",
    "acceleration_7dof": "Gia tốc tổng quát (7 bậc tự do)",
    "active_constraints": "Ràng buộc đang chạm biên",
    "allocated_force": "Lực do bộ điều khiển phân bổ",
    "capacity_reason": "Lý do giới hạn tham chiếu",
    "carrier_error": "Sai số kích thước cụm BCO",
    "closure_error": "Sai số khép vòng",
    "contact_point": "Điểm tiếp xúc",
    "double_area_error": "Sai số gấp đôi diện tích tam giác BCO",
    "requested_height": "Độ cao yêu cầu",
    "requested_speed": "Vận tốc yêu cầu",
    "requested_yaw_rate": "Tốc độ quay yêu cầu",
    "effective_height": "Độ cao tham chiếu có hiệu lực",
    "effective_speed": "Vận tốc tham chiếu có hiệu lực",
    "effective_yaw_rate": "Tốc độ quay tham chiếu có hiệu lực",
    "feature_amount": "Mức tham gia của nhiệm vụ phụ",
    "feature_intent": "Yêu cầu kích hoạt nhiệm vụ phụ",
    "feature_status": "Trạng thái nhiệm vụ phụ",
    "hip_rate": "Vận tốc góc hông",
    "hip_acceleration": "Gia tốc góc hông",
    "hip_travel_margin": "Khoảng cách tới giới hạn hành trình hông",
    "hip_utilization": "Tỷ lệ sử dụng mô-men hông",
    "boundary_margin": "Khoảng cách tới biên hình học",
    "loop_sine": "Trị tuyệt đối sin góc giữa BC và DC",
    "sensor_valid": "Dữ liệu cảm biến hợp lệ",
    "slip_detected": "Phát hiện trượt",
    "solver_status": "Trạng thái bộ giải",
    "solver_residual": "Phần dư của bộ giải",
    "solver_seconds": "Thời gian chạy bộ giải",
    "spin": "Góc quay bánh so với cụm BCO",
    "spin_rate": "Tốc độ quay bánh so với cụm BCO",
    "spin_absolute": "Góc quay tuyệt đối của bánh",
    "spin_relative_rate": "Tốc độ quay bánh đo tương đối",
    "wheel_spin": "Góc quay cặp bánh",
    "wheel_spin_rate": "Tốc độ quay cặp bánh",
    "task_names": "Thứ tự nhiệm vụ",
    "task_scales": "Hệ số tỷ lệ của nhiệm vụ",
    "task_targets": "Mục tiêu của nhiệm vụ",
    "wheel_velocity": "Vận tốc tâm bánh",
    "wheel_acceleration": "Gia tốc tâm bánh",
    "wheel_center_body": "Tâm bánh trong hệ thân",
    "wheel_velocity_body": "Vận tốc tâm bánh trong hệ thân",
    "wheel_velocity_world": "Vận tốc tâm bánh trong hệ thế giới",
    "wheel_accel_body": "Gia tốc tâm bánh trong hệ thân",
    "wheel_accel_world": "Gia tốc tâm bánh trong hệ thế giới",
    "axle_external_load": "Tải ngoài tại trục bánh",
    "axle_position": "Vị trí trục bánh",
    "encoder_rate": "Tốc độ quay đo bởi encoder",
    "energy": "Cơ năng",
    "force_admissible": "Lực tiếp xúc khả thi",
    "sensed_acceleration": "Gia tốc đo được",
    "sensed_lean": "Góc nghiêng đo được",
    "lean_rate": "Tốc độ thay đổi góc nghiêng",
    "requested": "Mô-men tổng yêu cầu",
    "limited": "Mô-men tổng sau giới hạn",
    "applied": "Mô-men tổng áp dụng",
    "limit_reason": "Lý do giới hạn mô-men",
    "gain": "Ma trận hệ số phản hồi",
    "grip_usage": "Tỷ lệ sử dụng độ bám",
    "linear_a": "Ma trận trạng thái rời rạc A",
    "linear_b": "Ma trận đầu vào rời rạc B",
    "nominal_force_constraints": "Phần dư ràng buộc lực danh định",
    "nominal_state": "Trạng thái danh định",
    "tracking_error": "Sai số bám trạng thái",
}
STATE_LABELS = {
    "completed": "Hết thời lượng",
    "paused": "Tạm dừng tính",
    "cancelled": "Đã hủy giữa ca",
    "touchdown": "Chạm đất",
    "rolling_limit": "Mất điều kiện lăn",
    "hip_travel": "Vượt hành trình hông",
    "flight_or_fall": "Bay quá lâu hoặc ngã",
    "sensor": "Mất cảm biến",
    "no_valid_command": "Thiếu lệnh khả thi",
    "pitch_limit": "Chạm giới hạn nghiêng",
    "traction_infeasible": "Mô-men bất khả thi",
    "lost_contact": "Mất tiếp xúc",
    "rolling": "Lăn",
    "sliding": "Trượt",
    "flight": "Bay",
    "allocator": "Bộ phân bổ",
    "balance_fallback": "LQR dự phòng",
    "normal": "Bình thường",
    "stop": "Dừng",
    "deadline": "Bộ giải quá hạn",
    "infeasible": "Không khả thi",
    "solved": "Đã tìm được nghiệm",
    "skipped_sensor": "Bỏ qua do mất cảm biến",
    "skipped_safety": "Bỏ qua do chạm ngưỡng an toàn",
    "reduced": "Giảm chế độ điều khiển",
    "supervisor": "Giới hạn bởi bộ giám sát",
    "available": "Đủ điều kiện",
    "contact_or_slip": "Mất tiếp xúc hoặc trượt",
    "actuator_headroom": "Giới hạn dự trữ mô-men",
    "scenario_cap": "Giới hạn theo kịch bản",
    "Idle": "Chưa hoạt động",
    "Enter": "Đang kích hoạt",
    "Active": "Đang hoạt động",
    "Exit": "Đang kết thúc",
    "Limited": "Bị giới hạn",
    "Paused": "Đang tạm dừng",
    "motor": "Giới hạn động cơ",
    "traction": "Giới hạn độ bám",
    "within_limits": "Trong giới hạn",
    "limited": "Đã giới hạn mô-men",
    "midpoint_infeasible": "Không khả thi tại giữa bước",
    "endpoint_infeasible": "Không khả thi tại cuối bước",
    "mpc_fallback": "Chuyển sang LQR dự phòng",
    "lean": "Giữ thăng bằng dọc",
    "height": "Điều chỉnh độ cao",
    "level_or_symmetry": "Giữ ngang hoặc đối xứng hai chân",
    "yaw_rate": "Điều chỉnh tốc độ quay",
    "bounce": "Dao động độ cao",
    "level": "Giữ ngang",
    "anchor": "Về điểm neo",
    "heading": "Giữ hướng",
    "synthetic_research_only": "Dữ liệu nghiên cứu tổng hợp",
}

FRAME_LABELS = {
    "body": "Hệ thân",
    "body x/z": "Trục x/z trong hệ thân",
    "body → world": "Từ hệ thân sang hệ thế giới",
    "carrier": "Hệ cụm BCO",
    "heading": "Theo hướng tiến",
    "hip L/R, carrier-relative spin L/R": "Góc hông trái/phải, góc quay bánh trái/phải so với cụm BCO",
    "leg x/z": "Trục x/z trong hệ chân",
    "sagittal": "Mặt phẳng dọc",
    "total": "Tổng của hai bánh",
    "total +x": "Tổng của hai bánh theo chiều tiến",
    "total, world x": "Tổng của hai bánh theo trục x thế giới",
    "upper CoM about axle": "Tâm khối lượng phần trên so với trục bánh",
    "world": "Hệ thế giới",
    "world linear/angular, joints L/R": "Vận tốc tịnh tiến và góc trong hệ thế giới, vận tốc khớp trái/phải",
    "world x": "Trục x thế giới",
    "world x/y": "Mặt phẳng x/y thế giới",
    "world, XYZ": "Hệ thế giới, quy ước XYZ",
}

METADATA_LABELS = {
    "parameters": "Tham số mô hình",
    "planar": "Mô hình phẳng",
    "spatial": "Mô hình không gian",
    "hip_envelope": "Đặc tính mô-men hông",
    "settings": "Thiết lập bộ mô phỏng",
    "contact": "Mô hình tiếp xúc",
    "plant": "Đối tượng điều khiển",
    "simulation": "Thiết lập mô phỏng",
    "design_inputs": "Đầu vào thiết kế điều khiển",
    "linear_model": "Mô hình tuyến tính",
    "coordinate_order": "Thứ tự tọa độ",
    "velocity_order": "Thứ tự vận tốc",
    "feature_order": "Thứ tự nhiệm vụ phụ",
    "feedback_state_order": "Thứ tự trạng thái phản hồi",
    "state_order": "Thứ tự trạng thái",
    "constraint_order": "Thứ tự ràng buộc",
    "fallback_state_order": "Thứ tự trạng thái của bộ điều khiển dự phòng",
    "fallback_gain": "Ma trận phản hồi dự phòng",
    "fallback_a": "Ma trận trạng thái dự phòng A",
    "fallback_b": "Ma trận đầu vào dự phòng B",
    "assumptions": "Giả thiết",
    "contact_assumptions": "Giả thiết tiếp xúc",
    "inverse_solutions_deg": "Nghiệm động học nghịch (°)",
    "geometric_bounds_deg": "Biên hình học (°)",
    "study": "Bài thử",
    "case": "Kịch bản",
    "continuous_a": "Ma trận trạng thái liên tục A",
    "continuous_b": "Ma trận đầu vào liên tục B",
    "spectral_radius": "Bán kính phổ",
    "branch": "Nhánh nghiệm",
}

DESCRIPTION_LABELS = {
    "Thin-disk support geometry. Sagittal static load at O with separate speed-dependent actuator limit.": "Tiếp xúc theo hình học đĩa mỏng. Tính tải tĩnh tại O trong mặt phẳng dọc và xét giới hạn mô-men theo tốc độ khớp.",
    "Aggregate fixed-hip pendulum. Torques refer to both wheels together.": "Mô hình con lắc gộp với góc hông cố định. Giá trị mô-men là tổng của hai bánh.",
    "Prescribed symmetric hip trajectory and rigid rolling. Wheel command is total +x torque.": "Hai khớp hông có quỹ đạo đặt đối xứng, bánh lăn cứng. Lệnh là mô-men tổng của hai bánh theo chiều tiến.",
    "Thin disks on independent horizontal supports. Force is from the preceding plant substep (static normal preload at t=0).": "Hai bánh dạng đĩa mỏng tựa trên hai mặt ngang độc lập. Lực lấy từ bước tích phân trước. Tại thời điểm 0, lực ban đầu là tải pháp tuyến tĩnh.",
    "x axle [m]": "Vị trí trục bánh x [m]",
    "v axle [m/s]": "Vận tốc trục bánh [m/s]",
    "lean relative to initial pitch offset [rad]": "Góc nghiêng so với mốc nghiêng dọc ban đầu [rad]",
    "lean rate [rad/s]": "Tốc độ thay đổi góc nghiêng [rad/s]",
    "θ CCW [rad]": "θ ngược chiều kim đồng hồ [rad]",
    "φL CCW [rad]": "φ trái ngược chiều kim đồng hồ [rad]",
    "φR CCW [rad]": "φ phải ngược chiều kim đồng hồ [rad]",
    "spiṅL [rad/s]": "Tốc độ quay bánh trái [rad/s]",
    "spiṅR [rad/s]": "Tốc độ quay bánh phải [rad/s]",
}


SCHEDULE_LABELS = {
    "drive_speed_m_s": "Vận tốc tiến (m/s)",
    "turn_rate_rad_s": "Tốc độ quay (rad/s)",
    "height_offset_m": "Độ lệch cao độ đặt (m)",
    "friction_left": "Độ bám bánh trái",
    "friction_right": "Độ bám bánh phải",
    "velocity_impulse_m_s": "Đổi vận tốc x/y/z (m/s)",
    "angular_velocity_impulse_rad_s": "Đổi vận tốc góc x/y/z (rad/s)",
    "fault": "Lỗi giả lập",
    "bounce": "Tác vụ độ cao / nhún",
    "level_hold": "Cân bằng ngang",
    "anchor": "Neo vị trí",
    "heading_hold": "Giữ hướng",
}


def field_label(name: str) -> str:
    return FIELD_LABELS.get(name, name)


def case_label(study: str, case: str | None) -> str:
    if study == "NB4" and case == "low_grip":
        return "Nền ít bám"
    return CASE_LABELS.get(case, case or study)


def frame_label(name: str) -> str:
    return FRAME_LABELS.get(name, name)


def parameter_label(name: str) -> str:
    return METADATA_LABELS.get(name, field_label(name))


def configuration_value(study: str, name: str, value: Any) -> Any:
    if name == "case":
        return case_label(study, value)
    if isinstance(value, str):
        return OPTIONS.get(name, {}).get(value, value)
    return value


def unit_label(unit: str) -> str:
    return {
        "1": "",
        "mixed": "nhiều đơn vị",
        "mixed/s": "nhiều đơn vị",
        "mixed/s²": "nhiều đơn vị",
    }.get(unit, unit)


def label(name: str) -> str:
    side, sep, signal = name.partition("_")
    prefix = {"left": "Trái · ", "right": "Phải · "}.get(side) if sep else None
    channel = signal if prefix else name
    point, _, frame = channel.partition("_")
    if point in ("A", "B", "C", "D", "H", "O") and frame in ("", "world"):
        coordinates = "hệ thế giới" if frame else "hệ chân"
        return f"{prefix or ''}Điểm {point} trong {coordinates}"
    return (prefix or "") + SIGNAL_LABELS.get(channel, channel.replace("_", " "))


def shown(value: Any) -> str:
    if isinstance(value, str):
        if value in DESCRIPTION_LABELS:
            return DESCRIPTION_LABELS[value]
        value = value.replace("left · ", "Bánh trái · ").replace(
            "right · ", "Bánh phải · "
        )
        side, _, edge = value.partition("_friction_edge_")
        if edge and side in ("L", "R"):
            return f"Bánh {'trái' if side == 'L' else 'phải'} · Biên ma sát {edge}"
        return " · ".join(STATE_LABELS.get(part, part) for part in value.split(" · "))
    if isinstance(value, (tuple, list)):
        return "[ " + " / ".join(shown(item) for item in value) + " ]" if value else "—"
    return decimal(value)


def wheel_readout(run: RunRecord, index: int, side: str) -> tuple[str, str, str, str]:
    def value(channel: str) -> Any:
        signal = run.signals.get(f"{side}_{channel}")
        return signal.values[index] if signal else None

    def reading(amount: Any, unit: str, precision: int) -> str:
        return "—" if amount is None else f"{decimal(amount, precision)} {unit}"

    def effort(channel: str, title: str) -> str:
        amount = value(f"{channel}_applied")
        if amount is None:
            amount = value(f"{channel}_preceding_effort")
            if amount is not None:
                title += " ở bước trước"
            elif channel == "hip":
                amount = value("hip_static_effort")
                if amount is not None:
                    title += " tĩnh"
        return f"{title}  {reading(amount, 'N·m', 3)}"

    return (
        f"Tiếp xúc  {shown(value('contact_mode'))}",
        f"Lực pháp tuyến  {reading(value('normal_force'), 'N', 2)}",
        effort("wheel", "Mô-men bánh"),
        effort("hip", "Mô-men hông"),
    )
