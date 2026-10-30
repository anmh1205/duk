# Môi trường ROS 2 và Python

## Phương thức phát triển chính

Docker Compose là nền tảng đóng gói, Dev Containers là cách để IDE mở đúng
môi trường. Mọi lệnh `ros2`, `colcon` và Python được chạy bình thường
trong terminal của container, không cần wrapper riêng cho từng lệnh.

1. Cài Docker Engine, Compose và IDE có hỗ trợ Dev Containers.
2. `cp .env.example .env` (hoặc tạo file tương đương trên hệ điều hành khác).
3. Mở thư mục DUK và chọn **Reopen in Container**.
4. Kiểm tra `ros2 --help`, `colcon --help`, rồi chạy:
   `python -m pytest robot/software/core/tests`.
5. Core Python được cài editable qua `postCreateCommand` của Dev Container.

Khi muốn đổi bản ROS, sửa `.env` rồi rebuild container. Việc này **không**
tự bảo đảm mọi dependency/Gazebo/hardware phù hợp với bản mới; xem
[ma trận nền tảng](../platforms/compatibility.md).

## Docker Compose trực tiếp

```bash
docker compose up -d dev
docker compose exec dev bash
# Trong container, cài core nếu chưa mở từ Dev Containers:
python -m pip install -e "./robot/software/core[test,research]"
```

Compose quản lý môi trường, không đóng gói thiết bị MCU thành container.
Không bật quyền truy cập USB/CAN/GPU hay real-time privileges mặc định.
Các cấu hình Jetson và mô phỏng đồ họa sẽ được bổ sung sau khi có kiểm thử
trên phần cứng cụ thể.

## Giới hạn hiện tại

- Có Dockerfile ROS dev cơ bản và khai báo Dev Container, chưa có bằng chứng
  Docker image chạy thành công trên máy phát triển hoặc board của DUK.
- Chưa có ROS package thực tế để chạy `ros2 launch duk_bringup`.
- Chưa có GUI Gazebo/RViz hoặc Jetson CUDA/runtime được cấu hình.
- Docker host Linux/macOS/Windows và ARM64 có thể khác nhau về network,
  filesystem và quyền thiết bị; phải thử riêng.
