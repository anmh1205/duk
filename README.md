# DUK — Wheel-Legged Biped Robot

DUK là monorepo cho thiết kế, nghiên cứu và phát triển robot hai chân có bánh xe (wheel-legged biped robot). Repository quản lý đồng bộ phần cứng, phần mềm, mô hình toán học, mô phỏng, thí nghiệm và các ứng dụng liên quan.

**Trạng thái:** khung kiến trúc đã khởi tạo. Các ROS package, firmware, dashboard và cấu hình Jetson chưa có mã chạy thực tế. Ví dụ động học phẳng trong core chỉ minh họa API và quy trình thử nghiệm; **không phải mô hình đã kiểm chứng của DUK**.

## Bắt đầu

### Python core (không cần ROS)

Yêu cầu Python 3.10 trở lên:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e "./robot/software/core[test,research]"
python -m pytest robot/software/core/tests
python robot/software/core/examples/planar_chain.py
```

Mở notebook tại `robot/software/core/notebooks/kinematics/` bằng JupyterLab hoặc IDE hỗ trợ Jupyter. Notebook sử dụng cùng API `duk_core` với kiểm thử và ví dụ.

### ROS 2 trong Docker

Yêu cầu Docker Engine, Docker Compose và IDE có hỗ trợ Dev Containers.

1. Sao chép `.env.example` thành `.env`, chọn bản ROS theo cấu hình hỗ trợ.
2. Mở repository trong IDE, chọn **Reopen in Container**.
3. Terminal bên trong container sử dụng lệnh `ros2`, `colcon` và Python bình thường.

Có thể dùng `docker compose up -d dev` để chạy service phát triển mà không qua IDE, sau đó truy cập bằng `docker compose exec dev bash`. Đây là cách vận hành thay thế; quy trình chính vẫn là mở Dev Container. Các ROS package đang là placeholder nên chưa có launch file của DUK.

Hướng dẫn và giới hạn thực tế: [Môi trường phát triển](docs/development/environment.md), [Nền tảng được quy hoạch](docs/platforms/compatibility.md).

## Cấu trúc

| Thư mục | Trách nhiệm |
| --- | --- |
| `robot/hardware/` | CAD gốc, bản vẽ cơ khí, PCB, hệ thống điện và bản chế tạo |
| `robot/software/core/` | Thư viện toán học Python, công cụ biểu diễn, notebook và unit test |
| `robot/software/ros2_ws/` | Tích hợp ROS 2; các package chưa triển khai |
| `robot/software/firmware/` | Firmware thiết bị nhúng |
| `robot/simulation/` | World và kịch bản mô phỏng DUK |
| `robot/config/` | Cấu hình vật lý, bộ điều khiển và hiệu chuẩn |
| `research/` | Thí nghiệm, benchmark và công bố khoa học |
| `applications/` | Dashboard vận hành, gateway và mobile |
| `environment/` | Docker, Compose, ma trận nền tảng và dependency |
| `tools/` | Công cụ kỹ thuật có thể tái sử dụng |
| `scripts/` | Script lẻ phục vụ quá trình dev, không sở hữu CI/CD |
| `docs/` | Tài liệu chính thức |
| `tests/` | Kiểm thử tích hợp, tương thích và hệ thống |
| `data/` | Dữ liệu phát sinh local, không commit mặc định |

## Tài liệu và đóng góp

- [Mục lục tài liệu](docs/README.md)
- [Cấu trúc và ranh giới repository](docs/architecture/repository-layout.md)
- [Quy tắc phát triển](docs/development/engineering-rules.md)
- [Hướng dẫn kiểm thử](docs/development/testing.md)
- [Hướng dẫn agent](AGENTS.md)

Giấy phép: [MIT](LICENSE).
