# Cấu trúc monorepo DUK

## Các miền sở hữu

| Đường dẫn | Trách nhiệm | Ranh giới |
| --- | --- | --- |
| `robot/hardware/mechanical/` | CAD gốc, bản vẽ, sản phẩm chế tạo | Không phải ROS description runtime |
| `robot/hardware/electronics/` | Board, thiết kế nguồn, sơ đồ dây | Không phải firmware |
| `robot/software/core/` | Python model, control, visualization, notebook | Không phụ thuộc ROS hoặc Docker |
| `robot/software/ros2_ws/` | ROS nodes, interfaces, hardware, bringup, Gazebo integration | Không duy trì bản sao thuật toán core |
| `robot/software/firmware/` | Firmware MCU và thiết bị nhúng | Không sở hữu schematic/PCB |
| `robot/simulation/` | World, scenario và tài nguyên mô phỏng | ROS bridge thuộc duk_gazebo |
| `robot/config/` | Tham số vật lý, controller, hiệu chuẩn | Không có nhiều nguồn giá trị cùng ý nghĩa |
| `research/` | Experiments, benchmark, publications | Không thay thế core notebook |
| `applications/` | Operator dashboard, gateway, mobile | Không điều khiển motor trực tiếp |
| `environment/` | Docker, Compose, platforms, dependencies | Không hardcode đường dẫn vào core |
| `tools/` | Các công cụ kỹ thuật tái sử dụng | Không phải script lẻ |
| `scripts/` | Các tiện ích phát sinh trong quá trình dev | Không phải CI/CD |
| `.github/` | GitHub Actions, issue/PR templates | Không giữ thuật toán cốt lõi |
| `tests/` | Kiểm thử integration, compatibility và system | Unit test nằm cạnh module |
| `data/` | Output local bị ignore, giữ khung bằng .gitkeep | Không phải nguồn thông số chính thức |

## Mối quan hệ giữa các phần

1. **CAD → tham số → URDF/core:** CAD, thông số đo đạc và hiệu chuẩn có nguồn riêng.
   Phải công bố những giá trị chuẩn và kiểm tra các biểu diễn. Không tự xem
   file STL hay URDF là nguồn thay thế toàn bộ CAD.
2. **Core → ROS:** Core cung cấp API tính toán. ROS package chuyển đổi ROS
   messages, nguồn trạng thái và lệnh sang hợp đồng của core.
3. **Core → notebook:** Notebook phục vụ tính thử, biểu đồ và chuyển động kiểu
   MATLAB. Hàm dùng lại nằm ở `src/duk_core/`; notebook không sở hữu công thức riêng.
4. **Core → research:** Experiment và benchmark dùng lại cùng mô hình, thuật toán
   và dữ liệu tham chiếu để kết quả có khả năng tái lập.
5. **Compose → runtime:** Compose tạo môi trường; không làm thay đổi cấu trúc
   import, API hoặc code của thuật toán theo máy host.

## Quy tắc thư mục

- Dùng `.gitkeep` duy nhất trong các thư mục rỗng được quy hoạch. Xóa file
  này khi đã có file thật trong thư mục đó.
- Những ROS package có mỗi `.gitkeep` **không được coi là package đã chạy**.
- Giữ build output, rosbag, video, log và dữ liệu lớn ngoài Git mặc định.
- Không thêm `analysis/`, `research/workbench/` hoặc `robotics-core/` thứ hai
  để thực hiện lại chức năng đã thuộc `core/`.

## Thử nghiệm LQR minh họa

Các module mô hình và bộ điều khiển được viết trong core. Notebook
`core/notebooks/control/` thử các ma trận và vẽ đáp ứng; experiment tại
`research/experiments/` lưu cấu hình và kết quả so sánh. ROS controller chỉ
tích hợp thuật toán đã được kiểm chứng. Hiện chưa có triển khai LQR của DUK.
