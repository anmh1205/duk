# Trạng thái hỗ trợ nền tảng

Danh sách khai báo ở
[compatibility.yaml](../../environment/platforms/compatibility.yaml).

- `unverified`: đã quy hoạch, chưa kiểm thử trên mục tiêu.
- `unconfigured`: chưa chọn ROS/JetPack hoặc chưa có image runtime thích hợp.
- Chỉ ghi `verified` sau khi có kết quả build/run trên đúng tổ hợp ROS,
  Ubuntu image, CPU architecture và các driver cần thiết.

Image ROS có thể hỗ trợ amd64/arm64 nhưng không tự bảo đảm CUDA/JetPack,
Gazebo GUI, USB/CAN hoặc time-critical scheduling hoạt động trên Jetson.
Không dùng lại cache nhị phân giữa các ROS distro hoặc kiến trúc khác nhau.
