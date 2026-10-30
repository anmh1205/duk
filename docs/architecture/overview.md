# Kiến trúc DUK

DUK là monorepo của một robot wheel-legged biped. `robot/` chứa đối tượng
kỹ thuật cụ thể; `research/` chứa các phép đánh giá có phương pháp;
`applications/` chứa các ứng dụng bên ngoài runtime robot;
`environment/` đóng gói môi trường phát triển và triển khai.

Mô hình và bộ điều khiển trong `robot/software/core/` là Python API độc lập ROS.
Notebook, chương trình ví dụ và unit test cùng gọi API này. Các ROS package
trong `robot/software/ros2_ws/` tích hợp chức năng với cảm biến, hardware và
Gazebo; không sao chép mô hình hoặc thuật toán sang ROS node.

Mọi đường đi từ thiết kế CAD, tập tham số, URDF/Xacro đến thuật toán và simulator
phải rõ nguồn dữ liệu, đơn vị, hệ tọa độ và cách kiểm tra tính nhất quán.

Xem [cấu trúc repository](repository-layout.md) và
[quy tắc phát triển](../development/engineering-rules.md).
