# Robot DUK

Thư mục `robot/` sở hữu các tài sản thuộc **robot DUK cụ thể**:

- `hardware/`: nguồn CAD, thiết kế mạch, sơ đồ điện và bản phát hành chế tạo;
- `software/core/`: mô hình, thuật toán và ví dụ tính toán độc lập ROS;
- `software/ros2_ws/`: tích hợp ROS 2; các thư mục package đang là placeholder;
- `software/firmware/`: chương trình cho thiết bị nhúng khi được phát triển;
- `simulation/`: môi trường và kịch bản mô phỏng của DUK;
- `config/`: thông số danh định, controller và hiệu chuẩn.

CAD, URDF/Xacro và mô hình toán học là những biểu diễn khác nhau. Phải có nguồn sở hữu, đơn vị và cơ chế kiểm tra tính nhất quán; không sao chép thông số không kiểm soát giữa chúng.

Xem thêm [Kiến trúc repository](../docs/architecture/repository-layout.md).
