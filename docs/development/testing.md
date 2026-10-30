# Kiểm thử

## Python core

```bash
python -m pip install -e "./robot/software/core[test]"
python -m pytest -q robot/software/core/tests
python robot/software/core/examples/planar_chain.py
```

Các bài kiểm thử hiện tại xác minh **hàm hình học phẳng tổng quát** và một
ví dụ dùng API, không chứng minh tính đúng đắn mô hình DUK thực.

## CI

Workflow `.github/workflows/python-core.yml` cài package Python,
chạy unit test, ví dụ và kiểm tra compile. CI chưa build ROS, mô phỏng
Gazebo hoặc thử nghiệm trên phần cứng; không tuyên bố những khả năng đó
đã được xác minh.

## Khi có phần mềm ROS và robot thực

- Unit test đặt trong package.
- `tests/integration/` xác nhận giao tiếp và hợp đồng dữ liệu.
- `tests/compatibility/` xác nhận môi trường ROS và nền tảng.
- `tests/system/` kiểm thử điều kiện vận hành của toàn hệ thống.
- Test tác động phần cứng phải có quy trình cho phép, giới hạn an toàn và
  khả năng dừng khẩn cấp phù hợp.
