# duk-core — Python modeling and control

Core là Python package dùng cho mô hình toán học, tính toán điều khiển và biểu diễn dữ liệu DUK. Các phép tính trong `modeling/` và `control/` không bắt buộc ROS hoặc các thư viện đồ họa.

## Cài đặt

```bash
python -m pip install -e "./robot/software/core[test,research]"
python -m pytest robot/software/core/tests
python robot/software/core/examples/planar_chain.py
```

`research` cài các công cụ phục vụ notebook và hiển thị; runtime không phải cài những thư viện này khi không cần.

## Ví dụ ban đầu

`duk_core.modeling.planar.forward_chain` tính tọa độ các điểm của **chuỗi khớp quay phẳng tổng quát**, với góc tương đối và chiều dài từng đoạn. `duk_core.visualization.planar.draw_chain` biểu diễn các điểm đó bằng Matplotlib. Đây chỉ là ví dụ API và thử nghiệm hình học, **không phải mô hình động học của DUK**; các công thức DUK thực tế phải được xây dựng từ thiết kế cơ khí và mô hình đã xác minh.

Notebook tại `notebooks/kinematics/planar-chain.ipynb` minh họa luồng tính toán → giá trị → hình vẽ. Các module `control/` và nhiều nhóm notebook khác hiện mới là khung mở rộng.

## Quy tắc

- Mọi phép tính được dùng lại phải nằm trong `src/duk_core/`, không chỉ trong notebook.
- Không hardcode đường dẫn tới `robot/config/` trong thư viện; cấu hình được cấp qua API hoặc adapter.
- Dữ liệu và hình được tạo từ notebook lưu dưới `data/` (bị ignore) khi cần, không commit output lớn.
- Bổ sung test đối với chức năng toán học trước khi coi nó là triển khai chính thức.
