# DUK Simulation Lab

Dashboard mô phỏng cục bộ dùng trực tiếp `duk_core`. Từ thư mục gốc của repo,
chạy các lệnh dưới đây trong môi trường Python 3.10 trở lên.

```bash
python -m pip install -e './robot/software/core[research]'
OPENBLAS_NUM_THREADS=1 python applications/operator-dashboard/server.py
```

Mở [dashboard cục bộ](http://127.0.0.1:8765/). Có thể đổi cổng bằng `--port`.
Ứng dụng chỉ lắng nghe trên `127.0.0.1`.

## Chạy và so sánh

- **Cân bằng vòng kín** dùng mô hình con lắc bánh xe NB4 với góc khớp hông cố
  định. Chọn điều kiện độ bám, LQR giữ vị trí hoặc giữ thăng bằng, nguồn phản hồi
  vận tốc và giới hạn mô-men theo độ bám. Phần **Nhiễu và độ trễ** cho phép chỉnh
  độ lệch chuẩn nhiễu, seed và số mẫu trễ.
- Chọn **Chạy mô phỏng** để tính toàn bộ ca. Thanh thời gian và tốc độ phát điều
  khiển việc phát lại. Tín hiệu đọc tại mẫu đã tính gần nhất ở hoặc trước con
  trỏ. Mô-men được giữ theo từng khoảng lấy mẫu. Biểu đồ kết thúc ở thời điểm
  dừng thực tế của mỗi ca.
- Chọn **Lưu để so sánh** để giữ ca hiện tại trên biểu đồ, rồi chạy cấu hình
  khác. **Tải kết quả JSON** xuất cấu hình, seed, các tham số mô hình, dấu băm
  dữ liệu đầu vào, ma trận tuyến tính và toàn bộ tín hiệu của ca hiện tại.
- **Động học hai chân** hiển thị hai hình chiếu X–Z trong hệ thân, tâm bánh,
  Jacobian và các sai số hình học. Miền nhập theo phần dự phòng số học của
  bảng khảo sát động học.

Ca mặc định dùng LQR giữ vị trí và vận tốc suy từ bánh xe. Với cùng cấu hình,
`μ = 0,8` chạy hết 2,5 s, `μ = 0,3` mất tiếp xúc, và `μ = 0,3` với giới hạn
mô-men theo `μ` giả định `0,25` chạy hết 2,5 s. Các kết quả này thuộc mô hình
tổng hợp với tham số nghiên cứu. Hình vẽ con lắc biểu diễn trạng thái tổng hợp.
Hình chiếu động học dùng hình học chuẩn từ cấu hình chân.

[Kịch bản độ bám](../../robot/simulation/scenarios/fixed-hip-balance.json) ·
[Notebook NB4](../../robot/software/core/notebooks/control/duk-fixed-hip-balance.ipynb) ·
[Dữ liệu nghiên cứu](../../research/experiments/duk-v3/README.md).
