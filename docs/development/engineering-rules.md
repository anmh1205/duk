# Quy tắc phát triển DUK

## Kiến trúc và dữ liệu

- Một chức năng có một nơi sở hữu; không sao chép thuật toán giữa notebook,
  core, ROS và research.
- Xác định đơn vị, hệ tọa độ, thứ tự trạng thái và nguồn gốc mọi thông số.
- Không tự đặt kích thước, mômen quán tính hay thông số hiệu chuẩn của DUK.
- Giữ thư viện modeling/control độc lập ROS và GUI; import đồ họa khi cần.
- Khi thêm ROS package, tạo manifest, cấu hình build và test có thực; không
  để tên package tồn tại với một file code không build được.

## Chất lượng

- Unit test đặt gần mã nguồn; kiểm thử xuyên module đặt ở `tests/`.
- Sửa hành vi/API phải cập nhật tài liệu và ví dụ liên quan.
- Ghi chính xác lệnh và kết quả đã chạy; không tuyên bố tương thích chỉ vì
  cấu hình YAML tồn tại.
- Bảo đảm nội dung AGENTS.md và các tài liệu không đưa ra chỉ dẫn trái nhau.
- Chỉ phát hành CAD/PCB/manufacturing output theo phiên bản được xác nhận;
  không commit các bản xuất tạm hoặc dữ liệu lớn vô điều kiện.

## Trách nhiệm công cụ

- `.github/workflows/`: CI/CD chính thức và policy của GitHub.
- `tools/`: tool kỹ thuật có đầu vào/đầu ra rõ ràng, test khi cần.
- `scripts/`: script lẻ phục vụ thao tác phát triển, không là dependency CI.

## An toàn

- Không gửi lệnh chạy motor, nạp firmware hoặc triển khai lên thiết bị thật
  khi chưa có phê duyệt rõ ràng.
- Kết quả notebook, simulator và kiểm thử phần mềm không đồng nghĩa đã bảo
  đảm an toàn robot thật.
