# scripts

Nơi chứa **script lẻ** để thao tác nhanh, kiểm tra tạm hoặc hỗ trợ quá trình
phát triển. Không tổ chức lại thư mục này thành một hệ thống CI/CD thứ hai.

- CI/CD chính thức thuộc `.github/workflows/`.
- Công cụ có logic dùng lại, giao diện rõ ràng và kiểm thử thuộc `tools/`.
- Khi một script nhỏ phát triển thành chức năng được duy trì, chuyển nó đến
  nơi sở hữu phù hợp thay vì để nhiều bản sao.
