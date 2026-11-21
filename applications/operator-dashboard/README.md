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

## Bàn thử Qt và RViz

Bàn thử mở ca NB6 giảm độ bám bánh trái khi khởi động. Qt điều khiển kịch bản,
biểu đồ và phát lại. RViz hiển thị robot theo cùng con trỏ thời gian, với
tâm khối lượng và vectơ lực tiếp xúc từ mẫu phát lại.
Bảng **Tiếp xúc và mô-men tại mẫu đang phát** trong Qt hiện trạng thái tiếp
xúc, lực pháp tuyến và mô-men từng bánh, từng hông bằng tiếng Việt. Màu cam
và xanh dương tương ứng với vectơ lực của bánh trái và phải trong RViz.
Đầu mũi tên nằm tại điểm tiếp xúc và hướng theo lực tác dụng lên bánh, với
tỷ lệ vẽ 0,004 m/N.

Trên máy Linux có Docker và X11/XWayland, dùng môi trường
[mô phỏng Jazzy](../../docs/development/simulation-preview.md).

```bash
python3 tools/dev/duk-dev.py configure --preset vmware
python3 tools/dev/duk-dev.py up
python3 tools/dev/duk-dev.py shell
```

Trong container, từ `/workspace`:

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/workspace/robot/software/ros2_ws/src/duk_description:${PYTHONPATH}"
OPENBLAS_NUM_THREADS=1 python applications/operator-dashboard/qt_workbench.py
```

Thêm `--no-rviz` để mở Qt riêng. Có thể bấm **Mở RViz** sau khi Qt đã chạy.
**Nạp kịch bản JSON** và **Lưu kịch bản JSON** đọc hoặc lưu các đầu vào đang
hiển thị. Dấu `/` phân cách các thành phần của một vectơ trong biểu mẫu.

### Tạo kịch bản NB6 với nhiều mốc

1. Chọn **NB6** rồi chọn **Kịch bản mới** để bắt đầu với tham số danh định
   và lịch trống. Kết quả trước vẫn xem được trong khi chỉnh kịch bản.
2. Chọn **Sửa lịch**, thêm dòng, đặt thời điểm, tác động và giá trị. Có thể
   lập nhiều mốc tiến/quay/dừng, đổi độ bám riêng cho mỗi bánh, thay độ lệch
   cao độ đặt, bật hoặc tắt tác vụ độ cao, cân bằng ngang, neo vị trí và giữ
   hướng, đặt bước nhảy vận tốc trong hệ thế giới và chọn lỗi giả lập.
3. Chọn **Áp dụng lịch** rồi **Chạy mô phỏng**. Trong tab **Sự kiện**, mỗi
   dòng ghi giá trị, thời điểm yêu cầu và thời điểm thực tế. Kích hoạt dòng
   có mốc thực tế để tới mẫu tương ứng.
4. **Lưu kịch bản JSON** để mở lại hoặc chạy tệp đó bằng `--scenario`.
   **Tệp kết quả → Lưu kết quả** giữ các mốc thực tế cùng tín hiệu và cấu
   hình đã dùng.

Lịch tùy chỉnh thay các mốc tiến/quay, đổi bám, nhiễu động và lỗi đơn lẻ của
ca mẫu. Vận tốc đặt và tốc độ quay đặt bắt đầu ở 0, giữ đến mốc đổi tiếp theo.
Độ bám ban đầu vẫn lấy từ biểu mẫu. Các tác vụ giữ thiết lập ban đầu cho đến
khi được đổi trong lịch. Độ lệch cao độ tác động khi tác vụ độ cao được bật.
Bước nhảy vận tốc được áp một lần; chọn **Không lỗi** để kết thúc lỗi. Nếu
lỗi đã làm ca dừng, các mốc sau đó vẫn được ghi với thời điểm thực tế để trống.

Mốc áp dụng ở mẫu lệnh đầu tiên từ thời điểm yêu cầu. Có thể đổi hai bánh
trong cùng mẫu. Hai dòng đổi cùng một tác động trong cùng mẫu sẽ được báo lỗi.
Các mốc đúng thời lượng kết thúc hoặc sau mẫu lệnh cuối không được áp dụng.
Kịch bản NB6 cũ không có `schedule` tiếp tục dùng các mốc của ca mẫu.

Ví dụ JSON cho cùng luồng Qt và CLI:

```json
{
  "study": "NB6",
  "duration_s": 0.45,
  "schedule": [
    {"time_s": 0.0, "field": "drive_speed_m_s", "value": 0.15},
    {"time_s": 0.12, "field": "friction_left", "value": 0.3},
    {"time_s": 0.18, "field": "turn_rate_rad_s", "value": 0.12},
    {"time_s": 0.27, "field": "drive_speed_m_s", "value": 0.0},
    {"time_s": 0.27, "field": "turn_rate_rad_s", "value": 0.0}
  ]
}
```

### Chạy NB6 từng bước

- Chọn **Chạy từng bước NB6** để chốt cấu hình và chuẩn bị ở 0 s.
  **Tính 1 bước** chạy một khoảng lệnh, cùng các bước tích phân con bên
  trong. Nếu mô hình dừng trong khoảng đó, kết quả giữ mốc dừng thực.
- **Chạy tiếp** giữ trạng thái mô phỏng, bộ ước lượng và lịch sự kiện rồi
  chạy tới mốc cuối. **Tạm dừng tính** chờ khoảng đang tính kết thúc.
  Biểu đồ và RViz hiển thị mốc mới khi tạm dừng, hủy hoặc kết thúc. Nút
  **Phát** bên thanh thời gian chỉ phát lại dữ liệu đã tính.
- Tại biên tạm dừng, kết quả có tư thế, lực và mô-men của khoảng vừa tính.
  Lệnh và sự kiện ở đầu khoảng kế tiếp chưa được xử lý. Khi chạy tiếp,
  các mốc đó được áp dụng đúng một lần theo cấu hình đã chốt.
- **Hủy ca** chốt dữ liệu tại biên đã tính. Nếu chưa tính bước nào, giữ
  kết quả trước. Có thể lưu bản ghi tạm dừng hoặc đã hủy để xem lại. Trạng
  thái chạy tiếp được giữ trong phiên Qt. Để tái lập một bản ghi từng phần
  sau khi đóng phiên, mở kịch bản và chạy lại từ đầu.

### Các bài thử có sẵn

| Bài thử | Cấu hình và tín hiệu chính |
| --- | --- |
| NB1–NB2 | Góc, đạo hàm riêng mỗi chân, quỹ đạo quintic, IK tâm bánh, mặt đỡ lệch cao độ, tải tại tâm bánh và đường bao mô-men–tốc độ. Hệ thân và hệ thế giới dùng hình học chuẩn. Tải hông được tính tĩnh trong mặt phẳng sagittal. |
| NB3 | Động lực học phẳng 7 bậc tự do. Lăn cứng, tiếp xúc đàn hồi, trượt, bay và chạm đất. Lực, năng lượng, odometry và ước lượng phát hiện trượt. Bộ tích phân ghi mốc guard chạm đất hoặc mất khả thi của mô hình lăn. |
| NB4 | Con lắc hông cố định, LQR giữ vị trí hoặc thăng bằng, vận tốc mô hình/odometry/ước lượng, trễ lệnh, nhiễu và giới hạn mô-men theo độ bám. Bài thử dùng một mô-men tổng theo chiều tiến cho cặp bánh. |
| NB5 | LQR cố định, LQR theo góc hông, TV-LQR và MPC trên cùng quỹ đạo nâng đối xứng. Xem ma trận tuyến tính, gain, giới hạn, mô-men tổng, tải hông và lực. |
| NB6 | QP có trọng số hoặc HQP, lựa chọn thứ tự nhiệm vụ, độ bám và cao độ riêng từng bánh. Có tham chiếu độ cao, tốc độ, tốc độ quay, giữ ngang, giữ hướng và về điểm neo. Xem tham chiếu áp dụng, trạng thái tiếp xúc, độ bám ước lượng, lực, bốn mô-men, giám sát và trạng thái cuối. |

Chọn **Bánh trái**, **Cơ cấu bốn khâu** hoặc **Thân** trong cây để lọc theo bộ
phận. Nhấp đôi một tín hiệu để đưa lên biểu đồ. Tab **Phương trình** lấy đúng
mẫu đang xem. Tab **Sự kiện** ghi mốc yêu cầu và mốc thực tế, nhấp đôi để tới
mẫu tương ứng. NB4 có các tín hiệu của mô hình gộp trong Qt. Các mô hình có
đầy đủ tư thế và góc cơ cấu phát lại trong RViz.

Trong NB1–NB2, chọn **Quét góc và tải hông tĩnh** rồi đặt góc đầu và cuối từng
chân. Nhập tốc độ khảo sát trái/phải vào ô vận tốc góc. Mỗi mẫu lấy mô-men
tĩnh tại góc đang xem và tra đường bao động cơ ở tốc độ khảo sát đó. Vận tốc
và gia tốc chuyển động hình học vẫn lấy từ quỹ đạo quintic. Chọn một thanh
như **BC** để xem hai đầu thanh, chiều dài, hướng và các đạo hàm theo góc hông.

Trong NB3, **Thời điểm đổi độ bám** để trống sẽ giữ độ bám ban đầu suốt ca.
Khi nhập thời điểm, nhập thêm hai hệ số sau thay đổi theo thứ tự trái/phải.
Core ghi mẫu ngay lúc đổi, kể cả giữa hai mốc lấy mẫu. Trạng thái liên tục qua
mốc này. Lực và hệ số tại mốc dùng điều kiện mới. Nếu lực yêu cầu vượt miền
lăn cứng, mốc đó cũng là thời điểm dừng. NB6 áp dụng thay đổi ở mẫu lệnh đầu
tiên từ thời điểm yêu cầu trở đi.

Trong NB5, **Khoảng mô-men khả thi** áp dụng cho mô-men tổng của cặp bánh.
Các dư lượng lực ở lệnh áp dụng phải nhỏ hơn hoặc bằng 0 trong dung sai số.
Thứ tự và đơn vị của từng thành phần nằm trong tab **Tham số**.
Sai số bám dùng tham chiếu có hiệu lực trừ giá trị mô hình. Sai số ước lượng
vận tốc dùng giá trị ước lượng trừ vận tốc mô hình. Mẫu thiếu hiển thị bằng
dấu gạch ngang.

### Ca nghiệm thu

Mở ca [giảm độ bám bánh trái](../../robot/simulation/scenarios/whole-body-left-grip.json).
Tại 0,36 s, độ bám bánh trái đổi từ 0,65 xuống 0,13 và bánh phải giữ 0,65.
Nhiễu vận tốc được đặt ở 0,39 s. Xem lực, trượt, mô-men hông và tư thế thân ở
cùng mốc. Chọn **So sánh điều khiển** để chạy HQP với cùng cấu hình và seed.
Tại mẫu dừng NB6, tư thế và lực vẫn phát lại được nếu hình học còn giải được.
Các trường lệnh tại cuối bước tích phân để trống nếu vòng điều khiển đã kết thúc.
Mô-men giữ ở bước trước vẫn hiển thị tại mốc này. Trạng thái tiếp xúc cuối bước
dùng khe hở, lực quan sát và tốc độ trượt của mô hình.
Tín hiệu lực NB6 dùng lực quan sát từ bước tích phân trước, với tải tĩnh khởi
tạo tại thời điểm 0. Chế độ tiếp xúc và sự kiện NB6 là quan sát theo mẫu lệnh.

### Tái lập và xuất tín hiệu

**Lưu kết quả** tạo JSON chứa kịch bản đã áp dụng, dấu vân tay mã, các tham số,
seed, thông tin phiên bản, tín hiệu, đơn vị, hệ tọa độ và sự kiện. **Mở kết quả**
đọc lại dữ liệu mà không chạy bộ mô phỏng. **Đối chiếu tệp** chồng hai ca cùng
mô hình trên cùng trục thời gian. Con trỏ chạy tới hết cả hai ca. Bảng để trống
giá trị ngoài thời lượng từng ca, và chú thích con trỏ ghi **Đối chiếu** khi
đang hiển thị tư thế từ ca đối chiếu. **Xuất CSV** giữ độ chính xác số đầy đủ, còn
Qt dùng dấu phẩy thập phân và hiển thị sai số rất nhỏ dưới dạng `< 0,000001`.

**Chạy lại và kiểm tra** dùng dung sai tuyệt đối 0,00000001 và tương đối
0,0000001. Thời gian thực thi bộ giải mang tính đo đạc và được loại khỏi phép
đối chiếu số. Dùng phiên bản mã và bộ tham số gốc khi tái lập.

Từ repo đã cài [core với research extra](../../robot/software/core/README.md),
lệnh CLI dưới đây chạy được trong môi trường Python không có Qt hoặc ROS.
Tạo thư mục đích trước khi ghi.

```bash
OPENBLAS_NUM_THREADS=1 python applications/operator-dashboard/qt_workbench.py \
  --scenario robot/simulation/scenarios/whole-body-left-grip.json \
  --output data/results/left-grip-qp.json

OPENBLAS_NUM_THREADS=1 python applications/operator-dashboard/qt_workbench.py \
  --scenario robot/simulation/scenarios/whole-body-left-grip.json \
  --controller hierarchical --output data/results/left-grip-hqp.json

OPENBLAS_NUM_THREADS=1 python applications/operator-dashboard/qt_workbench.py \
  --open data/results/left-grip-qp.json --output data/results/left-grip-repeat.json
```

`--profile` nhận một hồ sơ tổng hợp có cùng cấu trúc và phiên bản với
[hồ sơ NB1–NB6](../../research/experiments/duk-v3/synthetic.json). Điều này cho
phép thử độ nhạy theo khối lượng, CoM, quán tính, cơ cấu chấp hành và tham số
tiếp xúc. Hai mặt đỡ NB6 là hai mặt ngang độc lập. Các giả thiết về trọng lực,
đĩa bánh, độ bám và vị trí khớp nằm trong core và bộ tham số.

Trong Qt, **Chạy với bộ tham số…** nạp hồ sơ JSON rồi chạy bài thử đang chọn.
Các ô có giá trị khác mặc định được giữ lại. Các ô đang ở giá trị mặc định
lấy giá trị từ hồ sơ mới. Khi hồ sơ khớp với ca đang xem, biểu mẫu giữ toàn bộ
giá trị. Kết quả ghi tên và SHA-256 của hồ sơ đã dùng.

### Phạm vi của Gazebo và dữ liệu thật

[Gazebo preview](../../docs/development/simulation-preview.md) đối chiếu tâm
và bán kính hai bánh cùng các đầu thanh từ cảnh Gazebo đang chạy với nghiệm
Python tại góc đã chọn, với dung sai 0,000001 m. Kiểm tra cũng đọc TF và đồng
hồ mô phỏng. Đối chiếu động lực học cần mô hình
Gazebo đã khai báo ràng buộc vòng kín, trục/mô-men, va chạm và luật tiếp xúc.
Trình tự đối chiếu theo kế hoạch là hình học, tải tĩnh, chuyển động đặt, tiếp
xúc và vòng kín trên các điều kiện ban đầu chung. Dốc và địa hình biến thiên
cần mô hình mặt đỡ tương ứng.

[Dữ liệu vật lý](../../robot/config/physical/README.md) xác định nguồn hình học
chuẩn và các tham số còn chờ nhận dạng. Chuyển sang robot thật cần hồ sơ CAD,
khối lượng/quán tính, truyền động, lốp và cảm biến đã hiệu chuẩn. Mô hình
điện và nhiệt cần dữ liệu đầu vào riêng. Việc thử trên phần cứng có ủy quyền
riêng.
