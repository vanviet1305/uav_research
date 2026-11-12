# Kết quả kiểm tra bản sửa X500 — 11/11/2026

## Phạm vi đã thực hiện

Đã biên dịch mã C++ thật bằng GCC 16.2, C++17, Eigen 3.4.0 trên Windows; bật `-Wall -Wextra -Wpedantic`. Không dùng mã thay thế cho bộ điều khiển trong các kiểm tra này. Thư viện Eigen được tải từ bản phát hành chính thức vào thư mục kiểm tra ngoài package, không đưa vào mã nguồn `uav_research`.

Các chương trình độc lập đã biên dịch/chạy: `test_p01`, `test_p01_controller`, `test_x500_control`, `p01_allocation_check`, `p01_offline_case1`, `p01_x500_offline`. `test_p01` và `p01_allocation_check` là các chương trình in số liệu; kiểm tra tự động nằm trong `test_p01_controller`, `test_x500_control` và các ngưỡng cuối chương trình `p01_x500_offline`.

Đã kiểm tra cú pháp Python của trình chạy kiểm tra, cấu trúc XML của package và lỗi khoảng trắng trong thay đổi Git. Chưa có ROS 2/colcon/Gazebo trong môi trường này, nên **chưa biên dịch các chương trình ROS hoặc xác nhận kết quả bay Gazebo**. Cấu hình bài kiểm tra cho `colcon test` đã được thêm nhưng chưa chạy bằng colcon tại đây.

## Phân bổ lực và chuyển chế độ

- 10.000 yêu cầu lực/mômen ngẫu nhiên: lệnh động cơ hữu hạn trong `[0,1]`, lực mỗi rotor trong giới hạn, tổng lực đạt được bằng tổng lực đã giới hạn; mômen nghiêng ngang/dọc giữ đúng hệ số giảm chung.
- Kiểm tra dấu mômen ngang/dọc độc lập bằng tích có hướng giữa vị trí rotor và lực đẩy; kiểm tra thứ tự mômen quay hướng mũi theo X500 chuẩn.
- Trường hợp hồi quy dùng mômen của bộ hệ số cũ tại sai số hướng mũi 2°: yêu cầu −0,30746 N·m, thực hiện −0,22310 N·m, giữ tổng lực 20,25086 N; vẫn tạo đủ bốn lệnh động cơ hợp lệ. Bộ hệ số X500 mới ở cùng sai số 2° không chạm giới hạn.
- Ánh xạ lực → tốc độ → lệnh đã kiểm tra với hệ số mô hình lực đẩy 0, 0,5 và 1.
- Đạo hàm vị trí đến bậc bốn và hướng mũi đến bậc hai được đối chiếu bằng sai phân tại các đoạn chạy và điểm nối tăng/giảm tốc. Kiểm tra bắt đầu đứng yên, không tụt tham chiếu xuống dưới độ cao ban đầu và dừng giữ vị trí cuối.
- Máy trạng thái chuyển chế độ đã kiểm tra: chỉ quan sát không phát lệnh; chưa ổn định thì chưa nhận quyền; gửi tín hiệu trước khi yêu cầu Offboard; chờ xác nhận trước khi phát lệnh động cơ; mất dữ liệu thì yêu cầu Hold và kết thúc phiên; không tự tái nhập; tôn trọng thoát thủ công; phát hiện nguồn phát trùng; hết hạn chờ xác nhận.

## Mô hình X500 giản lược

Mỗi lần chạy dài 40 s, tích phân vật rắn bước 1 ms. Có trễ một chu kỳ điều khiển, đáp ứng tốc độ rotor bậc nhất với hằng số tăng/giảm tốc 12,5/25 ms, giới hạn lực rotor, nhiễu đo vị trí/vận tốc/tốc độ góc. Hạt giống nhiễu cố định để tái kiểm tra được. Đây không phải động cơ vật lý Gazebo hoặc bộ ước lượng PX4.

Điều kiện đầu thường: lệch vị trí `[0,15; −0,10; 0,10]` m, nghiêng ngang 5°, lệch hướng mũi 2° so với mục tiêu. Trường hợp hồi phục thay lệch hướng mũi bằng 60° và tốc độ quay trục đứng 0,1 rad/s để chủ động gây chạm giới hạn.

| Trường hợp | Tần số | Sai số vị trí lớn nhất sau 5 s | Sai số vị trí cuối | Số mẫu chạm giới hạn |
|---|---:|---:|---:|---:|
| Giữ vị trí | 100 Hz | 0,004485 m | 0,002087 m | 0 |
| Case I đã điều chỉnh, hướng mũi cố định | 100 Hz | 0,006781 m | 0,002087 m | 0 |
| Case I đã điều chỉnh, hướng mũi cố định | 50 Hz | 0,008337 m | 0,002352 m | 0 |
| Case I đã điều chỉnh, hướng mũi quay theo tham chiếu | 100 Hz | 0,006837 m | 0,002075 m | 0 |
| Hồi phục sau sai lệch hướng mũi 60° | 100 Hz | 0,004485 m | 0,002087 m | 14 |

Tất cả vượt qua ngưỡng kiểm tra: sai số vị trí sau 5 s dưới 0,2 m; sai số cuối dưới 0,03 m; góc nghiêng ngang/dọc dưới 35°; hàm sai số tư thế cuối `Psi` dưới 0,001. `Psi` không phải góc tính bằng radian. Trường hợp hồi phục bắt buộc có ít nhất một mẫu chạm giới hạn.

Chương trình lý tưởng P01 cũ cũng đã chạy lại: sai số vị trí tại 10 s khoảng 3,06 × 10⁻¹² m. Số này chỉ xác nhận chương trình lý tưởng vẫn hoạt động, không dùng để so sánh trực tiếp với Gazebo hoặc chứng minh UAV thực hiện được lực/mômen yêu cầu.

Nhật ký máy hiện tại nằm ngoài package tại `../tmp/uav_gazebo_audit/final_checks/validation.log`; kết quả cuối có thêm ngưỡng tư thế nằm tại `../tmp/uav_gazebo_audit/final_checks/x500_final.log`. Có thể tái tạo bằng `tools/run_core_checks.py` theo README.

## Xác nhận tiếp trong môi trường PX4/Gazebo

1. Biên dịch package và chạy `colcon test`; kiểm tra đúng phiên bản thông điệp PX4, thứ tự động cơ, min/max và `THR_MDL_FAC`.
2. Dùng QGroundControl cất cánh 3 m và chọn Hold; chạy P01 giữ vị trí, thu CSV cùng nhật ký PX4. Kiểm tra phiên chuyển `1 → 2 → 3`, `motor_gate=ACTIVE`, sai số ổn định và tần suất chạm giới hạn.
3. Chuyển về Hold để kết thúc thử giữ vị trí, rồi chạy phiên quỹ đạo riêng. Không chạy đồng thời hai nguồn phát Offboard.
4. Đối chiếu lực yêu cầu/đạt được và trạng thái PX4 quanh thời điểm bất thường. Chỉ sau bước này mới có thể khẳng định lỗi rơi/lật đã được khắc phục trong đúng môi trường Gazebo của người dùng.
