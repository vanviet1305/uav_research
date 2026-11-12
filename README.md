# UAV research — P01 và Gazebo X500

Đọc tệp này để chạy phiên bản đã sửa. Phạm vi điều khiển trực tiếp là **PX4 SITL (PX4 chạy mô phỏng trên máy tính) + Gazebo `gz_x500` tiêu chuẩn**. Mã chưa được xác nhận trên UAV thật. Bộ tham số X500 là lựa chọn kỹ thuật riêng; không phải tham số hay kết quả của bài báo.

## Chọn đúng chương trình

| Chương trình | Công việc |
|---|---|
| `offboard_control` | Điều khiển vị trí qua các vòng điều khiển PX4, có yêu cầu Arm/cất cánh về `[0,0,-3]`. Không chạy cùng P01 chủ động. |
| `p01_offboard_hover` | P01 điều khiển trực tiếp bốn động cơ; chọn `hover` (giữ vị trí) hoặc `case1` (quỹ đạo đã điều chỉnh). Mặc định chỉ tính, chưa phát lệnh. |
| `p01_shadow_controller` | Chỉ quan sát và tính giữ vị trí bằng cấu hình X500 mặc định; không phát lệnh. Muốn kiểm tra tham số tùy chỉnh, dùng chế độ mặc định không phát lệnh của `p01_offboard_hover`. |
| `p01_offline_case1` | Mô phỏng lý tưởng theo mã P01 cũ; không có giới hạn/độ trễ động cơ. Giữ nguyên để đối chiếu. |
| `p01_x500_offline` | Mô hình X500 giản lược có giới hạn lực, điều khiển rời rạc, trễ một chu kỳ, đáp ứng tốc độ động cơ và nhiễu đo. Không phải Gazebo. |
| `test_x500_control` | Kiểm tra phân bổ lực, đạo hàm quỹ đạo, chuyển quyền Offboard và phiên bản tên kênh dữ liệu. |

## Chuẩn bị và biên dịch trên Ubuntu/ROS 2

Thực hiện trong workspace ROS 2 đang chứa `src/uav_research` và `px4_msgs`. Ví dụ dưới dùng `~/ros2_ws`; đổi sang đường dẫn workspace của bạn. Cần ROS 2, `rclcpp`, `rcl_interfaces`, `geometry_msgs`, `px4_msgs`, Eigen 3 và `eigen3_cmake_module`. `px4_msgs` phải tương thích với PX4 đang chạy, có trường `OffboardControlMode.direct_actuator`.

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select uav_research --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select uav_research
colcon test-result --verbose
```

Đã đăng ký các bài thử C++ trong CMake; các kiểm tra định dạng có sẵn của package vẫn được giữ. Các bài thử động lực học xuất CSV trong thư mục build. `CMake_additions.txt` là ghi chú cũ, không chép thêm các target từ đó vào CMakeLists.

## Chạy giữ vị trí bằng P01

1. Khởi động PX4 `gz_x500`, cầu nối Micro XRCE-DDS và QGroundControl như môi trường đang dùng.
2. Dùng QGroundControl mở khóa động cơ, cất cánh khoảng 3 m, chuyển **Hold** và chờ đứng yên. Không chạy `offboard_control` đồng thời.
3. Có thể quan sát trước bằng lệnh dưới; chương trình chỉ tính, không phát bất kỳ lệnh bay nào. Kết thúc lần quan sát trước khi chạy chế độ chủ động.

```bash
ros2 run uav_research p01_offboard_hover
```

4. Chạy giữ vị trí chủ động:

```bash
ros2 run uav_research p01_offboard_hover --ros-args \
  -p enable_actuation:=true \
  -p reference_mode:=hover \
  -p log_csv:=p01_hover_log.csv
```

Chương trình đợi trạng thái mới, dữ liệu NED hợp lệ, UAV đã mở khóa động cơ, ở Hold/Position, cao hơn gốc tọa độ cục bộ hơn 2 m, vận tốc dưới 0,3 m/s, tốc độ góc dưới 0,3 rad/s và gần nằm ngang. Sau khoảng 1,2 s ổn định và gửi tín hiệu duy trì Offboard, chương trình yêu cầu chuyển chế độ đúng một lần. Chỉ gửi lệnh động cơ sau khi PX4 xác nhận Offboard. Mục tiêu được cập nhật trong giai đoạn chờ rồi giữ cố định khi yêu cầu chuyển quyền; khởi động chương trình trên mặt đất không khóa mục tiêu mãi ở mặt đất.

Các mã `stage` trong CSV: `0` chờ, `1` gửi tín hiệu chuẩn bị, `2` chờ PX4 xác nhận, `3` đang điều khiển, `4` đã dừng phiên. Trong chế độ chỉ quan sát, chương trình tính giữ vị trí tại mẫu đầu tiên; chọn `case1` vẫn chưa chạy quỹ đạo khi chưa nhận quyền Offboard.

**Kết thúc thử:** chuyển UAV về Hold trong QGroundControl trước, kiểm tra PX4 đã thoát Offboard rồi dừng chương trình. Nếu thoát Offboard, mất dữ liệu, bộ ước lượng đổi gốc hoặc phát hiện chương trình cùng phát lệnh, phiên sẽ dừng và cần khởi động lại. Không tự tái chiếm quyền điều khiển. Nếu mất dữ liệu trong phiên đang điều khiển, mã yêu cầu Hold một lần và ngừng tín hiệu duy trì; thành công vẫn phụ thuộc PX4 chấp nhận lệnh và có trạng thái ước lượng hợp lệ. Cấu hình xử lý mất Offboard của PX4 vẫn là cơ chế dự phòng, không phải bảo đảm UAV không rơi.

## Chạy quỹ đạo đã điều chỉnh

Sau khi kiểm tra giữ vị trí, đưa UAV về Hold rồi khởi động một phiên mới:

```bash
ros2 run uav_research p01_offboard_hover --ros-args \
  -p enable_actuation:=true \
  -p reference_mode:=case1 \
  -p trajectory_speed:=0.25 \
  -p log_csv:=p01_case1_log.csv
```

Quỹ đạo bắt đầu sau 3 s kể từ khi PX4 xác nhận Offboard. Nó dịch về vị trí đang bay, quay các trục ngang theo hướng mũi lúc nhận quyền, chạy chậm bằng 1/4 tốc độ tham số Case I và tăng/giảm tốc êm trong 4 s ở hai đầu. Sau 30 s chuyển động, giữ vị trí cuối; mặc định giữ nguyên hướng mũi. Có thể bật `trajectory_rotate_heading:=true` để quay hướng mũi theo tham chiếu đã làm chậm. Mặc định độ cao tham chiếu không thấp hơn độ cao lúc nhận quyền. Thời gian quỹ đạo lấy từ dấu thời gian odometry của PX4; thời gian chờ/mất liên lạc lấy từ đồng hồ thực của máy.

Đây là **Case I đã thay đổi để kiểm tra kỹ thuật**, không phải tái hiện nguyên bản bài báo. Hàm `reference_case1` và chương trình lý tưởng cũ vẫn được giữ nguyên. Chưa có kết luận lý thuyết về ổn định toàn cục cho cấu hình X500 có bão hòa này.

## Những thay đổi xử lý lỗi rơi/lật

- Bộ phân bổ lực dùng giới hạn của từng động cơ, ưu tiên tổng lực, sau đó mômen nghiêng ngang/dọc, cuối cùng mômen quay hướng mũi. Khi yêu cầu vượt khả năng, nó giảm phần yêu cầu không thực hiện được và tiếp tục phát lệnh hợp lệ. `saturated=1` nghĩa là chạm giới hạn, không phải mất lệnh.
- Cấu hình X500 dùng hệ số tư thế và tốc độ góc riêng cho từng trục, giảm yêu cầu mômen quá lớn ở trục quay hướng mũi. Tham số bài báo trong chương trình lý tưởng không bị thay thế.
- Tên kênh dữ liệu PX4 tự lấy hậu tố phiên bản từ `px4_msgs`; vẫn có thể đổi tên bằng ROS remapping. Việc này không tự chuyển đổi cấu trúc thông điệp khi hai bên khác phiên bản.
- Có kiểm tra xung đột chương trình phát Offboard/động cơ, phản hồi cũ/sai, dấu thời gian lặp hoặc đi lùi và bộ đếm đổi gốc ước lượng. Phát hiện xung đột dựa trên danh sách chương trình ROS đã được phát hiện, không phát hiện được mọi nguồn điều khiển MAVLink bên ngoài.
- `offboard_control` giữ hướng mũi ban đầu thay vì ép quay ngay về 180° và dừng phát khi có chương trình ROS khác cùng gửi Offboard.
- Nhật ký ghi cả lực/mômen yêu cầu lẫn giá trị đạt được sau phân bổ, lệnh bốn động cơ, sai số và trạng thái phiên.

## Tham số cần khớp với mô hình PX4/Gazebo của bạn

Tất cả tham số P01 chủ động được kiểm tra lúc khởi động và chỉ đọc trong phiên. Muốn đổi, trở về Hold rồi khởi động lại với tham số mới.

| Tham số | Mặc định | Ý nghĩa |
|---|---:|---|
| `mass_kg` | 2.0643076923 | Tổng khối lượng X500, kg |
| `inertia_diagonal` | `[0.02383948,0.02394241,0.04399995]` | Mômen quán tính, kg·m² |
| `position_gain_per_mass`, `velocity_gain_per_mass` | 4.0; 3.2 | Hệ số vị trí/vận tốc trước khi nhân khối lượng |
| `attitude_gains` | `[1.5,1.5,0.35]` | Hệ số mômen phản hồi sai số tư thế |
| `rate_gains` | `[0.35,0.35,0.22]` | Hệ số phản hồi sai số tốc độ góc |
| `rotor_arm_m` | 0.174 | Độ lệch theo mỗi trục x/y của rotor, không phải chiều dài đường chéo |
| `rotor_moment_ratio` | 0.016 | Tỉ số mômen phản lực/lực đẩy vật lý trong Gazebo |
| `motor_constant` | 8.54858e-6 | Hệ số lực đẩy theo bình phương tốc độ rotor |
| `motor_speed_min`, `motor_speed_max` | 150; 1000 | Giới hạn đầu ra PX4–Gazebo, rad/s |
| `motor_thrust_factor` | 0.0 | Phải bằng hệ số mô hình lực đẩy `THR_MDL_FAC` thực tế áp dụng trong đường đầu ra PX4 |

Thứ tự động cơ phải là 0–3 của X500 chuẩn, các đầu ra `SIM_GZ_EC_FUNC1..4` lần lượt 101–104, min/max khớp bảng. Nếu đã sửa SDF hoặc dùng X500 gắn camera/tải, kiểm tra lại khối lượng, quán tính và động cơ trước khi dùng cấu hình này. Mã không tự đọc tham số PX4 từ xa. Không thay `rotor_moment_ratio` bằng giá trị `CA_ROTOR*_KM` của bộ phân bổ PX4 chỉ vì chúng có cùng tên gần giống nhau.

Nếu không vào được chế độ điều khiển, xem dòng `Waiting/dry-run` và tên kênh ở đầu chương trình; dùng `ros2 topic list`, `ros2 topic hz /fmu/out/vehicle_odometry`, `ros2 topic info /fmu/in/offboard_control_mode --verbose`. Khuyến nghị odometry 50–100 Hz; khoảng cách mẫu lớn hơn 50 ms hoặc không có mẫu mới trong 150 ms bị coi là lỗi phản hồi. Tên có hậu tố phiên bản phải dùng đúng như chương trình in ra.

## Kiểm tra độc lập, không cần ROS

Cần C++17 và Eigen 3. Truyền đường dẫn thư mục chứa `Eigen/Dense`:

```bash
python3 tools/run_core_checks.py --eigen /usr/include/eigen3
```

Bài kiểm tra gồm 10.000 yêu cầu phân bổ lực, hồi quy lệch hướng 2°, ánh xạ động cơ, đạo hàm quỹ đạo đến bậc bốn, chuyển chế độ, giữ vị trí và bám quỹ đạo. Mô hình giản lược dùng bước tích phân 1 ms, điều khiển 50/100 Hz, trễ một chu kỳ, hằng số đáp ứng động cơ 12,5/25 ms, nhiễu đo cố định hạt giống. Mô hình chưa gồm toàn bộ ước lượng trạng thái PX4, truyền thông, khí động, va chạm, tiếp xúc mặt đất hoặc thay đổi tải.

Các bài kiểm tra C++ độc lập đã chạy trên Windows ngày 11/11/2026. Chưa biên dịch/chạy toàn bộ ROS 2 hoặc Gazebo trong môi trường này. Xem `VALIDATION.md` để biết kết quả cụ thể; không dùng số liệu mô hình giản lược làm kết quả Gazebo.

## Nguồn kỹ thuật đối chiếu

- [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard): cơ chế tín hiệu duy trì, quyền điều khiển và đầu ra trực tiếp động cơ.
- [Cấu hình PX4 gz_x500](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500): thứ tự, vị trí và giới hạn đầu ra động cơ.
- [Gazebo X500](https://github.com/PX4/PX4-gazebo-models/blob/main/models/x500/model.sdf): hệ số lực, mômen và hằng số đáp ứng động cơ.
- [Đường ánh xạ động cơ PX4](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/mixer_module/functions/FunctionMotors.hpp): ánh xạ lệnh chuẩn hóa và hệ số mô hình lực đẩy.

Các liên kết `main` chỉ dùng đối chiếu; cần so với đúng phiên bản PX4/Gazebo đang chạy trên máy mô phỏng của bạn.
