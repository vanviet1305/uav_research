# Phiên bản CMake tối thiểu cần thiết để đọc file cấu hình này.
cmake_minimum_required(VERSION 3.8)

# Tên project/package ROS 2. Phải thống nhất với tên package trong package.xml.
project(uav_research)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Bố cục: tìm dependency -> khai báo executable -> liên kết -> cài đặt -> ament_package.
# Mỗi target bên dưới là một chương trình độc lập, không tự chạy khi build.

# Nếu trình biên dịch là GCC hoặc Clang thì bật thêm cảnh báo.
# Các tùy chọn này giúp phát hiện lỗi code C++, không phải lỗi lúc UAV bay.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Tìm các package mà chúng ta sẽ dùng để biên dịch chương trình.
# ament_cmake: công cụ build package ROS 2 bằng CMake.
# rclcpp: API viết ROS 2 node bằng C++.
# px4_msgs: các kiểu message của PX4.
# geometry_msgs: có kiểu Point để nhận mục tiêu x, y, z từ ROS 2.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)


# ============================================================
# CHƯƠNG TRÌNH 1: odometry_listener
# ============================================================

# Build odometry_listener node
# Tạo target (chương trình thực thi) từ file C++ bên dưới.
add_executable(odometry_listener src/odometry_listener.cpp)
target_include_directories(odometry_listener PRIVATE include)

# Link ROS 2 and PX4 dependencies
# Khai báo các thư viện mà riêng odometry_listener cần dùng.
ament_target_dependencies(odometry_listener
  rclcpp
  px4_msgs
)

# Install executable for ros2 run
# Cài chương trình vào install/lib/uav_research để chạy:
# ros2 run uav_research odometry_listener
install(TARGETS odometry_listener
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# CHƯƠNG TRÌNH 2: offboard_control
# ============================================================

# Build offboard_control node
# Yêu cầu CMake tạo chương trình thực thi tên offboard_control
# từ file src/offboard_control.cpp.
add_executable(offboard_control src/offboard_control.cpp)
target_include_directories(offboard_control PRIVATE include)

# Link ROS 2 and PX4 dependencies
# Khai báo các thư viện và package mà chương trình cần sử dụng.
# geometry_msgs cần thiết vì code mới subscribe geometry_msgs/msg/Point.
ament_target_dependencies(offboard_control
  rclcpp
  px4_msgs
  geometry_msgs
)

# Install executable for ros2 run
# Yêu cầu cài đặt chương trình đã biên dịch vào thư mục thích hợp
# để sau này có thể chạy bằng: ros2 run uav_research offboard_control
install(TARGETS offboard_control
  DESTINATION lib/${PROJECT_NAME}
)



# ============================================================
# CHUONG TRINH 3: p01_reference_test
# ============================================================

add_executable(test_p01
  src/test_p01.cpp
)

target_include_directories(test_p01 PRIVATE
  include
)

target_link_libraries(test_p01
  Eigen3::Eigen
)

install(TARGETS test_p01
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# KIỂM TRA TÙY CHỌN TRONG QUÁ TRÌNH BUILD
# ============================================================
# ============================================================
# P01: KIEM TRA BO DIEU KHIEN
# ============================================================

add_executable(test_p01_controller
  src/test_p01_controller.cpp
)

target_include_directories(test_p01_controller PRIVATE include)

target_link_libraries(test_p01_controller Eigen3::Eigen)

install(TARGETS test_p01_controller
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# P01: MO PHONG DONG LUC HOC
# ============================================================

add_executable(p01_offline_case1
  src/p01_offline_case1.cpp
)

target_include_directories(p01_offline_case1 PRIVATE include)

target_link_libraries(p01_offline_case1 Eigen3::Eigen)

install(TARGETS p01_offline_case1
  DESTINATION lib/${PROJECT_NAME}
)



# ============================================================
# P01 SHADOW CONTROLLER
# ============================================================

add_executable(p01_shadow_controller
  src/p01_shadow_controller.cpp
)

ament_target_dependencies(p01_shadow_controller
  rclcpp
  px4_msgs
)

target_include_directories(p01_shadow_controller PRIVATE include)

target_link_libraries(p01_shadow_controller Eigen3::Eigen)

install(TARGETS p01_shadow_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # The following line skips the linter which checks for copyrights.
  # Comment the line when a copyright and license is added to all source files.
  set(ament_cmake_copyright_FOUND TRUE)

  # The following line skips cpplint (only works in a git repo).
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files.
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

add_executable(p01_allocation_check
  src/p01_allocation_check.cpp
)
target_include_directories(p01_allocation_check PRIVATE include)
target_link_libraries(p01_allocation_check Eigen3::Eigen)

install(TARGETS p01_allocation_check
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# P01 OFFBOARD HOVER - DRY RUN
# ============================================================

# Tao chuong trinh tu file C++
add_executable(p01_offboard_hover
  src/p01_offboard_hover.cpp
)

# Khai bao thu vien ROS 2 va PX4
ament_target_dependencies(p01_offboard_hover
  rclcpp
  rcl_interfaces
  px4_msgs
)

# Tim cac file header cua project
target_include_directories(p01_offboard_hover PRIVATE
  include
)

# Lien ket thu vien Eigen
target_link_libraries(p01_offboard_hover
  Eigen3::Eigen
)

# Cho phep chay bang lenh ros2 run
install(TARGETS p01_offboard_hover
  DESTINATION lib/${PROJECT_NAME}
)
# Đăng ký package với hệ thống ament của ROS 2; thường đặt ở cuối file.
add_executable(test_x500_control src/test_x500_control.cpp)
target_include_directories(test_x500_control PRIVATE include)
target_link_libraries(test_x500_control Eigen3::Eigen)
add_executable(p01_x500_offline src/p01_x500_offline.cpp)
target_include_directories(p01_x500_offline PRIVATE include)
target_link_libraries(p01_x500_offline Eigen3::Eigen)
install(TARGETS test_x500_control p01_x500_offline DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  function(add_core_test name target)
    ament_add_test(${name}
      COMMAND $<TARGET_FILE:${target}> ${ARGN}
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO)
  endfunction()
  add_core_test(p01_controller test_p01_controller)
  add_core_test(x500_control test_x500_control)
  add_core_test(x500_hover_100hz p01_x500_offline x500_hover.csv hover 0.01)
  add_core_test(x500_case1_100hz p01_x500_offline x500_case1.csv case1 0.01)
  add_core_test(x500_case1_50hz p01_x500_offline x500_case1_50hz.csv case1 0.02)
  add_core_test(x500_case1_yaw p01_x500_offline x500_case1_yaw.csv case1_yaw 0.01)
  add_core_test(x500_yaw_recovery p01_x500_offline x500_yaw_recovery.csv yaw_recovery 0.01)
endif()
ament_package()
