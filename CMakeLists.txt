cmake_minimum_required(VERSION 3.8)
project(uav_research)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# Build odometry_listener node
add_executable(odometry_listener src/odometry_listener.cpp)

# Link ROS 2 and PX4 dependencies
ament_target_dependencies(odometry_listener
  rclcpp
  px4_msgs
)

# Install executable for ros2 run
install(TARGETS odometry_listener
  DESTINATION lib/${PROJECT_NAME}
)



# Build offboard_control node
#Yêu cầu CMake tạo chương trình thực thi tên offboard_control từ file src/offboard_control.cpp.
add_executable(offboard_control src/offboard_control.cpp) 

# Link ROS 2 and PX4 dependencies
#Khai báo các thư viện và package mà chương trình cần sử dụng
ament_target_dependencies(offboard_control
  rclcpp
  px4_msgs
)

# Install executable for ros2 run
#Yêu cầu cài đặt chương trình đã biên dịch vào thư mục thích hợp để sau này có thể chạy bằng:ros2 run uav_research offboard_control
install(TARGETS offboard_control
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
