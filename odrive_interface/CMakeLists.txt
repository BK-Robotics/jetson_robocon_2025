cmake_minimum_required(VERSION 3.8)
project(odrive_interface)

# ────────────────────────────────────────────────────────────────
# 1. Compiler flags
# ────────────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ────────────────────────────────────────────────────────────────
# 2. Dependencies
# ────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(robot_interfaces REQUIRED)

# (nếu bạn sử dụng shooter_control hoặc package khác hãy thêm:)
# find_package(shooter_control REQUIRED)

# ────────────────────────────────────────────────────────────────
# 3. Build target
# ────────────────────────────────────────────────────────────────
# Liệt kê *toàn bộ* nguồn .cpp cần biên dịch
add_executable(odrive_interface_node
  src/odrive_interface.cpp
  src/can_comm.cpp
  src/odrive_motor.cpp
)

# Đưa đường dẫn include/ cho target
target_include_directories(odrive_interface_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link các dependency của ROS2
ament_target_dependencies(odrive_interface_node
  rclcpp
  robot_interfaces
  std_msgs
  # shooter_control
)

# ────────────────────────────────────────────────────────────────
# 4. Install
# ────────────────────────────────────────────────────────────────
install(TARGETS odrive_interface_node
        DESTINATION lib/${PROJECT_NAME})

# Cài đặt cả thư mục header để gói khác có thể dùng
install(DIRECTORY include/
        DESTINATION include)

# ────────────────────────────────────────────────────────────────
# 5. Tests (giữ nguyên nếu cần)
# ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
