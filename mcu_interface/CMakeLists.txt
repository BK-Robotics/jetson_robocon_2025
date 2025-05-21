cmake_minimum_required(VERSION 3.8)
project(mcu_interface)

# Build flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED) # Gói message bạn dùng

# Build executable
add_executable(uart_node src/MCU_Interface.cpp)
ament_target_dependencies(uart_node
  rclcpp
  std_msgs
  robot_interfaces
)

# Install executable
install(TARGETS
  uart_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
