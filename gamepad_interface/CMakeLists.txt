cmake_minimum_required(VERSION 3.8)
project(gamepad_interface)

###############################################################################
# 0. Chung
###############################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# 1. Phụ thuộc
###############################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

include_directories(include)

###############################################################################
# 2. Thư viện driver  (/dev/input/eventNN)
###############################################################################
add_library(gamepad_driver
  src/gamepad.cpp
)
target_compile_features(gamepad_driver PUBLIC cxx_std_11)

###############################################################################
# 3. Thư viện mapper  (Manual / Semi-Auto logic)
###############################################################################
add_library(robot_input_mapper
  src/mapper.cpp
)
# mapper cần driver
target_link_libraries(robot_input_mapper
  gamepad_driver
)
ament_target_dependencies(robot_input_mapper
  robot_interfaces
)
target_compile_features(robot_input_mapper PUBLIC cxx_std_11)

###############################################################################
# 4. Node ROS2
###############################################################################
add_executable(gamepad_node
  src/gamepad_node.cpp
)
# liên kết node với 2 thư viện
target_link_libraries(gamepad_node
  robot_input_mapper
  gamepad_driver
)
ament_target_dependencies(gamepad_node
  rclcpp
  robot_interfaces
)
target_compile_features(gamepad_node PUBLIC cxx_std_11)

###############################################################################
# 5. Cài đặt
###############################################################################
install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
  gamepad_driver
  robot_input_mapper
  gamepad_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

###############################################################################
ament_package()
