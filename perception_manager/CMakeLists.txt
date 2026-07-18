cmake_minimum_required(VERSION 3.8)
project(perception_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(perception_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)

add_executable(camera_tf_broadcaster_node src/camera_tf_broadcaster.cpp)
ament_target_dependencies(camera_tf_broadcaster_node
  rclcpp geometry_msgs tf2 tf2_ros
)

add_executable(perception_manager_node src/perception_manager.cpp)
ament_target_dependencies(perception_manager_node
  rclcpp rclcpp_action action_msgs perception_interfaces
)

install(TARGETS
  camera_tf_broadcaster_node
  perception_manager_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()