cmake_minimum_required(VERSION 3.5)
project(wall_scan_to_pointcloud)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(wall_scan_to_pointcloud_node
  src/wall_scan_to_pointcloud_node.cpp)

ament_target_dependencies(wall_scan_to_pointcloud_node
  rclcpp
  sensor_msgs)

install(TARGETS
  wall_scan_to_pointcloud_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
