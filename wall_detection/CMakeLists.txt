cmake_minimum_required(VERSION 3.5)

project(wall_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(
  laser_scan_box_filter
  src/laser_scan_box_filter.cpp)

ament_target_dependencies(
  laser_scan_box_filter
  rclcpp
  sensor_msgs
  visualization_msgs)

add_executable(
  obstacle_detector
  src/obstacle_detector.cpp)

ament_target_dependencies(
  obstacle_detector
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs)

install(
  TARGETS
    laser_scan_box_filter
    obstacle_detector
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
