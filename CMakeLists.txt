################################################################################
# Set minimum required version of CMake, project name and compile options
################################################################################

cmake_minimum_required(VERSION 3.5)

project(sllidar_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()


################################################################################
# SLLIDAR SDK
################################################################################

set(SLLIDAR_SDK_PATH "./sdk/")

FILE(GLOB SLLIDAR_SDK_SRC
  "${SLLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${SLLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
)


################################################################################
# Find ROS 2 dependencies
################################################################################

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(std_srvs REQUIRED)

find_package(visualization_msgs REQUIRED)


################################################################################
# Include directories
################################################################################

include_directories(
  ${SLLIDAR_SDK_PATH}/include
  ${SLLIDAR_SDK_PATH}/src
  ${Boost_INCLUDE_DIRS}
)


################################################################################
# 1. SLLIDAR NODE
################################################################################

add_executable(
  sllidar_node
  src/sllidar_node.cpp
  ${SLLIDAR_SDK_SRC}
)

ament_target_dependencies(
  sllidar_node
  rclcpp
  std_srvs
  sensor_msgs
)


################################################################################
# 2. SLLIDAR CLIENT
################################################################################

add_executable(
  sllidar_client
  src/sllidar_client.cpp
)

ament_target_dependencies(
  sllidar_client
  rclcpp
  std_srvs
  sensor_msgs
)


################################################################################
# 3. LASER SCAN BOX FILTER
#
# Input:
#   /wall_scan
#
# Output:
#   /wall_scan_filtered
#
# Visualization:
#   /laser_filter_box
################################################################################

add_executable(
  laser_scan_box_filter
  src/laser_scan_box_filter.cpp
)

ament_target_dependencies(
  laser_scan_box_filter
  rclcpp
  sensor_msgs
  visualization_msgs
)


################################################################################
# 4. OBSTACLE DETECTOR
#
# Input:
#   /wall_scan_filtered
#
# Output:
#   /obstacle_detection_markers
################################################################################

add_executable(
  obstacle_detector
  src/obstacle_detector.cpp
)

ament_target_dependencies(
  obstacle_detector
  rclcpp
  sensor_msgs
  visualization_msgs
  std_srvs
)


################################################################################
# Install launch and RViz directories
################################################################################

install(
  DIRECTORY
    launch
    rviz
  DESTINATION share/${PROJECT_NAME}
)


################################################################################
# Install executables
################################################################################

install(
  TARGETS
    sllidar_node
    sllidar_client
    laser_scan_box_filter
    obstacle_detector
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


################################################################################
# Export ROS 2 dependencies
################################################################################

ament_export_dependencies(
  rclcpp
  std_srvs
  sensor_msgs
  visualization_msgs
)


################################################################################
# Export include directory
################################################################################

ament_export_include_directories(include)


################################################################################
# Ament package
################################################################################

ament_package()