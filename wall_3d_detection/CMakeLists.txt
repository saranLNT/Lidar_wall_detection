cmake_minimum_required(VERSION 3.5)
project(wall_3d_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(cloud_accumulator
  src/cloud_accumulator_node.cpp
)

ament_target_dependencies(cloud_accumulator
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

target_link_libraries(cloud_accumulator ${PCL_LIBRARIES})

add_executable(laser_odometry
  src/laser_odometry_node.cpp
)

ament_target_dependencies(laser_odometry
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(laser_odometry ${PCL_LIBRARIES})

install(TARGETS
  cloud_accumulator
  laser_odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()