cmake_minimum_required(VERSION 3.8)
project(ros2_nano33)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#generate compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sensor_reader_node src/sensor_reader_node.cpp)
ament_target_dependencies(sensor_reader_node rclcpp std_msgs)

add_executable(imu_publisher_node src/imu_publisher_node.cpp)
ament_target_dependencies(imu_publisher_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  sensor_reader_node
  imu_publisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
