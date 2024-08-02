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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
    include
    ${tf2_ros_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
)

add_executable(sensor_reader_node src/sensor_reader_node.cpp)
ament_target_dependencies(sensor_reader_node rclcpp std_msgs)

add_executable(imu_publisher_euler src/imu_publisher_euler.cpp)
ament_target_dependencies(imu_publisher_euler rclcpp std_msgs sensor_msgs tf2 tf2_ros)

add_executable(imu_publisher_raw src/imu_publisher_raw.cpp)
ament_target_dependencies(imu_publisher_raw rclcpp std_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS
  sensor_reader_node
  imu_publisher_euler
  imu_publisher_raw
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
