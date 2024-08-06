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

find_package(Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(system_libs
    ${Eigen_LIBRARIES}
    ${OpenCV_LIBS}
)


add_executable(sensor_reader src/sensor_reader.cpp)
ament_target_dependencies(sensor_reader rclcpp std_msgs)
target_link_libraries(sensor_reader ${system_libs})

add_executable(imu_publisher src/imu_publisher.cpp)
ament_target_dependencies(imu_publisher rclcpp std_msgs sensor_msgs tf2 tf2_ros)
target_link_libraries(imu_publisher ${system_libs})

include_directories(
    include
    ${tf2_ros_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
)

install(TARGETS
  sensor_reader
  imu_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
