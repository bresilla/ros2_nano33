import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='ros2_nano33',
        executable='sensor_reader',
        output="screen",
        emulate_tty=True,
    ))

    ld.add_action(Node(
        package='ros2_nano33',
        executable='imu_publisher',
        output="screen",
        emulate_tty=True,
    ))




    return ld
