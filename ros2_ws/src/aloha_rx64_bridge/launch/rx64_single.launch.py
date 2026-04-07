from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("device_name", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baudrate", default_value="57600"),
            DeclareLaunchArgument("servo_id", default_value="6"),
            DeclareLaunchArgument("joint_name", default_value="rx64_joint"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("moving_speed", default_value="100"),
            DeclareLaunchArgument("zero_position_raw", default_value="512"),
            Node(
                package="aloha_rx64_bridge",
                executable="rx64_single_node",
                name="rx64_single",
                output="screen",
                parameters=[
                    {
                        "device_name": LaunchConfiguration("device_name"),
                        "baudrate": LaunchConfiguration("baudrate"),
                        "servo_id": LaunchConfiguration("servo_id"),
                        "joint_name": LaunchConfiguration("joint_name"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "moving_speed": LaunchConfiguration("moving_speed"),
                        "zero_position_raw": LaunchConfiguration("zero_position_raw"),
                    }
                ],
            ),
        ]
    )
