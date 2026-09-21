"""Live laser odometry. ROS-side parameters come from config/config.yaml; the SLAM
configuration path is a launch argument because it depends on the install layout."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("evergreenslam_ros")
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution([share, "configs", "evergreenslam.yaml"]),
            ),
            Node(
                package="evergreenslam_ros",
                executable="laser_odometry_node",
                name="laser_odometry",
                output="screen",
                parameters=[
                    PathJoinSubstitution([share, "config", "config.yaml"]),
                    {"config": config},
                ],
            ),
        ]
    )
