"""Sensor-only Pi bringup; no motor commands are published."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    repo = Path(__file__).resolve().parents[1]
    camera_launch = Path(get_package_share_directory("astra_camera")) / "launch/astra_pro.launch.xml"
    return LaunchDescription([
        DeclareLaunchArgument("colored_cloud", default_value="false",
                              description="Enable registered depth and the RGB-colored cloud"),
        Node(package="ydlidar_ros2_driver", executable="ydlidar_ros2_driver_node",
             name="ydlidar_ros2_driver_node", output="screen",
             parameters=[str(repo / "configs/pi5-lidar.yaml")]),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(camera_launch)),
                                 launch_arguments={
                                     "enable_ir": "false",
                                     "depth_registration": LaunchConfiguration("colored_cloud"),
                                     "enable_colored_point_cloud": LaunchConfiguration("colored_cloud"),
                                 }.items()),
    ])
