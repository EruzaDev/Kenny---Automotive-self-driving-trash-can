"""Live sensor GUI. Closing RViz also stops the sensor drivers."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    repo = Path(__file__).resolve().parents[1]
    viewer = Node(package="rviz2", executable="rviz2", name="kenny_sensor_viewer",
                  arguments=["-d", str(repo / "configs/pi5-sensors.rviz")],
                  additional_env={"QT_QPA_PLATFORM": "xcb"}, output="screen")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(repo / "scripts/pi5_sensors.launch.py")),
                                 launch_arguments={"colored_cloud": "false"}.items()),
        ExecuteProcess(cmd=["/usr/bin/python3", str(repo / "scripts/pi5_depth_view.py")], output="screen"),
        viewer,
        RegisterEventHandler(OnProcessExit(target_action=viewer, on_exit=[
            EmitEvent(event=Shutdown(reason="Sensor viewer closed")),
        ])),
    ])
