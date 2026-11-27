"""CubeMars CAN bench with the two-state multi-activity controller (SMMultipleAct).

ROS2 port of cubemars_can_multi_activity.launch (devel/CMbenchTop ef24346).

    ros2 launch micropython_logger cubemars_can_multi_activity.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ARGS = {
    "motor1": "true",
    "motor2": "true",
    "node1_id": "1",
    "node2_id": "2",
    "start_receiver": "true",
    "canip_host": "192.168.20.2",
    "canip_port": "10002",
    "listen_port": "12346",
    "relay_port": "12347",
    "command_ack_timeout_s": "5.0",
    "node1_sm_params_file": "",
    "node2_sm_params_file": "",
}


def generate_launch_description():
    can_launch = os.path.join(
        get_package_share_directory("micropython_logger"), "launch", "cubemars_can.launch.py"
    )
    arguments = {name: LaunchConfiguration(name) for name in ARGS}
    arguments["ctrl"] = "SMMultipleAct"
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, default in ARGS.items()]
        + [IncludeLaunchDescription(PythonLaunchDescriptionSource(can_launch), launch_arguments=arguments.items())]
    )
