"""CubeMars CAN bench: canIP2ROS receiver/relay, one ROS2canIP per motor, group control.

ROS2 port of cubemars_can.launch (devel/CMbenchTop ef24346).

    ros2 launch micropython_logger cubemars_can.launch.py
    ros2 launch micropython_logger cubemars_can.launch.py motor2:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

ARGS = {
    "motor1": "true",
    "motor2": "true",
    "node1_id": "1",
    "node2_id": "2",
    "robot": "CubeMars",
    "ctrl": "SMWalking",
    "node1_sm_params_file": "",
    "node2_sm_params_file": "",
    "start_receiver": "true",
    "canip_host": "192.168.20.2",
    "canip_port": "10002",
    "listen_port": "12346",
    "relay_host": "127.0.0.1",
    "relay_port": "12347",
    "command_ack_timeout_s": "5.0",
    "relay_startup_timeout_s": "10.0",
    "service_wait_timeout_s": "10.0",
    "starting_state": "-1.0",
    "print_stats": "true",
}


def _bool(text):
    return text.strip().lower() in ("1", "true", "yes", "on")


def _nodes(context):
    arg = {name: context.launch_configurations[name] for name in ARGS}
    motors = [
        (namespace, int(arg[node_key]), arg[params_key])
        for namespace, enabled_key, node_key, params_key in (
            ("cm1", "motor1", "node1_id", "node1_sm_params_file"),
            ("cm2", "motor2", "node2_id", "node2_sm_params_file"),
        )
        if _bool(arg[enabled_key])
    ]
    if not motors:
        raise RuntimeError("Select at least one motor (motor1:=true or motor2:=true)")
    can_nodes = [node_id for _, node_id, _ in motors]
    actions = []

    if _bool(arg["start_receiver"]):
        actions.append(Node(
            package="micropython_logger",
            executable="canIP2ROS.py",
            name="canIP2ROS",
            output="screen",
            parameters=[{
                "robot": arg["robot"],
                "can_nodes": can_nodes,
                "canip_host": arg["canip_host"],
                "canip_port": int(arg["canip_port"]),
                "listen_port": int(arg["listen_port"]),
                "command_relay_host": arg["relay_host"],
                "command_relay_port": int(arg["relay_port"]),
                "print_stats": _bool(arg["print_stats"]),
            }],
            on_exit=Shutdown(),
        ))

    motor_launch = os.path.join(
        get_package_share_directory("micropython_logger"), "launch", "cubemars_can_motor.launch.py"
    )
    for namespace, node_id, sm_params_file in motors:
        actions.append(GroupAction([
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(motor_launch),
                launch_arguments={
                    "node_id": str(node_id),
                    "sm_params_file": sm_params_file,
                    "robot": arg["robot"],
                    "ctrl": arg["ctrl"],
                    "relay_host": arg["relay_host"],
                    "relay_port": arg["relay_port"],
                    "command_ack_timeout_s": arg["command_ack_timeout_s"],
                    "relay_startup_timeout_s": arg["relay_startup_timeout_s"],
                    "starting_state": arg["starting_state"],
                }.items(),
            ),
        ]))

    actions.append(Node(
        package="micropython_logger",
        executable="CANMotorGroup.py",
        name="can_motor_group",
        output="screen",
        parameters=[{
            "can_nodes": can_nodes,
            "controller_nodes": [
                "{}/ROS2canIP_can_node_{}".format(namespace, node_id) for namespace, node_id, _ in motors
            ],
            "service_wait_timeout_s": float(arg["service_wait_timeout_s"]),
            "activity_control": arg["ctrl"] == "SMMultipleAct",
        }],
        on_exit=Shutdown(),
    ))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, default in ARGS.items()]
        + [OpaqueFunction(function=_nodes)]
    )
