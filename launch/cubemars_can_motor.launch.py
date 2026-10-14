"""One ROS2canIP command node for a single CubeMars CAN board.

ROS2 port of cubemars_can_motor.launch (devel/CMbenchTop ef24346). Include it
inside a namespace (cubemars_can.launch.py uses cm1 / cm2).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch_ros.actions import Node

ARGS = {
    "node_id": None,
    "robot": "CubeMars",
    "ctrl": "SMWalking",
    "relay_host": "127.0.0.1",
    "relay_port": "12347",
    "command_ack_timeout_s": "5.0",
    "relay_startup_timeout_s": "10.0",
    "starting_state": "-1.0",
    "sm_params_file": "",
}


def _nodes(context):
    arg = {name: context.launch_configurations[name] for name in ARGS}
    node_id = int(arg["node_id"])
    parameters = {
        "robot": arg["robot"],
        "ctrl": arg["ctrl"],
        "can_target_node": node_id,
        "use_local_relay": True,
        "relay_host": arg["relay_host"],
        "relay_port": int(arg["relay_port"]),
        "relay_startup_timeout_s": float(arg["relay_startup_timeout_s"]),
        "command_ack_enabled": True,
        "command_ack_timeout_s": float(arg["command_ack_timeout_s"]),
        "inter_frame_delay_s": 0.002,
        "starting_state": float(arg["starting_state"]),
        "start_disabled": True,
        "enable_controller": False,
    }
    if arg["sm_params_file"]:
        parameters["sm_params_file"] = arg["sm_params_file"]
    return [Node(
        package="micropython_logger",
        executable="ROS2canIP.py",
        name="ROS2canIP_can_node_{}".format(node_id),
        output="screen",
        parameters=[parameters],
        on_exit=Shutdown(),
    )]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name) if default is None else DeclareLaunchArgument(name, default_value=default)
         for name, default in ARGS.items()]
        + [OpaqueFunction(function=_nodes)]
    )
