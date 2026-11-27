#!/usr/bin/env python3
"""Control selected CAN motors from rqt_reconfigure or ROS services.

ROS2 port of CANMotorGroup.py from micropython_logger devel/CMbenchTop (ef24346).
The group logic is unchanged; dynamic_reconfigure is replaced by
micropython_logger.reconfigure.ReconfigureServer.
"""

import threading
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from micropython_logger.ctrl_configs import GuiCANMotorGroup, GuiCANMotorGroupActivity
from micropython_logger.reconfigure import ReconfigureServer
from micropython_logger.srv import SetCANActivity

STS_ACTIVITY = 5
ACTIONS = ("ready", "prepare", "start", "stop", "set_activity")


class CANMotorGroup(Node):
    def __init__(self):
        super().__init__("can_motor_group")
        nodes = list(self.declare_parameter("controller_nodes", [""]).value)
        nodes = [name for name in nodes if name]
        if not nodes:
            raise ValueError("controller_nodes must be a nonempty list of ROS node names")
        self.controller_nodes = [self.resolve_service_name(name) for name in nodes]
        if len(set(self.controller_nodes)) != len(self.controller_nodes):
            raise ValueError("controller_nodes must not contain duplicates")
        can_nodes = list(self.declare_parameter("can_nodes", [-1]).value)
        if len(can_nodes) != len(self.controller_nodes):
            raise ValueError("can_nodes must list one CAN ID per selected controller")
        can_nodes = [int(node) for node in can_nodes]
        if len(set(can_nodes)) != len(can_nodes) or any(node < 0 for node in can_nodes):
            raise ValueError("Selected CAN node IDs must be distinct and nonnegative")
        self.service_wait_timeout_s = float(self.declare_parameter("service_wait_timeout_s", 10.0).value)
        if self.service_wait_timeout_s <= 0:
            raise ValueError("service_wait_timeout_s must be positive")
        # rospy service proxies had no call timeout; bound it so a dead motor node cannot hang the group.
        self.service_call_timeout_s = float(self.declare_parameter("service_call_timeout_s", 30.0).value)
        self._initial_config = True
        self._enabled = False
        self._activity_control = bool(self.declare_parameter("activity_control", False).value)
        self._activity = 0
        self._starting_phase = 0
        self._request = threading.local()

        # Board ACKs arrive on other executor threads while a group request blocks.
        client_group = ReentrantCallbackGroup()
        self._controller_clients = {
            (node, action): self.create_client(
                SetCANActivity if action == "set_activity" else Trigger,
                node.rstrip("/") + "/" + action,
                callback_group=client_group,
            )
            for node in self.controller_nodes for action in ACTIONS
        }

        config_type = GuiCANMotorGroupActivity if self._activity_control else GuiCANMotorGroup
        self.server = ReconfigureServer(self, config_type, self.reconfigure_callback)
        self.prepare_service = self.create_service(Trigger, "~/prepare", self.prepare_callback)
        self.start_service = self.create_service(Trigger, "~/start", self.start_callback)
        self.stop_service = self.create_service(Trigger, "~/stop", self.stop_callback)
        self.get_logger().info(
            "CAN motor group: {}; toggle enable_controller in rqt_reconfigure "
            "or use ~/start / ~/stop".format(self.controller_nodes)
        )

    def _call_controller(self, node, action, activity=None, starting_state=-1):
        client = self._controller_clients[(node, action)]
        try:
            if not client.wait_for_service(timeout_sec=self.service_wait_timeout_s):
                return node, False, "service {} not available".format(client.srv_name)
            if action == "set_activity":
                request = SetCANActivity.Request(activity=activity, starting_state=starting_state)
            else:
                request = Trigger.Request()
            response = client.call(request, timeout_sec=self.service_call_timeout_s)
            return node, response.success, response.message
        except Exception as error:
            return node, False, str(error) or type(error).__name__

    def _call_all(self, action, activity=None, starting_phase=None):
        # Dispatch both enable requests before waiting for either board's ACK.
        with ThreadPoolExecutor(max_workers=len(self.controller_nodes)) as executor:
            futures = []
            for index, node in enumerate(self.controller_nodes):
                phase = -1
                if action == "set_activity":
                    first_phase = self._starting_phase if starting_phase is None else starting_phase
                    phase = first_phase if activity == STS_ACTIVITY else (first_phase + index) % 2
                futures.append(executor.submit(self._call_controller, node, action, activity, phase))
            return [future.result() for future in futures]

    @staticmethod
    def _response(action, results):
        success = all(result[1] for result in results)
        details = "; ".join("{}: {}".format(node, message) for node, _, message in results)
        return Trigger.Response(success=success, message="{}: {}".format(action, details))

    def _disable_after_failure(self, action, results):
        # The request may have been applied even when its ACK was lost.
        response = self._response(action, results)
        self._enabled = False
        stopped = self._call_all("stop")
        response.message += "; disable attempt: " + self._response("stop", stopped).message
        return response

    def _set_activity(self, activity, starting_phase=None):
        phase = self._starting_phase if starting_phase is None else starting_phase
        results = self._call_all("set_activity", activity, phase)
        if not all(result[1] for result in results):
            return self._disable_after_failure("shared activity not confirmed", results)
        self._activity = activity
        self._starting_phase = phase
        return self._response("shared activity accepted (local phase boundaries may differ)", results)

    def _start(self, activity=None, starting_phase=None):
        ready = self._call_all("ready")
        if not all(result[1] for result in ready):
            if self._enabled:
                return self._disable_after_failure("start refused; existing group state unconfirmed", ready)
            return self._response("start refused; no enable requests sent", ready)
        if self._activity_control:
            # Re-send the shared selection before every start, even if GUI
            # editors or profiles changed since the last group operation.
            selected = self._set_activity(self._activity if activity is None else activity, starting_phase)
            if not selected.success:
                return selected
        started = self._call_all("start")
        if all(result[1] for result in started):
            return self._response("start confirmed (approximate timing)", started)
        return self._disable_after_failure("start failed", started)

    def reconfigure_callback(self, config, level):
        if self._initial_config:
            # Ignore a stored enable value on startup, without sending commands.
            self._initial_config = False
            config["enable_controller"] = False
            if self._activity_control:
                config["activity"] = self._activity
                config["starting_phase"] = self._starting_phase
            return config

        enabled = bool(config["enable_controller"])
        activity = int(config["activity"]) if self._activity_control else None
        activity_changed = self._activity_control and activity != self._activity
        phase = int(config["starting_phase"]) if self._activity_control else None
        phase_changed = self._activity_control and phase != self._starting_phase
        if self._activity_control and enabled and self._enabled and (
            phase_changed or ((activity == STS_ACTIVITY) != (self._activity == STS_ACTIVITY))
        ):
            config["activity"] = self._activity
            config["starting_phase"] = self._starting_phase
            self._request.response = Trigger.Response(success=False, message="Disable the group before changing starting phase or switching into/out of StS")
            self.get_logger().warning(self._request.response.message)
            return config
        action = getattr(self._request, "action", None)
        if action is None:
            if enabled == self._enabled and not activity_changed and not phase_changed:
                return config
            if not enabled and self._enabled:
                action = "stop"
            elif enabled and not self._enabled:
                action = "start"
            else:
                action = "activity"

        if action == "start":
            response = self._start(activity, phase)
            self._enabled = response.success
        elif action == "activity":
            response = self._set_activity(activity, phase)
        else:
            response = self._response(
                "{} (disable requested)".format(action), self._call_all(action)
            )
            self._enabled = False
            if response.success and self._activity_control and (activity_changed or phase_changed or action == "prepare"):
                response = self._set_activity(activity, phase)

        config["enable_controller"] = self._enabled
        if self._activity_control:
            config["activity"] = self._activity
            config["starting_phase"] = self._starting_phase
        self._request.response = response
        if response.success:
            self.get_logger().info("CAN motor group: {}".format(response.message))
        else:
            self.get_logger().warning("CAN motor group: {}".format(response.message))
        return config

    def _service_action(self, action):
        # The server serializes GUI and service actions. Thread-local metadata
        # forces service retries without changing another caller's request.
        self._request.action = action
        try:
            self.server.update_configuration({"enable_controller": action == "start"})
            return self._request.response
        except Exception as error:
            # An exception in a ROS2 service callback would stop the executor.
            return Trigger.Response(success=False, message=str(error))
        finally:
            del self._request.action

    def prepare_callback(self, request, response=None):
        return self._service_action("prepare")

    def start_callback(self, request, response=None):
        return self._service_action("start")

    def stop_callback(self, request, response=None):
        return self._service_action("stop")


def main(args=None):
    rclpy.init(args=args)
    node = CANMotorGroup()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
