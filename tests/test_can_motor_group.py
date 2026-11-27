#!/usr/bin/env python3
"""Group control regression tests; no ROS graph, CAN, or motors are used.

ROS2 port of tests/test_can_motor_group.py (devel/CMbenchTop ef24346).
"""

import runpy
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import Mock

MODULE = runpy.run_path(str(Path(__file__).resolve().parents[1] / "scripts" / "CANMotorGroup.py"))
CANMotorGroup = MODULE["CANMotorGroup"]


class CallbackServer:
    """Match Server's initial callback and non-reentrant request mutex."""

    def __init__(self, callback, stored_enable=True, stored_activity=None):
        self.callback = callback
        self.mutex = threading.Lock()
        config = {"enable_controller": stored_enable}
        if stored_activity is not None:
            config["activity"] = stored_activity
            config["starting_phase"] = 1
        self.config = callback(config, -1)

    def update_configuration(self, changes):
        with self.mutex:
            config = dict(self.config)
            config.update(changes)
            self.config = self.callback(config, 0)
            return dict(self.config)


class MotorGroupTests(unittest.TestCase):
    def setUp(self):
        self.group = CANMotorGroup.__new__(CANMotorGroup)
        # Built without Node.__init__, so give it a silent logger.
        self.group.get_logger = Mock()
        self.group._initial_config = True
        self.group._enabled = False
        self.group._activity_control = False
        self.group._activity = 0
        self.group._starting_phase = 0
        self.group._request = threading.local()
        self.ok = [("cm1", True, "ACK received"), ("cm2", True, "ACK received")]
        self.failed = [("cm1", True, "ACK received"), ("cm2", False, "ACK timeout")]
        self.group._call_all = Mock(return_value=self.ok)
        self.group.server = CallbackServer(self.group.reconfigure_callback)

    def configure(self, enabled):
        return self.group.server.update_configuration({"enable_controller": enabled})

    def test_stored_enable_does_not_start(self):
        self.assertFalse(self.group.server.config["enable_controller"])
        self.group._call_all.assert_not_called()

    def test_checkbox_enables_after_readiness(self):
        self.assertTrue(self.configure(True)["enable_controller"])
        self.assertEqual([call.args[0] for call in self.group._call_all.call_args_list], ["ready", "start"])

    def test_unchecking_disables_and_unchanged_gui_sends_nothing(self):
        self.configure(True)
        self.group._call_all.reset_mock()
        self.configure(True)
        self.group._call_all.assert_not_called()
        self.assertFalse(self.configure(False)["enable_controller"])
        self.group._call_all.assert_called_once_with("stop")

    def test_failed_readiness_does_not_send_enables(self):
        self.group._call_all.return_value = self.failed
        self.assertFalse(self.configure(True)["enable_controller"])
        self.group._call_all.assert_called_once_with("ready")

    def test_failed_start_resets_checkbox_and_attempts_disable(self):
        self.group._call_all.side_effect = [self.ok, self.failed, self.ok]
        self.assertFalse(self.configure(True)["enable_controller"])
        self.assertEqual([call.args[0] for call in self.group._call_all.call_args_list], ["ready", "start", "stop"])

    def test_services_update_checkbox(self):
        self.assertTrue(self.group.start_callback(None).success)
        self.assertTrue(self.group.server.config["enable_controller"])
        self.assertTrue(self.group.prepare_callback(None).success)
        self.assertFalse(self.group.server.config["enable_controller"])
        self.assertFalse(hasattr(self.group._request, "action"))

    def test_stop_service_retries_when_checkbox_is_already_false(self):
        self.group._call_all.side_effect = [self.failed, self.ok]
        self.assertFalse(self.group.stop_callback(None).success)
        self.assertTrue(self.group.stop_callback(None).success)
        self.assertEqual(self.group._call_all.call_count, 2)

    def test_service_failure_is_returned_to_caller(self):
        self.group._call_all.side_effect = [self.ok, self.failed, self.failed]
        result = self.group.start_callback(None)
        self.assertFalse(result.success)
        self.assertIn("disable attempt", result.message)
        self.assertFalse(self.group.server.config["enable_controller"])

    def test_gui_and_service_requests_are_serialized(self):
        ready_entered = threading.Event()
        release_ready = threading.Event()
        actions = []

        def call_all(action):
            actions.append(action)
            if action == "ready":
                ready_entered.set()
                self.assertTrue(release_ready.wait(2))
            return self.ok

        self.group._call_all.side_effect = call_all
        with ThreadPoolExecutor(max_workers=2) as executor:
            gui = executor.submit(self.configure, True)
            self.assertTrue(ready_entered.wait(2))
            stop = executor.submit(self.group.stop_callback, None)
            release_ready.set()
            self.assertTrue(gui.result(timeout=2)["enable_controller"])
            self.assertTrue(stop.result(timeout=2).success)
        self.assertEqual(actions, ["ready", "start", "stop"])
        self.assertFalse(self.group.server.config["enable_controller"])


class SharedActivityTests(MotorGroupTests):
    def setUp(self):
        super().setUp()
        self.group._activity_control = True
        self.group._initial_config = True
        self.group.server = CallbackServer(self.group.reconfigure_callback, stored_activity=4)

    def test_stored_activity_is_reset_without_commands(self):
        self.assertEqual(self.group.server.config["activity"], 0)
        self.group._call_all.assert_not_called()

    def test_checkbox_enables_after_readiness(self):
        result = self.group.server.update_configuration({"enable_controller": True, "activity": 3})
        self.assertTrue(result["enable_controller"])
        self.assertEqual(result["activity"], 3)
        self.assertEqual([call.args for call in self.group._call_all.call_args_list],
                         [("ready",), ("set_activity", 3, 0), ("start",)])

    def test_unchecking_disables_and_unchanged_gui_sends_nothing(self):
        super().test_unchecking_disables_and_unchanged_gui_sends_nothing()

    def test_failed_start_resets_checkbox_and_attempts_disable(self):
        self.group._call_all.side_effect = [self.ok, self.ok, self.failed, self.ok]
        self.assertFalse(self.configure(True)["enable_controller"])
        self.assertEqual([call.args[0] for call in self.group._call_all.call_args_list],
                         ["ready", "set_activity", "start", "stop"])

    def test_service_failure_is_returned_to_caller(self):
        self.group._call_all.side_effect = [self.ok, self.ok, self.failed, self.failed]
        result = self.group.start_callback(None)
        self.assertFalse(result.success)
        self.assertIn("disable attempt", result.message)

    def test_gui_and_service_requests_are_serialized(self):
        # The superclass test asserts the old controller's exact call sequence.
        self.group._activity_control = False
        super().test_gui_and_service_requests_are_serialized()

    def test_disabled_activity_selection_does_not_enable(self):
        result = self.group.server.update_configuration({"activity": 2})
        self.assertFalse(result["enable_controller"])
        self.assertEqual(result["activity"], 2)
        self.group._call_all.assert_called_once_with("set_activity", 2, 0)

    def test_running_activity_change_does_not_restart(self):
        self.configure(True)
        self.group._call_all.reset_mock()
        result = self.group.server.update_configuration({"activity": 4})
        self.assertTrue(result["enable_controller"])
        self.group._call_all.assert_called_once_with("set_activity", 4, 0)

    def test_failed_activity_change_disables_all_and_restores_selection(self):
        self.configure(True)
        self.group._call_all.reset_mock()
        self.group._call_all.side_effect = [self.failed, self.failed]
        result = self.group.server.update_configuration({"activity": 3})
        self.assertFalse(result["enable_controller"])
        self.assertEqual(result["activity"], 0)
        self.assertEqual([call.args for call in self.group._call_all.call_args_list],
                         [("set_activity", 3, 0), ("stop",)])
        self.assertFalse(self.group._request.response.success)
        self.assertIn("disable attempt", self.group._request.response.message)

    def test_failed_activity_prevents_enable_requests(self):
        self.group._call_all.side_effect = [self.ok, self.failed, self.ok]
        self.assertFalse(self.configure(True)["enable_controller"])
        self.assertEqual([call.args[0] for call in self.group._call_all.call_args_list],
                         ["ready", "set_activity", "stop"])

    def test_failed_readiness_on_start_retry_attempts_disable(self):
        self.configure(True)
        self.group._call_all.reset_mock()
        self.group._call_all.side_effect = [self.failed, self.ok]
        result = self.group.start_callback(None)
        self.assertFalse(result.success)
        self.assertFalse(self.group.server.config["enable_controller"])
        self.assertEqual([call.args[0] for call in self.group._call_all.call_args_list],
                         ["ready", "stop"])

    def test_combined_disable_and_activity_change_stops_first(self):
        self.configure(True)
        self.group._call_all.reset_mock()
        result = self.group.server.update_configuration({"enable_controller": False, "activity": 1})
        self.assertFalse(result["enable_controller"])
        self.assertEqual([call.args for call in self.group._call_all.call_args_list],
                         [("stop",), ("set_activity", 1, 0)])

    def test_prepare_realigns_shared_activity_without_enabling(self):
        self.group._activity = 2
        self.group.server.config["activity"] = 2
        self.assertTrue(self.group.prepare_callback(None).success)
        self.assertEqual([call.args for call in self.group._call_all.call_args_list],
                         [("prepare",), ("set_activity", 2, 0)])
        self.assertFalse(self.group.server.config["enable_controller"])

    def test_sts_change_while_running_is_refused_without_traffic(self):
        self.configure(True)
        self.group._call_all.reset_mock()
        result = self.group.server.update_configuration({"activity": 5})
        self.assertEqual(result["activity"], 0)
        self.assertTrue(result["enable_controller"])
        self.group._call_all.assert_not_called()
        self.assertFalse(self.group._request.response.success)

    def test_sts_and_phase_change_with_disable_stops_first(self):
        self.configure(True)
        self.group._call_all.reset_mock()
        result = self.group.server.update_configuration({"enable_controller": False, "activity": 5, "starting_phase": 1})
        self.assertEqual((result["activity"], result["starting_phase"]), (5, 1))
        self.assertEqual([call.args for call in self.group._call_all.call_args_list],
                         [("stop",), ("set_activity", 5, 1)])

    def test_running_starting_phase_change_is_refused(self):
        self.configure(True)
        self.group._call_all.reset_mock()
        result = self.group.server.update_configuration({"starting_phase": 1})
        self.assertEqual(result["starting_phase"], 0)
        self.group._call_all.assert_not_called()

    def test_explicit_phases_follow_selected_order_not_can_node_id(self):
        self.group.controller_nodes = ["cm8", "cm3"]
        self.group._call_controller = Mock(side_effect=lambda node, *args: (node, True, "ACK"))
        for activity in range(6):
            for phase in (0, 1):
                self.group._call_controller.reset_mock()
                CANMotorGroup._call_all(self.group, "set_activity", activity, phase)
                starts = {call.args[0]: call.args[3] for call in self.group._call_controller.call_args_list}
                self.assertEqual(starts, {"cm8": phase, "cm3": phase if activity == 5 else 1 - phase})
        self.group.controller_nodes = ["cm3"]
        self.group._call_controller.reset_mock()
        CANMotorGroup._call_all(self.group, "set_activity", 5, 1)
        self.group._call_controller.assert_called_once_with("cm3", "set_activity", 5, 1)


if __name__ == "__main__":
    unittest.main()
