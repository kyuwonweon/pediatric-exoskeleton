#!/usr/bin/env python3
"""CubeMars profile, persistence and ROS callback tests without hardware.

ROS2 port of tests/test_can_sm_profiles.py (devel/CMbenchTop ef24346).
"""

import copy
import runpy
import struct
import tempfile
import threading
from types import SimpleNamespace
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
from micropython_logger.can_sm_profiles import CANSMProfiles, ACTIVITIES, PHASES, validate_activities


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.profiles = CANSMProfiles(str(ROOT / "cubemars_sm_params/default.yaml"), self.directory.name, 1)

    def test_rows_and_activities_are_independent(self):
        config = {}
        self.profiles.display(config, "OW", "Extension")
        config["q_eq"] = 70
        self.profiles.update(config, "OW", "Extension")
        self.assertEqual(self.profiles.wire_values("OW")[0], 70)
        self.assertEqual(self.profiles.wire_values("OW")[5], 120)
        self.assertEqual(self.profiles.wire_values("RampAsc")[0], 60)

    def test_round_trip_saves_every_activity(self):
        self.profiles.activities["StS"]["phases"]["Flexion"]["q_eq"] = 130
        self.profiles.activities["RampDesc"]["phases"]["Flexion"]["q_eq"] = 115
        filename = self.profiles.save("bench_test")
        self.profiles.activities["RampDesc"]["phases"]["Flexion"]["q_eq"] = 120
        self.profiles.load("bench_test")
        self.assertEqual(self.profiles.activities["RampDesc"]["phases"]["Flexion"]["q_eq"], 115)
        self.assertTrue(Path(filename).is_file())
        self.assertEqual(self.profiles.wire_values("StS")[5], 130)
        self.assertEqual(yaml.safe_load(Path(filename).read_text())["schema_version"], 2)
        self.assertFalse(self.profiles.migrated_legacy)

    def test_legacy_snapshot_migrates_without_overwriting_existing_profiles(self):
        activities = copy.deepcopy(self.profiles.activities)
        del activities["StS"]
        activities["OW"]["phases"]["Extension"]["q_eq"] = 75
        document = dict(schema_version=1, controller="CubeMarsSMMultipleAct", can_node_id=1, activities=activities)
        filename = Path(self.profiles.filename("legacy"))
        filename.write_text(yaml.safe_dump(document))
        before = filename.read_bytes()
        self.profiles.load("legacy")
        self.assertTrue(self.profiles.migrated_legacy)
        self.assertEqual({key: self.profiles.activities[key] for key in ACTIVITIES[:-1]}, activities)
        self.assertEqual(self.profiles.wire_values("StS")[0], 60)
        self.assertEqual(filename.read_bytes(), before)
        self.profiles.save("upgraded")
        self.assertEqual(yaml.safe_load(Path(self.profiles.filename("upgraded")).read_text())["schema_version"], 2)

    def test_invalid_profiles_do_not_replace_cache(self):
        before = copy.deepcopy(self.profiles.activities)
        for value in (float("nan"), float("inf"), -1, "100", True):
            candidate = copy.deepcopy(before)
            candidate["OW"]["phases"]["Flexion"]["q_eq"] = value
            with self.assertRaises(ValueError):
                validate_activities(candidate)
        self.assertEqual(self.profiles.activities, before)

    def test_angles_and_missing_phases_rejected(self):
        candidate = copy.deepcopy(self.profiles.activities)
        candidate["OW"]["phases"]["Extension"]["q_eq"] = 90
        candidate["OW"]["phases"]["Flexion"]["q_eq"] = 90
        with self.assertRaises(ValueError):
            validate_activities(candidate)
        del candidate["OW"]["phases"]["Extension"]
        with self.assertRaises(ValueError):
            validate_activities(candidate)

    def test_bad_filename_and_other_motor_rejected(self):
        for name in ("", "../node_2/test", "name.yaml", "with space"):
            with self.assertRaises(ValueError):
                self.profiles.filename(name)
        self.profiles.save("node1")
        other = CANSMProfiles(str(ROOT / "cubemars_sm_params/default.yaml"), self.directory.name, 2)
        with self.assertRaises(ValueError):
            other.load("node1")

    def test_malformed_yaml_and_fox_files_rejected_atomically(self):
        before = copy.deepcopy(self.profiles.activities)
        for contents in ("bad: [", "OW: {phases: {EarlyStance: {q_eq: 0}}}"):
            Path(self.profiles.filename("bad")).write_text(contents)
            with self.assertRaises(ValueError):
                self.profiles.load("bad")
            self.assertEqual(self.profiles.activities, before)


class CallbackTests(unittest.TestCase):
    def setUp(self):
        ProfileTests.setUp(self)
        module = runpy.run_path(str(ROOT / "scripts/ROS2canIP.py"))
        self.node = module["ROS2canIP"].__new__(module["ROS2canIP"])
        self.node.sm_profiles = self.profiles
        self.node._sm_activity, self.node._sm_phase = "OW", "Extension"
        self.node._sm_requested_activity = "OW"
        self.node._sm_enable_requested = False
        self.node._sm_file_result = threading.local()
        self.node._sm_control_request = threading.local()
        self.node._command_result = threading.local()
        self.node._closed = False
        self.node.can_target_node = 1
        self.node.ctrl = "SMMultipleAct"
        self.node.ctrl_msg = np.zeros(20, dtype=np.float32)
        self.node.starting_state = -1.0
        self.node.send_command_sample = Mock(return_value=True)
        def send_sample():
            result = self.node.send_command_sample.return_value
            self.node._command_result.success = result
            return result
        self.node.send_command_sample.side_effect = send_sample
        self.node.server = Mock()
        self.node.server.update_configuration.side_effect = lambda changes: self.update(**changes)
        # Built without Node.__init__, so silence the rospy-style log helpers.
        for name in ("_logwarn", "_loginfo", "_logerr"):
            setattr(self.node, name, Mock())
        self.config = {
            "enable_controller": False, "activity": 0, "phase": 0,
            "save_now": False, "load_now": False, "save_name": "", "load_name": "",
        }
        self.node._multi_activity_callback(self.config, True)
        self.node.send_command_sample.reset_mock()

    def update(self, **changes):
        self.config.update(changes)
        return self.node._multi_activity_callback(self.config, False)

    def test_phase_browsing_is_gui_only(self):
        self.update(phase=1)
        self.assertEqual(self.config["q_eq"], 120)
        self.node.send_command_sample.assert_not_called()

    def test_editor_activity_change_does_not_request_motion(self):
        self.update(activity=3)
        self.assertEqual(self.config["requested_activity"], "OW")
        self.node.send_command_sample.assert_not_called()

    def test_service_requests_full_profile_without_changing_editor(self):
        self.update(activity=1, phase=1)
        result = self.node.set_activity_callback(SimpleNamespace(activity=3, starting_state=-1))
        self.assertTrue(result.success)
        self.assertEqual(list(self.node.ctrl_msg[:3]), [0, -1, 3])
        self.assertEqual(list(self.node.ctrl_msg[3:15]), list(np.asarray(self.profiles.wire_values("RampAsc"), dtype=np.float32)))
        self.assertTrue(all(self.node.ctrl_msg[15:] == 0))
        self.assertEqual(self.config["activity"], 1)
        self.assertEqual(self.config["phase"], 1)
        self.assertEqual(self.config["requested_activity"], "RampAsc")

    def test_other_profile_edits_are_cached_without_can_traffic(self):
        self.update(activity=2)
        self.update(q_eq=70)
        self.node.send_command_sample.assert_not_called()
        self.node.set_activity_callback(SimpleNamespace(activity=2, starting_state=-1))
        self.assertEqual(self.node.ctrl_msg[3], 70)

    def test_enable_uses_requested_profile_not_editor_profile(self):
        self.update(activity=3)
        self.update(q_eq=75)
        self.update(enable_controller=True)
        self.assertEqual(self.node.ctrl_msg[2], 0)
        self.assertEqual(self.node.ctrl_msg[3], 60)

    def test_requested_activity_status_cannot_command_motion(self):
        self.update(requested_activity="RampDesc")
        self.assertEqual(self.config["requested_activity"], "OW")
        self.node.send_command_sample.assert_not_called()

    def test_activity_service_preserves_enabled_state_and_reports_missing_ack(self):
        self.update(enable_controller=True)
        self.node.send_command_sample.return_value = False
        response = self.node.set_activity_callback(SimpleNamespace(activity=4, starting_state=-1))
        self.assertFalse(response.success)
        self.assertEqual(self.node.ctrl_msg[0], 1)
        self.assertEqual(self.config["requested_activity"], "RampDesc")
        self.assertFalse(hasattr(self.node._sm_control_request, "activity"))

    def test_invalid_activity_service_request_is_rejected(self):
        self.assertFalse(self.node.set_activity_callback(SimpleNamespace(activity=6, starting_state=-1)).success)
        self.node.send_command_sample.assert_not_called()

    def test_sts_service_sets_explicit_start_without_enabling(self):
        result = self.node.set_activity_callback(SimpleNamespace(activity=5, starting_state=1))
        self.assertTrue(result.success)
        self.assertEqual(list(self.node.ctrl_msg[:3]), [0, 1, 5])
        self.update(enable_controller=True)
        self.assertEqual(list(self.node.ctrl_msg[:3]), [1, 1, 5])

    def test_sts_relation_change_and_phase_change_are_rejected_while_running(self):
        self.node.set_activity_callback(SimpleNamespace(activity=0, starting_state=0))
        self.update(enable_controller=True)
        self.node.send_command_sample.reset_mock()
        for activity, start in ((5, 0), (0, 1), (0, 2)):
            result = self.node.set_activity_callback(SimpleNamespace(activity=activity, starting_state=start))
            self.assertFalse(result.success)
        self.node.send_command_sample.assert_not_called()
        self.assertEqual(self.config["requested_activity"], "OW")
        self.assertEqual(self.node.starting_state, 0)

    def test_indexed_wire_order_activity_precedes_start_and_checksum_is_logical(self):
        self.node.seq, self.node.command_count, self.node.tx_base_can_id = 0, 25, 0x320
        values = np.arange(25, dtype=np.float32)
        values[0], values[2], values[3] = 9, -1, 5
        frames = self.node._pack_command_sample(values)
        self.assertEqual([frame[4] for frame in frames[:5]], [0, 1, 3, 2, 4])
        decoded = {frame[4]: struct.unpack("<f", frame[7:11])[0] for frame in frames}
        self.assertEqual([decoded[i] for i in range(25)], list(values))
        self.assertEqual(self.node._command_checksum(values), self.node._command_checksum([decoded[i] for i in range(25)]))

    def test_stop_service_resends_even_when_already_disabled(self):
        self.assertTrue(self.node.stop_callback(None).success)
        self.assertTrue(self.node.stop_callback(None).success)
        self.assertEqual(self.node.send_command_sample.call_count, 2)
        self.assertEqual(self.node.ctrl_msg[0], 0)

    def test_running_edit_and_load_are_refused_without_wire_traffic(self):
        self.update(enable_controller=True)
        self.node.send_command_sample.reset_mock()
        self.update(q_eq=75)
        self.assertEqual(self.config["q_eq"], 60)
        self.update(load_name="unused", load_now=True)
        self.assertFalse(self.node._sm_file_result.result[0])
        self.assertFalse(self.config["load_now"])
        self.node.send_command_sample.assert_not_called()

    def test_disable_still_sent_with_invalid_edit(self):
        self.update(enable_controller=True)
        self.node.send_command_sample.reset_mock()
        self.update(enable_controller=False, q_eq=130)
        self.node.send_command_sample.assert_called_once()
        self.assertEqual(self.node.ctrl_msg[0], 0)
        self.assertEqual(self.config["q_eq"], 60)

    def test_save_sends_no_command_and_load_sends_selected_profile(self):
        self.update(q_eq=70)
        self.node.send_command_sample.reset_mock()
        self.update(save_name="tuned", save_now=True)
        self.assertTrue(self.node._sm_file_result.result[0])
        self.assertFalse(self.config["save_now"])
        self.node.send_command_sample.assert_not_called()
        self.update(q_eq=65)
        self.node.send_command_sample.reset_mock()
        self.update(load_name="tuned", load_now=True)
        self.assertEqual(self.config["q_eq"], 70)
        self.assertFalse(self.config["load_now"])
        self.node.send_command_sample.assert_called_once()

    def test_initial_stored_enable_and_file_actions_are_cleared(self):
        self.config.update(enable_controller=True, save_now=True, load_now=True)
        self.node._multi_activity_callback(self.config, True)
        self.assertFalse(self.config["enable_controller"])
        self.assertFalse(self.config["save_now"])
        self.assertFalse(self.config["load_now"])
        self.assertEqual(self.node.ctrl_msg[0], 0)

    def test_load_never_enables_and_failed_ack_is_reported(self):
        self.profiles.save("tuned")
        self.node.send_command_sample.return_value = False
        self.update(load_name="tuned", load_now=True, enable_controller=True)
        self.assertFalse(self.config["enable_controller"])
        self.assertEqual(self.node.ctrl_msg[0], 0)
        self.assertFalse(self.node._sm_file_result.result[0])


if __name__ == "__main__":
    unittest.main()
