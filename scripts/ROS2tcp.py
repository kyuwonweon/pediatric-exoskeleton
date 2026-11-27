#!/usr/bin/env python3

import os
import socket
import json
import time
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, String

from micropython_logger.reconfigure import ReconfigureServer
from micropython_logger.ctrl_configs import (
    GuiOscillatorPos as GuiOscillatorPosConfig,
    GuiOscillatorForce as GuiOscillatorForceConfig,
    GuiFFCal as GuiFFCalConfig,
    GuiFBTransp as GuiFBTranspConfig,
    GuiTeleop as GuiTeleopConfig,
    GuiSMWalking as GuiSMWalkingConfig,
    GuiSMWalking3States as GuiSMWalking3StatesConfig,
    GuiSMMultiAct as GuiSMMultiActConfig,
)

from std_srvs.srv import Trigger

LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Must match ACTIVITIES / states_order in
# upy-app-exo/lib/Controllers/ctrlSMMultipleAct.py exactly (order + names) --
# the enum values on the wire are indices into these.
ACTIVITIES = ["OW", "StairsAsc", "StairsDesc", "RampAsc", "RampDesc"]
PHASES = ["EarlyStance", "LateStance", "EarlySwing", "LateSwing"]

# GUI-only mode selector (not sent over the wire): Manual = activity dropdown
# drives things directly. FromTopic_ComputerVision = activity is driven
# automatically by terrain_primitives_publisher's ~locomotion_mode topic (see
# _on_locomotion_topic / project_activity_classifier_two_nodes memory for
# what that node classifies and how it's smoothed). Named with the
# _ComputerVision suffix so future non-vision sources (e.g. IMU-based) can be
# added as their own distinct enum entries -- must match GuiSMMultiAct.cfg's
# activity_source_enum names exactly.
ACTIVITY_SOURCES = ["Manual", "FromTopic_ComputerVision"]

# terrain_primitives_publisher's directed locomotion_mode labels -> this
# node's ACTIVITIES taxonomy. "unknown" and anything unrecognised fall back
# to OW -- same fail-safe convention as activate()'s never-configured-phase
# fallback (see project_sm_multiactivity_controller memory).
LOCOMOTION_TOPIC_TO_ACTIVITY = {
    "overground":   "OW",
    "ramp_up":      "RampAsc",
    "ramp_down":    "RampDesc",
    "stairs_up":    "StairsAsc",
    "stairs_down":  "StairsDesc",
    "unknown":      "OW",
}

# The 5 generic per-(activity, phase) fields in GuiSMMultiAct.cfg, in wire order.
SM_MULTI_ACT_PHASE_FIELDS = ["q_eq", "qdot_eq", "dt", "K", "B"]

# Per-activity (not per-phase) fields -- broadcast to all 4 phases on the
# firmware side, same as CtrlSMWalking's original vel_tresh/Fz_tresh.
# thigh_tresh only actually affects EarlyStance (the others ignore it), but
# is cached/saved/loaded here the same generic way as the other two.
SM_MULTI_ACT_ACTIVITY_FIELDS = ["vel_tresh", "Fz_tresh", "thigh_tresh"]

# ROS1 ran from the source tree and saved snapshots next to this script. An
# installed ROS2 package is not the place to write, so defaults ship in the
# package share directory and new snapshots go under ROS_HOME (both overridable
# with the sm_params_file / sm_saved_dir parameters). Loading falls back to the
# shipped saved/ snapshots, so Lorenzo's SM_*.yaml files still load by name.
SM_MULTI_ACT_PARAMS_DIR = os.path.join(get_package_share_directory("micropython_logger"), "sm_multi_act_params")
SM_MULTI_ACT_DEFAULT_FILE = os.path.join(SM_MULTI_ACT_PARAMS_DIR, "default.yaml")
SM_MULTI_ACT_SHIPPED_SAVED_DIR = os.path.join(SM_MULTI_ACT_PARAMS_DIR, "saved")
SM_MULTI_ACT_SAVED_DIR = os.path.join(
    os.environ.get("ROS_HOME", os.path.expanduser("~/.ros")), "sm_multi_act_params", "saved")

"""
This class is design for slower but more reliable comunication 
in particular this class manages comunication of the kind
dynamic config --> tcp
"""
class ROS2tcp(Node):
    def __init__(self):
        # define ros interfaces
        super().__init__('ros2_tcp')
        port =12345
        robot = self.declare_parameter('robot', "CubeMars").value  # CubeMars mostly talks over the canIP channels (ROS2canIP.py), not tcp
        print("Robot ", robot)
        if robot == "Tails":
            receiver_ip= '192.168.0.172'
            # receiver_ip= '192.168.0.68'
            # receiver_ip= '192.168.4.1' # in the case in which tails is the access point
        elif robot == "Shadow":
            receiver_ip= '192.168.0.254'
        elif robot == "CubeMars":
            receiver_ip= '192.168.0.202'
        else:
            raise Exception("Wrong Robot assignement")
        # Optional override, e.g. a bench board on another address or a local test server.
        receiver_ip = self.declare_parameter('receiver_ip', receiver_ip).value
        print(receiver_ip, port)


        ctrl = self.declare_parameter('ctrl', "SMWalking").value
        print("Ctrl ", ctrl )
        self.ctrl = ctrl
        ctrl_msg_dim = 20
        if ctrl == "Teleop":
            header = 3
            config_params = GuiTeleopConfig
        if ctrl == "SMWalking":
            header = 4
            if robot == "CubeMars":
                config_params = GuiSMWalking3StatesConfig
                ctrl_msg_dim = 20
            else:
                config_params = GuiSMWalkingConfig
                ctrl_msg_dim = 23
        elif ctrl == "SMWalking3States":
            header = 4
            config_params = GuiSMWalking3StatesConfig
            ctrl_msg_dim = 20
        elif ctrl == "SMWalking4States":
            header = 4
            config_params = GuiSMWalkingConfig
            ctrl_msg_dim = 23
        elif ctrl == "SMMultipleAct":
            header = 7
            config_params = GuiSMMultiActConfig
            ctrl_msg_dim = 24
        elif ctrl== "OscillatorPos":
            header = 6
            config_params = GuiOscillatorPosConfig
        elif ctrl== "OscillatorForce":
            header = 5
            config_params = GuiOscillatorForceConfig
        elif ctrl== "FF":
            header = 1
            config_params = GuiFFCalConfig
        elif ctrl == "FB":
            header = 2
            config_params = GuiFBTranspConfig

        # define socket interfaces
        self.s = socket.socket()
        try:
            # fail fast if the board isn't reachable (e.g. bench-testing the
            # GUI/params alone, no hardware connected) instead of blocking on
            # the OS default TCP connect timeout (can be 1-2 minutes)
            self.s.settimeout(3.0)
            self.s.connect((receiver_ip, port))
            self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.s.settimeout(10.0)
        except OSError as e:
            print("Socket error:", e, ", Unable to use wifi --> stdout")

        # wait a second the socket to start
        time.sleep(1)

        self.header_msg = np.array([header])
        self.ctrl_msg = np.zeros(ctrl_msg_dim)
        # robot msg:
        # list[int], by default all zero
        # if index 0 ==1 --> imu calibration
        # elif index 2 ==1 --> load-cell calibration[int]
        self.robot_msg = np.zeros(4)

        if self.ctrl == "SMMultipleAct":
            self._init_sm_multi_act()
            reconfigure_callback = self.callback_sm_multi_act
        else:
            reconfigure_callback = self.callback

        # Create a Dynamic Reconfigure Server
        self.srv = ReconfigureServer(self, config_params, reconfigure_callback)
        self.service_IMU_Cal = self.create_service(Trigger, 'IMU_calibration', self.imu_cal_callback_)
        self.service_Load_Cel_cal = self.create_service(Trigger, 'LoadCell_calibration', self.LoadCell_cal_callback_)

        if self.ctrl == "SMMultipleAct":
            # constructing self.srv above already fired one callback using the
            # .cfg's hardcoded defaults for just the default (activity, phase)
            # -- push every (activity, phase) pair's real loaded values to the
            # firmware now, so nothing is left relying on activate()'s
            # OW-fallback just because the operator never happened to visit
            # it via the two dropdowns. Safe here: enable_controller is
            # always False at this point (node has just started).
            self._push_all_to_firmware()
            self.service_Save_SM = self.create_service(Trigger, 'SaveSMParams', self.save_sm_params_callback_)
            self.service_Load_SM = self.create_service(Trigger, 'LoadSMParams', self.load_sm_params_callback_)

            # Only safe to subscribe now that self.srv exists -- see the NOTE
            # in _init_sm_multi_act. locomotion_mode_announcer's ~committed_mode
            # is latched, so this can fire literally as soon as this line runs.
            locomotion_topic = self.declare_parameter(
                "locomotion_topic", "/locomotion_mode_announcer/committed_mode").value
            # ROS1 latched topic -> transient-local QoS so a late subscriber still gets the last mode.
            self.create_subscription(String, locomotion_topic, self._on_locomotion_topic, LATCHED_QOS)
            self.get_logger().info("SMMultipleAct: listening for FromTopic activity on {}".format(locomotion_topic))

        
    def send_and_Ack_msg(self):
        success = False
        try:
            rp = np.zeros(1 + len(self.ctrl_msg) + len(self.robot_msg), dtype=np.float32)
            rp[0:1] = self.header_msg
            rp[1:1 + len(self.ctrl_msg)] = self.ctrl_msg
            rp[1 + len(self.ctrl_msg):] = self.robot_msg
            msg = rp.astype(np.float32).tobytes()
            # compute time to go and come bacl
            start = time.perf_counter()
            self.s.send(msg)
            # print("--sent message")
            # self.s.settimeout(2.0)
            data = self.s.recv(len(msg))
            elapsed_ms = (time.perf_counter() - start) * 1000

            if data== msg:
                success = True
                if not getattr(self, '_bulk_pushing', False):
                    print("----Received confirmation, Time to go and comeback: ", elapsed_ms)
            else:
                # TCP has no message framing -- recv() just returns whatever
                # bytes are currently queued. This is most often a STALE
                # reply to a PREVIOUS message that arrived late (after we'd
                # already given up waiting on it), not corruption of this
                # particular message. Treat as non-fatal like the other
                # failure modes below (a bare `raise` here previously
                # crashed the whole node on this exact condition), and drain
                # the socket afterward so it can't also corrupt the NEXT read.
                # Always printed, even while bulk pushing -- a real failure
                # shouldn't go silent just because it's noisy elsewhere.
                print("Wrong message back (likely a stale/late reply to a previous message) -- discarding and resyncing")

            if not getattr(self, '_bulk_pushing', False):
                print("-------------------------------")
        except AttributeError as e:
            print(e)
            print("Client not yet connected")
        except OSError as e:
            # covers socket.timeout/TimeoutError, BrokenPipeError,
            # ConnectionRefusedError, and "No route to host"/"Network is
            # unreachable" (the board being off/unreachable at send time,
            # not just at connect time) -- all OSError subclasses in
            # Python 3. Catching only specific narrow subclasses here left
            # this reachable and uncaught, crashing the whole node the
            # moment Server.__init__ fires its first internal callback.
            print(e)
            print("Client not yet connected")

        if not success:
            self._drain_socket()
        return success

    def _drain_socket(self, timeout=0.05):
        """Discard any stale bytes still sitting in the socket's receive
        buffer -- e.g. a late reply to a message we already gave up waiting
        on, which would otherwise get misread as the reply to the NEXT
        message we send (see the note in send_and_Ack_msg). Called after
        any failed round trip, before the next send."""
        previous_timeout = self.s.gettimeout()
        try:
            self.s.settimeout(timeout)
            while True:
                try:
                    stale = self.s.recv(4096)
                    if not stale:
                        break  # peer closed
                    print("Draining stale bytes:", len(stale))
                except OSError:
                    break  # nothing more waiting right now -- drained
        finally:
            try:
                self.s.settimeout(previous_timeout)
            except OSError:
                pass

    def callback(self, config, level):
        # rospy.loginfo("Reconfigure Request: {freq_value}, {enable_controller}, {use_FF}, {use_Energy_var_freq}, {K_stiffness}, {B_damping}".format(**config))
        # You can perform actions based on the new parameter values here
        # call back of dyn reconfig for the controller
        print("---------------------")
        for k_i, k in enumerate(config.keys()):
            if k!= "groups":
                print(k, config[k])
                self.ctrl_msg[k_i] = (float(config[k]))
        print("---------------------")
        
        self.send_and_Ack_msg()

        return config

    def _init_sm_multi_act(self):
        self.sm_params_file = self.declare_parameter("sm_params_file", SM_MULTI_ACT_DEFAULT_FILE).value
        self.sm_saved_dir = os.path.expanduser(self.declare_parameter("sm_saved_dir", SM_MULTI_ACT_SAVED_DIR).value)
        os.makedirs(self.sm_saved_dir, exist_ok=True)
        if os.path.exists(self.sm_params_file):
            with open(self.sm_params_file, "r") as f:
                self.params_by_activity = yaml.safe_load(f) or {}
        else:
            self.get_logger().warning("No default SM multi-act params file at {}, starting from .cfg defaults".format(self.sm_params_file))
            self.params_by_activity = {}
        self.current_gui_activity = ACTIVITIES[0]
        self.current_gui_phase = PHASES[0]
        self._bulk_pushing = False  # set True during _push_all_to_firmware() to quiet the per-message logging
        self._last_send_ok = False
        self._last_known_enable_controller = False

        # ── FromTopic: classifier-driven activity switching ──────────────────
        # Set True only while the topic callback's own update_configuration()
        # call is in flight, so callback_sm_multi_act's FromTopic-guard (below)
        # can tell "this activity change came from the trusted topic path" apart
        # from "the operator clicked the activity dropdown while in FromTopic
        # mode" (which must still be rejected/pinned to OW, per the existing
        # verified behaviour).
        self._activity_from_topic_update = False
        # Only ever call update_configuration() when the incoming label
        # actually differs from the last one we acted on -- avoids a full
        # config deepcopy/level-diff + parameter_updates publish for a no-op.
        # This is intentionally the ONLY debounce on this path -- no separate
        # time-based dwell here. locomotion_mode_announcer.py already did that
        # debouncing upstream (see its ~min_dwell_sec) before republishing on
        # ~committed_mode; re-debouncing on a second, independent timer here
        # would let the spoken cue and the leg's activity switch disagree
        # about which transition is "real". ctrlSMMultipleAct.py also still
        # has its own lower-level safety net regardless -- while running, a
        # new activity only ever latches at the next EarlySwing entry (the
        # start of the swing that carries the foot onto the new terrain),
        # never mid-phase.
        self._last_topic_activity = None
        # NOTE: the ~locomotion_topic Subscriber itself is created later, in
        # __init__ AFTER self.srv exists -- NOT here. _on_locomotion_topic reads
        # self.srv.config, and locomotion_mode_announcer's ~committed_mode is
        # LATCHED, so subscribing this early would let the very first (buffered)
        # message fire the callback before self.srv = Server(...) has even run
        # yet, crashing with AttributeError: 'ROS2tcp' object has no attribute 'srv'
        # (hit exactly this live during testing).

    def _on_locomotion_topic(self, msg):
        """locomotion_mode_announcer's ~committed_mode subscriber (already
        debounced upstream -- see the note in _init_sm_multi_act). Runs on
        rospy's subscriber-callback thread, NOT nested inside
        Server._change_config -- same safe context as _reset_save_load_checkboxes
        / _deferred_push_all_to_firmware, so calling update_configuration()
        here is fine (see the NOTE in callback_sm_multi_act about why it's
        NOT safe to call from inside that callback itself)."""
        mapped_activity = LOCOMOTION_TOPIC_TO_ACTIVITY.get(msg.data, "OW")

        if mapped_activity == self._last_topic_activity:
            return

        # Only actually drive the leg while FromTopic is selected. Deliberately
        # do NOT update _last_topic_activity here while in Manual mode: leaving
        # it stale means the first message received after the operator later
        # switches to FromTopic will look "changed" and correctly get pushed,
        # instead of being silently swallowed by the equality check above.
        if ACTIVITY_SOURCES[int(self.srv.config['activity_source'])] != "FromTopic_ComputerVision":
            return

        self._last_topic_activity = mapped_activity
        self.get_logger().info("SMMultipleAct: FromTopic activity -> {} (from {!r})".format(
            mapped_activity, msg.data))
        self._activity_from_topic_update = True
        try:
            self.srv.update_configuration({'activity': ACTIVITIES.index(mapped_activity)})
        finally:
            self._activity_from_topic_update = False

    def _get_cached_phase(self, activity, phase):
        return self.params_by_activity.get(activity, {}).get('phases', {}).get(phase, {})

    def _snapshot_current(self, config):
        """Capture config's current phase fields + activity-level fields into
        the cache under self.current_gui_activity / self.current_gui_phase."""
        act_entry = self.params_by_activity.setdefault(self.current_gui_activity, {})
        act_entry.setdefault('phases', {})[self.current_gui_phase] = {
            k: float(config[k]) for k in SM_MULTI_ACT_PHASE_FIELDS
        }
        for k in SM_MULTI_ACT_ACTIVITY_FIELDS:
            act_entry[k] = float(config[k])

    def _cached_changes_for(self, activity, phase):
        """Build a {field: float value} dict of everything cached for
        (activity, phase), suitable for Server.update_configuration(). Always
        casts to float -- see the note in _apply_cached_to_config."""
        changes = {}
        stored_phase = self._get_cached_phase(activity, phase)
        changes.update({k: float(stored_phase[k]) for k in SM_MULTI_ACT_PHASE_FIELDS if k in stored_phase})
        stored_act = self.params_by_activity.get(activity, {})
        changes.update({k: float(stored_act[k]) for k in SM_MULTI_ACT_ACTIVITY_FIELDS if k in stored_act})
        return changes

    def _push_all_to_firmware(self):
        """Send every (activity, phase) pair's cached params to the firmware,
        one message at a time -- the same wire format/mechanism as ordinary
        GUI-driven configuration (callback_sm_multi_act already does the
        right thing whenever activity/phase change: snapshot outgoing, apply
        cached incoming, send). Only the two selector ints need to be passed
        here; the callback's own cache lookup fills in the rest.

        Only safe to call while stopped. While running, set_params_ctrl
        ignores all param fields and only updates pending_activity from the
        activity selector -- looping through 20 combos would leave
        pending_activity set to whatever combo happened to be pushed last,
        not what the operator actually wants. Refuses (with a warning) if
        the controller is currently running.
        """
        if self.srv.config['enable_controller']:
            self.get_logger().warning("Not pushing all (activity, phase) pairs to firmware while running -- stop the controller first")
            return

        original_activity, original_phase = self.current_gui_activity, self.current_gui_phase
        total = len(ACTIVITIES) * len(PHASES)
        print("Pushing {} (activity, phase) pairs to firmware...".format(total))
        ok_count = 0
        self._bulk_pushing = True
        try:
            for activity in ACTIVITIES:
                for phase in PHASES:
                    self.srv.update_configuration({
                        'activity': ACTIVITIES.index(activity),
                        'phase': PHASES.index(phase),
                    })
                    if self._last_send_ok:
                        ok_count += 1
                    # give the board breathing room between back-to-back sends --
                    # doesn't fix the underlying stale-ACK race (see send_and_Ack_msg),
                    # but makes it less likely to trigger in the first place
                    time.sleep(0.05)
        finally:
            self._bulk_pushing = False
        print("Pushed {}/{} pairs successfully.".format(ok_count, total))

        # restore whichever (activity, phase) the GUI was showing before the bulk push
        self.srv.update_configuration({
            'activity': ACTIVITIES.index(original_activity),
            'phase': PHASES.index(original_phase),
        })

    def _apply_cached_to_config(self, config, activity, phase):
        """Overwrite config's phase + activity-level fields with whatever is
        cached for (activity, phase), for whichever fields are cached.

        Always casts to float: these fields are declared double_t in the cfg,
        but a value loaded from YAML (default.yaml or a hand-edited saved
        snapshot) is a plain Python int if it was written without a decimal
        point (e.g. "K: 0" instead of "K: 0.0"). dynamic_reconfigure's
        encode_config() picks the wire type (ints vs doubles) from the
        runtime Python type of the value, not the field's declared cfg type,
        so an int here would get serialized into the wrong bucket.
        """
        stored_phase = self._get_cached_phase(activity, phase)
        for k in SM_MULTI_ACT_PHASE_FIELDS:
            if k in stored_phase:
                config[k] = float(stored_phase[k])
        stored_act = self.params_by_activity.get(activity, {})
        for k in SM_MULTI_ACT_ACTIVITY_FIELDS:
            if k in stored_act:
                config[k] = float(stored_act[k])

    def callback_sm_multi_act(self, config, level):
        # In FromTopic mode, only _on_locomotion_topic's own
        # update_configuration() call (flagged via _activity_from_topic_update)
        # may actually change `activity` -- an operator clicking the activity
        # dropdown themselves gets pinned back to OW. _bulk_pushing is exempt
        # too: it deliberately needs to deliver every activity's data to the
        # firmware regardless of the current mode; this gate is about which
        # activity OPERATES, not what can be configured/pushed.
        if (not self._bulk_pushing and not self._activity_from_topic_update
                and ACTIVITY_SOURCES[int(config['activity_source'])] == "FromTopic_ComputerVision"):
            config['activity'] = 0

        new_activity = ACTIVITIES[int(config['activity'])]
        new_phase = PHASES[int(config['phase'])]

        activity_changed = new_activity != self.current_gui_activity
        phase_changed = new_phase != self.current_gui_phase
        enable_changed = bool(config['enable_controller']) != self._last_known_enable_controller
        self._last_known_enable_controller = bool(config['enable_controller'])

        # Did the operator actually change a param VALUE in this callback (as
        # opposed to just switching which activity/phase they're looking at)?
        # Compare against what's cached for the OUTGOING combo, before it
        # gets overwritten below by the incoming combo's cached values.
        prev_phase_row = self._get_cached_phase(self.current_gui_activity, self.current_gui_phase)
        prev_act_row = self.params_by_activity.get(self.current_gui_activity, {})
        params_changed = (
            any(float(config[k]) != prev_phase_row.get(k) for k in SM_MULTI_ACT_PHASE_FIELDS)
            or any(float(config[k]) != prev_act_row.get(k) for k in SM_MULTI_ACT_ACTIVITY_FIELDS)
        )

        if activity_changed or phase_changed:
            # a dropdown just changed -- config still holds the OUTGOING
            # (activity, phase)'s values (dynamic_reconfigure only changes
            # the field(s) actually touched), so snapshot those before switching
            self._snapshot_current(config)
            self.current_gui_activity = new_activity
            self.current_gui_phase = new_phase
            self._apply_cached_to_config(config, new_activity, new_phase)
        else:
            self._snapshot_current(config)

        load_triggered = False
        if config['save_now']:
            _, message = self._save_sm_params(config['save_name'], config)
            self.get_logger().info(message)
            config['save_now'] = False  # self-resetting "button"
            self._oneshot(0.3, self._reset_save_load_checkboxes)

        if config['load_now']:
            success, message = self._load_sm_params(config['load_name'])
            self.get_logger().info(message)
            if success:
                # refresh the GUI's displayed combo immediately (synchronous,
                # part of this callback's return value)...
                self._apply_cached_to_config(config, self.current_gui_activity, self.current_gui_phase)
                load_triggered = True
                # ...but pushing every (activity, phase) combo to the firmware
                # has to happen from a separate thread, not inline here -- see
                # the NOTE on self.srv.mutex below. Without this, checking
                # load_now in the GUI would only ever reach the firmware for
                # whichever one combo happens to be selected, same gap
                # _push_all_to_firmware() exists to close everywhere else.
                self._oneshot(0.35, self._deferred_push_all_to_firmware)
            config['load_now'] = False
            self._oneshot(0.3, self._reset_save_load_checkboxes)

        # Only actually talk to the firmware if something it needs to know
        # about changed. _bulk_pushing always sends regardless -- that's the
        # one caller that deliberately wants every (activity, phase) combo
        # delivered even when, from this function's point of view, "only
        # the phase changed" (see _push_all_to_firmware). Ordinary
        # interactive use (e.g. just browsing to a different phase to look
        # at its values, with nothing actually edited) skips the send --
        # nothing changed that the firmware needs, and every message is
        # extra load on a board we've already seen struggle under traffic
        # volume (heap pressure, WiFi stalls).
        if not (self._bulk_pushing or enable_changed or activity_changed or params_changed or load_triggered):
            return config

        # NOTE on the rospy.Timer calls above: setting config['save_now'/'load_now']
        # = False here already corrects the underlying value immediately (verified:
        # self.srv.config reflects False right after this callback returns). But
        # rqt_reconfigure's checkbox widget can fail to visually un-check when the
        # correction arrives in the exact same round-trip as the user's own click --
        # it can't tell that apart from an echo of its own pending edit. A short,
        # independent correction wave a moment later isn't attributable to the same
        # click, so it reliably repaints. Harmless if the display quirk doesn't
        # occur -- _reset_save_load_checkboxes() only sends changes if something is
        # actually still True.

        # NOTE: every branch above mutates `config` in place and relies on
        # Server._change_config to publish the returned config back to the
        # GUI -- never call self.srv.update_configuration() from inside this
        # callback. This callback runs from inside Server._change_config,
        # itself inside Server.update_configuration() holding self.srv.mutex
        # (a plain, non-reentrant threading.Lock) whenever the change came
        # from the GUI (Server._set_callback). A nested update_configuration()
        # call deadlocks re-acquiring that same lock on the same thread.

        # Build the wire message from the known field list, not positionally
        # from every cfg key -- config now also carries GUI-only fields
        # (save_name is a string; float() on it would crash).
        if not self._bulk_pushing:
            print("---------------------")
            print("SM multi-act activity/phase:", self.current_gui_activity, self.current_gui_phase)
        self.ctrl_msg[0] = float(config['enable_controller'])
        self.ctrl_msg[1] = float(config['activity'])
        self.ctrl_msg[2] = float(config['phase'])
        for i, k in enumerate(SM_MULTI_ACT_PHASE_FIELDS):
            self.ctrl_msg[3 + i] = float(config[k])
            if not self._bulk_pushing:
                print(k, config[k])
        self.ctrl_msg[8] = float(config['vel_tresh'])
        self.ctrl_msg[9] = float(config['Fz_tresh'])
        self.ctrl_msg[10] = float(config['thigh_tresh'])
        if not self._bulk_pushing:
            print("---------------------")

        self._last_send_ok = self.send_and_Ack_msg()
        return config

    def _reset_save_load_checkboxes(self, event=None):
        # runs from a rospy.Timer thread, not nested inside the dynamic_reconfigure
        # callback -- safe to call update_configuration() here (see the note in
        # callback_sm_multi_act about why it's NOT safe to call from inside it).
        current = self.srv.config
        changes = {}
        if current.get('save_now'):
            changes['save_now'] = False
        if current.get('load_now'):
            changes['load_now'] = False
        if changes:
            self.srv.update_configuration(changes)

    def _deferred_push_all_to_firmware(self, event=None):
        # runs from a rospy.Timer thread, not nested inside the dynamic_reconfigure
        # callback -- safe to call _push_all_to_firmware() here (it calls
        # update_configuration() once per (activity, phase) combo internally,
        # which would deadlock self.srv.mutex if called from inside the callback).
        self._push_all_to_firmware()

    def _sm_multi_act_file_for(self, name):
        safe_name = "".join(c for c in name if c.isalnum() or c in ("_", "-"))
        if not safe_name:
            raise ValueError("Invalid save/load name: {!r}".format(name))
        return os.path.join(self.sm_saved_dir, safe_name + ".yaml")

    def _save_sm_params(self, name, current_config):
        """Snapshot current_config's current (activity, phase) + activity-level
        fields into the cache, then write everything to <name>.yaml.
        Returns (success, message); never raises."""
        try:
            self._snapshot_current(current_config)
            filepath = self._sm_multi_act_file_for(name)
            with open(filepath, "w") as f:
                yaml.safe_dump(self.params_by_activity, f)
            return True, "Saved SM params to {}".format(filepath)
        except Exception as e:
            return False, str(e)

    def _load_sm_params(self, name):
        """Load <name>.yaml into self.params_by_activity. Returns
        (success, message); never raises. Caller is responsible for
        refreshing the GUI/config for whichever (activity, phase) is selected."""
        try:
            filepath = self._sm_multi_act_file_for(name)
            if not os.path.exists(filepath):
                # Snapshots shipped with the package (e.g. SM_AllOW) load by name too.
                filepath = os.path.join(SM_MULTI_ACT_SHIPPED_SAVED_DIR, os.path.basename(filepath))
            if not os.path.exists(filepath):
                return False, "No saved file at {}".format(filepath)
            with open(filepath, "r") as f:
                loaded = yaml.safe_load(f) or {}
            self.params_by_activity.update(loaded)
            return True, "Loaded SM params from {}".format(filepath)
        except Exception as e:
            return False, str(e)

    def save_sm_params_callback_(self, req, response=None):
        name = self.get_parameter('save_name').value
        if not name:
            return Trigger.Response(success=False, message="Set the save_name parameter before calling this service")
        success, message = self._save_sm_params(name, self.srv.config)
        return Trigger.Response(success=success, message=message)

    def load_sm_params_callback_(self, req, response=None):
        name = self.get_parameter('load_name').value
        if not name:
            return Trigger.Response(success=False, message="Set the load_name parameter before calling this service")
        success, message = self._load_sm_params(name)
        if success:
            # push everything just loaded to the firmware, not just whichever
            # combo happens to be selected right now -- see _push_all_to_firmware
            # (it also handles the "safe only while stopped" guard)
            self._push_all_to_firmware()
        return Trigger.Response(success=success, message=message)

    def imu_cal_callback_(self, req, response=None):
        self.get_logger().info("IMU Cal called!")
        self.robot_msg[0] = 1
        success = self.send_and_Ack_msg()
        if success:
            msg = "IMU cal completed successfully."
        else:
            msg = "IMU cal Not completed."
        # restore default value
        self.robot_msg[0] =0
        
        return Trigger.Response(success=success, message=msg)

    def LoadCell_cal_callback_(self, req, response=None):
        self.get_logger().info("LOAD cell cal called!")
        self.robot_msg[1] = 1
        success = self.send_and_Ack_msg()
        if success:
            msg = "Load cell cal completed successfully."
        else:
            msg = "Load cell cal Not completed."
        self.robot_msg[1] = 0
        return Trigger.Response(success=success, message=msg)

    
    def _oneshot(self, delay_s, callback):
        """rospy.Timer(..., oneshot=True) equivalent."""
        def fire():
            timer.cancel()
            self.destroy_timer(timer)
            callback()
        timer = self.create_timer(delay_s, fire)


def main(args=None):
    rclpy.init(args=args)
    R2t = ROS2tcp()
    try:
        rclpy.spin(R2t)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        R2t.s.close()
        R2t.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
