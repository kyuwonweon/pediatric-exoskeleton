"""dynamic_reconfigure-style server on top of ROS2 parameters.

The ROS1 CAN nodes rely on two dynamic_reconfigure behaviours that plain ROS2
parameter callbacks do not have:

  * the callback receives the *whole* configuration, not only changed fields;
  * the callback may rewrite values (e.g. reset enable_controller after a
    failed start, or refresh the profile fields when the editor selection
    changes) and the GUI then shows the rewritten values.

ReconfigureServer provides both through add_pre_set_parameters_callback, which
may replace the list of parameters being set. rqt_reconfigure, `ros2 param set`
and set_parameters_atomically all pass through it, so they behave like the ROS1
GUI did. Like dynamic_reconfigure, numbers are clamped to their range before
the callback sees them and requests are serialized.
"""

import threading

from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    ParameterType,
)
from rclpy.parameter import Parameter

_TYPES = {
    "bool": ParameterType.PARAMETER_BOOL,
    "int": ParameterType.PARAMETER_INTEGER,
    "double": ParameterType.PARAMETER_DOUBLE,
    "str": ParameterType.PARAMETER_STRING,
}


class ParamSpec:
    """One dynamic_reconfigure field: name, type, default, range, description."""

    def __init__(self, name, type_, default, minimum=None, maximum=None, description="", enum=None):
        if type_ not in _TYPES:
            raise ValueError("Unknown parameter type {!r}".format(type_))
        self.name = name
        self.type = type_
        self.default = self.coerce(default)
        self.minimum = minimum
        self.maximum = maximum
        self.description = description
        self.enum = enum or {}

    def coerce(self, value):
        if self.type == "bool":
            return bool(value)
        if self.type == "int":
            return int(value)
        if self.type == "double":
            return float(value)
        return str(value)

    def clamp(self, value):
        value = self.coerce(value)
        if self.minimum is not None and value < self.minimum:
            return self.coerce(self.minimum)
        if self.maximum is not None and value > self.maximum:
            return self.coerce(self.maximum)
        return value

    def descriptor(self):
        description = self.description
        if self.enum:
            # rqt_reconfigure has no ROS2 enum widget; list the choices instead.
            choices = ", ".join("{}={}".format(value, label) for value, label in sorted(self.enum.items()))
            description = "{} ({})".format(description, choices)
        descriptor = ParameterDescriptor(type=_TYPES[self.type], description=description)
        if self.type == "int" and self.minimum is not None:
            descriptor.integer_range = [IntegerRange(
                from_value=int(self.minimum), to_value=int(self.maximum), step=1)]
        elif self.type == "double" and self.minimum is not None:
            descriptor.floating_point_range = [FloatingPointRange(
                from_value=float(self.minimum), to_value=float(self.maximum), step=0.0)]
        return descriptor


class ReconfigureServer:
    """Same interface as dynamic_reconfigure.server.Server: config, update_configuration."""

    def __init__(self, node, specs, callback):
        self._node = node
        self._specs = {spec.name: spec for spec in specs}
        self._order = [spec.name for spec in specs]
        self._callback = callback
        self._lock = threading.RLock()
        # Per thread, so a concurrent GUI request never skips the callback.
        self._writing_back = threading.local()

        # Launch-file overrides apply here, like ROS1 values on the parameter server.
        for spec in specs:
            node.declare_parameter(spec.name, spec.default, spec.descriptor())
        config = {name: self._specs[name].clamp(node.get_parameter(name).value) for name in self._order}

        with self._lock:
            # dynamic_reconfigure runs the callback once at startup with all levels.
            self.config = self._run_callback(config, ~0)
            self._write_back(self._order)
        node.add_pre_set_parameters_callback(self._pre_set)

    def _run_callback(self, config, level):
        new_config = self._callback(dict(config), level)
        return {name: self._specs[name].clamp(new_config[name]) for name in self._order}

    def _write_back(self, names):
        """Publish callback-side changes without running the callback again."""
        if not names:
            return
        self._writing_back.active = True
        try:
            self._node.set_parameters([Parameter(name, value=self.config[name]) for name in names])
        finally:
            self._writing_back.active = False

    def update_configuration(self, changes):
        """Apply changes through the parameter interface and return the new config."""
        params = [Parameter(name, value=value) for name, value in changes.items()]
        with self._lock:
            if params:
                result = self._node.set_parameters_atomically(params)
                if not result.successful:
                    raise RuntimeError(result.reason)
            else:
                # An empty dynamic_reconfigure update re-runs the callback.
                previous = self.config
                self.config = self._run_callback(previous, 0)
                self._write_back([name for name in self._order if self.config[name] != previous[name]])
            return dict(self.config)

    def _pre_set(self, parameter_list):
        if getattr(self._writing_back, "active", False):
            return parameter_list
        managed = [param for param in parameter_list if param.name in self._specs]
        if not managed:
            return parameter_list
        others = [param for param in parameter_list if param.name not in self._specs]

        with self._lock:
            previous = self.config
            config = dict(previous)
            for param in managed:
                config[param.name] = self._specs[param.name].clamp(param.value)
            try:
                self.config = self._run_callback(config, 0)
            except Exception as error:  # Keep the node alive; show the old values.
                self._node.get_logger().error("Reconfigure callback failed: {}".format(error))
                self.config = previous
            requested = {param.name for param in managed}
            out = [
                Parameter(name, value=self.config[name]) for name in self._order
                if name in requested or self.config[name] != previous[name]
            ]
        return others + out
