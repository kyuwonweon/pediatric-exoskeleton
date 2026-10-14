"""Validated, independent CubeMars activity profiles and YAML persistence."""

import copy
import math
import os
import re
import tempfile

import yaml

ACTIVITIES = ("OW", "StairsAsc", "StairsDesc", "RampAsc", "RampDesc", "StS")
STS_ACTIVITY = 5
PHASES = ("Extension", "Flexion")
PHASE_FIELDS = ("q_eq", "qdot_eq", "dt", "K", "B")
ACTIVITY_FIELDS = ("vel_tresh", "Fz_tresh")
FIELD_BOUNDS = {
    "q_eq": (30.0, 150.0), "qdot_eq": (0.0, 2.0), "dt": (0.01, 10.0),
    "K": (0.0, 3.0), "B": (0.0, 0.3), "vel_tresh": (5.0, 30.0),
    "Fz_tresh": (0.0, 50.0),
}


def _number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a number".format(field))
    value = float(value)
    low, high = FIELD_BOUNDS[field]
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError("{} must be finite and in [{}, {}]".format(field, low, high))
    return value


def validate_activities(activities):
    if not isinstance(activities, dict) or set(activities) != set(ACTIVITIES):
        raise ValueError("A profile file must contain all six CubeMars activities")
    result = {}
    for activity in ACTIVITIES:
        entry = activities[activity]
        if not isinstance(entry, dict) or set(entry) != set(ACTIVITY_FIELDS + ("phases",)):
            raise ValueError("Invalid activity fields for {}".format(activity))
        phases = entry["phases"]
        if not isinstance(phases, dict) or set(phases) != set(PHASES):
            raise ValueError("{} requires Extension and Flexion".format(activity))
        row = {field: _number(entry[field], field) for field in ACTIVITY_FIELDS}
        row["phases"] = {}
        for phase in PHASES:
            values = phases[phase]
            if not isinstance(values, dict) or set(values) != set(PHASE_FIELDS):
                raise ValueError("Invalid phase fields for {} / {}".format(activity, phase))
            row["phases"][phase] = {field: _number(values[field], field) for field in PHASE_FIELDS}
        extension = row["phases"]["Extension"]["q_eq"]
        flexion = row["phases"]["Flexion"]["q_eq"]
        if not 30 <= extension <= 90 or not 90 <= flexion <= 150 or flexion <= extension:
            raise ValueError("{} requires extension in [30,90], flexion in [90,150], and flexion > extension".format(activity))
        result[activity] = row
    return result


class CANSMProfiles:
    def __init__(self, default_file, saved_dir, node_id):
        self.saved_dir = os.path.expanduser(saved_dir)
        self.node_id = int(node_id)
        self.activities, self.migrated_legacy = self._read(default_file)

    def _read(self, filename):
        with open(filename, "r") as handle:
            try:
                document = yaml.safe_load(handle)
            except yaml.YAMLError as error:
                raise ValueError("Invalid profile YAML: {}".format(error))
        if not isinstance(document, dict) or type(document.get("schema_version")) is not int or document["schema_version"] not in (1, 2):
            raise ValueError("Expected a version-1 or version-2 CubeMars profile file")
        if document.get("controller") != "CubeMarsSMMultipleAct":
            raise ValueError("Not a CubeMars two-state profile file; Fox four-phase files are not interchangeable")
        if "can_node_id" in document and document["can_node_id"] != self.node_id:
            raise ValueError("Profile belongs to CAN node {}, not {}".format(document["can_node_id"], self.node_id))
        activities = document.get("activities")
        legacy = document["schema_version"] == 1
        if legacy:
            if not isinstance(activities, dict) or set(activities) != set(ACTIVITIES[:-1]):
                raise ValueError("Version-1 snapshots must contain the five original activities")
            activities = copy.deepcopy(activities)
            # Untuned bench defaults; never reuse another activity's tuned row.
            activities["StS"] = {
                "vel_tresh": 10.0, "Fz_tresh": 10.0,
                "phases": {
                    "Extension": dict(q_eq=60.0, qdot_eq=0.0, dt=2.0, K=0.1, B=0.01),
                    "Flexion": dict(q_eq=120.0, qdot_eq=0.0, dt=2.0, K=0.1, B=0.01),
                },
            }
        return validate_activities(activities), legacy

    def display(self, config, activity, phase):
        row = self.activities[activity]
        for field in PHASE_FIELDS:
            config[field] = float(row["phases"][phase][field])
        for field in ACTIVITY_FIELDS:
            config[field] = float(row[field])

    def update(self, config, activity, phase):
        candidate = copy.deepcopy(self.activities)
        row = candidate[activity]
        row["phases"][phase] = {field: config[field] for field in PHASE_FIELDS}
        for field in ACTIVITY_FIELDS:
            row[field] = config[field]
        self.activities = validate_activities(candidate)

    def wire_values(self, activity):
        row = self.activities[activity]
        return [row["phases"][phase][field] for phase in PHASES for field in PHASE_FIELDS] + [row[field] for field in ACTIVITY_FIELDS]

    def filename(self, name):
        if not isinstance(name, str) or re.fullmatch(r"[A-Za-z0-9_-]{1,64}", name) is None:
            raise ValueError("Use a 1-64 character save/load name containing only letters, digits, '_' or '-'")
        return os.path.join(self.saved_dir, name + ".yaml")

    def save(self, name):
        filename = self.filename(name)
        os.makedirs(self.saved_dir, exist_ok=True)
        document = {
            "schema_version": 2, "controller": "CubeMarsSMMultipleAct",
            "can_node_id": self.node_id, "activities": validate_activities(self.activities),
        }
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", dir=self.saved_dir, delete=False) as handle:
                temporary = handle.name
                yaml.safe_dump(document, handle, sort_keys=False)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, filename)
            temporary = None
        finally:
            if temporary is not None:
                os.unlink(temporary)
        return filename

    def load(self, name):
        filename = self.filename(name)
        activities, legacy = self._read(filename)
        self.activities, self.migrated_legacy = activities, legacy
        return filename
