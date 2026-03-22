#!/usr/bin/env python3
"""Debug probe: append NDJSON lines for AUWO MoveIt/Gazebo sessions (agent instrumentation)."""
# region agent log
import json
import time

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

_DEBUG_LOG = "/home/ra2/Devel/auwo_ws/.cursor/debug-e9f1b9.log"
_SESSION = "e9f1b9"


def _append(hypothesis_id: str, location: str, message: str, data=None) -> None:
    line = {
        "sessionId": _SESSION,
        "timestamp": int(time.time() * 1000),
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data or {},
    }
    try:
        with open(_DEBUG_LOG, "a", encoding="utf-8") as f:
            f.write(json.dumps(line, default=str) + "\n")
    except OSError:
        pass


# endregion


class _Probe(Node):
    def __init__(self) -> None:
        super().__init__("auwo_debug_session_probe")
        self._clock_count = 0
        self._js_count = 0
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(Log, "/rosout", self._on_rosout, 100)
        self.create_timer(3.0, self._tick)
        _append("H1", "auwo_debug_session_probe.py:init", "probe_started", {"use_sim_time_param": False})

    def _on_clock(self, _msg: Clock) -> None:
        self._clock_count += 1
        if self._clock_count == 1:
            _append("H1", "auwo_debug_session_probe.py:clock", "first_clock_msg", {})

    def _on_js(self, msg: JointState) -> None:
        self._js_count += 1
        if self._js_count == 1:
            _append(
                "H3",
                "auwo_debug_session_probe.py:joint_states",
                "first_joint_states",
                {
                    "names": list(msg.name),
                    "positions": [float(x) for x in msg.position] if msg.position else [],
                },
            )

    def _on_rosout(self, msg: Log) -> None:
        text = (msg.msg or "").lower()
        name = (msg.name or "").lower()
        if not (
            "move_group" in name
            or "controller_manager" in name
            or "trajectory" in text
            or "aborted" in text
            or "invalid trajectory" in text
            or "follow_joint" in text
            or "tolerance" in text
            or "velocity" in text
        ):
            return
        _append(
            "H2",
            "auwo_debug_session_probe.py:rosout",
            "filtered_rosout",
            {
                "level": int(msg.level),
                "name": msg.name,
                "msg": (msg.msg or "")[:800],
            },
        )

    def _tick(self) -> None:
        _append(
            "H1",
            "auwo_debug_session_probe.py:tick",
            "heartbeat",
            {
                "clock_msgs": self._clock_count,
                "joint_states_msgs": self._js_count,
            },
        )


def main() -> None:
    rclpy.init()
    node = _Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
