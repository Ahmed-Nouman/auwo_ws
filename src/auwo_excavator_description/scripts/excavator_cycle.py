#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from enum import Enum, auto

# URDF joint ranges for clamping (body is continuous so we won't clamp it)
ARM1_MIN, ARM1_MAX = -1.308, -0.087     # boom
ARM2_MIN, ARM2_MAX = -2.428, -0.085     # stick
SHOVEL_MIN, SHOVEL_MAX = -2.395, -0.357 # bucket


def clamp_arm_joints(q):
    # q = [body, arm1, arm2, shovel]
    body, a1, a2, sh = q
    a1 = max(ARM1_MIN, min(ARM1_MAX, a1))
    a2 = max(ARM2_MIN, min(ARM2_MAX, a2))
    sh = max(SHOVEL_MIN, min(SHOVEL_MAX, sh))
    return [body, a1, a2, sh]


class CycleState(Enum):
    PREPARE_DIG = auto()
    SCOOP_AND_SLEW = auto()
    LIFT_AND_ALIGN_DUMP = auto()
    DUMP = auto()
    RETURN_HOME = auto()


class ExcavationCycleNode(Node):
    def __init__(self):
        super().__init__('excavation_cycle_node')

        # Publisher to your joint group position controller
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/arm_position_controller/commands',
            10
        )

        # Control loop timer (10 Hz like teleop)
        self.timer = self.create_timer(0.1, self._tick)

        # High-level sequence of states
        self.sequence = [
            CycleState.PREPARE_DIG,
            CycleState.SCOOP_AND_SLEW,
            CycleState.LIFT_AND_ALIGN_DUMP,
            CycleState.DUMP,
            CycleState.RETURN_HOME,
        ]
        self.state_index = 0
        self.state_start_wall = time.monotonic()

        # How long we "hold" each state (seconds)
        self.state_hold_seconds = {
            CycleState.PREPARE_DIG:         3.0,
            CycleState.SCOOP_AND_SLEW:      3.0,
            CycleState.LIFT_AND_ALIGN_DUMP: 3.0,
            CycleState.DUMP:                2.5,
            CycleState.RETURN_HOME:         3.0,
        }

        # Current commanded joint vector
        self.q = [0.0, -0.5, -1.0, -1.0]
        self.target_q = self.q[:]

        # Max per-tick change (rad/tick)
        self.max_step = 0.02

        self.get_logger().info("🟢 excavation_cycle_node (incremental stepping like teleop) started")

    #
    # --- State machine helpers ---
    #
    def _advance_state_if_time(self):
        """Switch to next high-level pose after hold time."""
        now = time.monotonic()
        current_state = self.sequence[self.state_index]
        if now - self.state_start_wall >= self.state_hold_seconds[current_state]:
            self.state_index = (self.state_index + 1) % len(self.sequence)
            self.state_start_wall = now
            self.get_logger().info(f"➡ state: {self.sequence[self.state_index].name}")

    def _desired_pose_for_state(self, state: CycleState):
        """Return the target joint pose [body, arm1, arm2, shovel] for the given state."""

        if state == CycleState.PREPARE_DIG:
            # boom & stick down/out, bucket open, body facing pile
            return [0.0, -1.20, -2.20, -0.50]

        elif state == CycleState.SCOOP_AND_SLEW:
            # start curling bucket + pulling stick, begin slewing body left toward dump
            return [0.5, -1.10, -1.80, -2.20]

        elif state == CycleState.LIFT_AND_ALIGN_DUMP:
            # raise boom with load, rotate more toward dump
            return [1.0, -0.40, -0.90, -2.20]

        elif state == CycleState.DUMP:
            # keep body at dump angle, open bucket
            return [1.0, -0.40, -0.90, -0.50]

        elif state == CycleState.RETURN_HOME:
            # rotate body back toward pile, neutral pose
            return [0.0, -0.60, -1.20, -1.20]

        # fallback
        return self.q[:]

    def _update_target_from_state(self):
        """Refresh target_q based on current high-level state."""
        current_state = self.sequence[self.state_index]
        desired = self._desired_pose_for_state(current_state)
        desired = clamp_arm_joints(desired)
        self.target_q = desired

    def _step_toward_target(self):
        """Move q toward target_q by at most self.max_step per joint per tick."""
        new_q = list(self.q)
        for i in range(4):
            diff = self.target_q[i] - self.q[i]
            if abs(diff) <= self.max_step:
                new_q[i] = self.target_q[i]
            else:
                new_q[i] += self.max_step * (1.0 if diff > 0.0 else -1.0)
        new_q = clamp_arm_joints(new_q)
        self.q = new_q

    def _publish(self):
        msg = Float64MultiArray()
        msg.data = self.q
        self.pub.publish(msg)

    def _tick(self):
        """Timer callback."""
        self._advance_state_if_time()
        self._update_target_from_state()
        self._step_toward_target()
        self._publish()


#
# --- Main entry point ---
#
def main(args=None):
    rclpy.init(args=args)
    node = ExcavationCycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🟥 excavation_cycle_node interrupted — shutting down cleanly")
    finally:
        node.destroy_node()
        # Only call shutdown if the context is still active
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

