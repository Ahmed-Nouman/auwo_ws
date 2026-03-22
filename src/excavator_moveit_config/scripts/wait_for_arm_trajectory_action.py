#!/usr/bin/env python3
"""Block until arm_trajectory_controller FollowJointTrajectory action server exists.

Used by demo_moveit_rviz.launch.py and bucket_moveit.launch.py so move_group starts after
controllers and the FollowJointTrajectory action are ready.
"""
import argparse
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args


def _ros_controllers_active(node, lc_client) -> bool:
    """True when this launch's ros2_control stack has both excavator controllers active."""
    if not lc_client.service_is_ready():
        return False
    fut = lc_client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
    if not fut.done():
        return False
    res = fut.result()
    if res is None:
        return False
    states = {c.name: (c.state or "").strip().lower() for c in res.controller}
    return (
        states.get("joint_state_broadcaster") == "active"
        and states.get("arm_trajectory_controller") == "active"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Wait for FollowJointTrajectory action server (ros2_control / MoveIt)."
    )
    parser.add_argument(
        "--action",
        default="/arm_trajectory_controller/follow_joint_trajectory",
        help="Fully-qualified action name (default: arm trajectory in Gazebo stack)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=180.0,
        help="Give up after this many seconds (default: 180)",
    )
    parser.add_argument(
        "--controller-manager",
        default="/controller_manager",
        help="Controller manager namespace/prefix (default: /controller_manager)",
    )
    parser.add_argument(
        "--poll",
        type=float,
        default=1.0,
        help="Seconds to wait per wait_for_server call (default: 1.0)",
    )
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node("wait_for_arm_trajectory_action")
    client = ActionClient(node, FollowJointTrajectory, args.action)
    cm_path = args.controller_manager.rstrip("/") or "/controller_manager"
    lc_client = node.create_client(ListControllers, f"{cm_path}/list_controllers")

    deadline = time.monotonic() + max(1.0, args.timeout)
    node.get_logger().info(
        "Waiting for joint_state_broadcaster + arm_trajectory_controller ACTIVE, then action %s "
        "(timeout %.1f s)..."
        % (args.action, args.timeout)
    )

    try:
        while time.monotonic() < deadline:
            if not _ros_controllers_active(node, lc_client):
                node.get_logger().info(
                    "Still waiting for both ros2_control controllers to be active ..."
                )
                rclpy.spin_once(node, timeout_sec=min(args.poll, 0.25))
                continue
            rem = max(0.1, deadline - time.monotonic())
            if client.wait_for_server(timeout_sec=min(args.poll, rem)):
                node.get_logger().info("Action server ready: %s" % args.action)
                return 0
            node.get_logger().info(
                "Controllers active; still waiting for action server %s ..." % args.action
            )
            rclpy.spin_once(node, timeout_sec=0.05)

        node.get_logger().error(
            "Timed out after %.1f s waiting for %s" % (args.timeout, args.action)
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
