#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PhysicalTfFollowerNode(Node):
    """Convert Novatron TF stream to excavator joint states."""

    def __init__(self) -> None:
        super().__init__("physical_tf_follower_node")

        # Frames from novatron_xsite3d_interface
        self.declare_parameter("world_frame", "world_offset")
        self.declare_parameter("base_frame", "kinematic_1005")
        self.declare_parameter("boom_joint_frame", "kinematic_1003")
        self.declare_parameter("stick_joint_frame", "kinematic_1002")
        self.declare_parameter("bucket_joint_frame", "kinematic_1001")

        # Outputs
        self.declare_parameter("publish_topic", "/physical_twin/state")
        self.declare_parameter("mirror_to_joint_states", True)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("publish_world_to_base_tf", True)
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 30.0)

        # Joint mapping knobs (different machines/installations can need sign/offset tuning)
        self.declare_parameter("body_sign", 1.0)
        self.declare_parameter("boom_sign", -1.0)
        self.declare_parameter("stick_sign", -1.0)
        self.declare_parameter("bucket_sign", -1.0)

        self.declare_parameter("body_offset", 0.0)
        self.declare_parameter("boom_offset", -0.70)
        self.declare_parameter("stick_offset", -1.26)
        self.declare_parameter("bucket_offset", -1.12)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.boom_joint_frame = self.get_parameter("boom_joint_frame").value
        self.stick_joint_frame = self.get_parameter("stick_joint_frame").value
        self.bucket_joint_frame = self.get_parameter("bucket_joint_frame").value

        self.state_topic = self.get_parameter("publish_topic").value
        self.mirror_to_joint_states = bool(self.get_parameter("mirror_to_joint_states").value)
        self.joint_states_topic = self.get_parameter("joint_states_topic").value
        self.publish_world_to_base_tf = bool(self.get_parameter("publish_world_to_base_tf").value)
        self.base_link_frame = self.get_parameter("base_link_frame").value

        self.body_sign = float(self.get_parameter("body_sign").value)
        self.boom_sign = float(self.get_parameter("boom_sign").value)
        self.stick_sign = float(self.get_parameter("stick_sign").value)
        self.bucket_sign = float(self.get_parameter("bucket_sign").value)

        self.body_offset = float(self.get_parameter("body_offset").value)
        self.boom_offset = float(self.get_parameter("boom_offset").value)
        self.stick_offset = float(self.get_parameter("stick_offset").value)
        self.bucket_offset = float(self.get_parameter("bucket_offset").value)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._timer_period = 1.0 / max(rate_hz, 1.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.physical_state_pub = self.create_publisher(JointState, self.state_topic, 10)
        self.joint_states_pub = self.create_publisher(JointState, self.joint_states_topic, 10)

        self.joint_names = [
            "body_rotation",
            "boom_rotation",
            "stick_rotation",
            "bucket_rotation",
        ]

        self.create_timer(self._timer_period, self._on_timer)
        self.get_logger().info("Physical TF follower started.")

    def _lookup(self, target: str, source: str) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None

    def _on_timer(self) -> None:
        t_world_to_base = self._lookup(self.world_frame, self.base_frame)
        t_base_to_boom = self._lookup(self.base_frame, self.boom_joint_frame)
        t_base_to_stick = self._lookup(self.base_frame, self.stick_joint_frame)
        t_base_to_bucket = self._lookup(self.base_frame, self.bucket_joint_frame)

        if None in (t_world_to_base, t_base_to_boom, t_base_to_stick, t_base_to_bucket):
            return

        q_base = t_world_to_base.transform.rotation
        base_yaw_world = _yaw_from_quaternion(q_base.x, q_base.y, q_base.z, q_base.w)

        # Work in body yaw frame so boom/stick pitch is independent of global heading.
        c = math.cos(-base_yaw_world)
        s = math.sin(-base_yaw_world)

        p_boom = t_base_to_boom.transform.translation
        p_stick = t_base_to_stick.transform.translation
        p_bucket = t_base_to_bucket.transform.translation

        v_boom_x = p_stick.x - p_boom.x
        v_boom_y = p_stick.y - p_boom.y
        v_boom_z = p_stick.z - p_boom.z

        v_stick_x = p_bucket.x - p_stick.x
        v_stick_y = p_bucket.y - p_stick.y
        v_stick_z = p_bucket.z - p_stick.z

        boom_x_body = c * v_boom_x - s * v_boom_y
        stick_x_body = c * v_stick_x - s * v_stick_y

        boom_pitch_abs = math.atan2(v_boom_z, boom_x_body)
        stick_pitch_abs = math.atan2(v_stick_z, stick_x_body)
        stick_pitch_rel = _normalize_angle(stick_pitch_abs - boom_pitch_abs)

        q_bucket = t_base_to_bucket.transform.rotation
        bucket_yaw = _yaw_from_quaternion(q_bucket.x, q_bucket.y, q_bucket.z, q_bucket.w)
        bucket_pitch_rel = _normalize_angle(bucket_yaw - stick_pitch_abs)

        body = self.body_sign * base_yaw_world + self.body_offset
        boom = self.boom_sign * boom_pitch_abs + self.boom_offset
        stick = self.stick_sign * stick_pitch_rel + self.stick_offset
        bucket = self.bucket_sign * bucket_pitch_rel + self.bucket_offset

        now = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        js.position = [body, boom, stick, bucket]

        self.physical_state_pub.publish(js)
        if self.mirror_to_joint_states:
            self.joint_states_pub.publish(js)

        if self.publish_world_to_base_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.world_frame
            tf_msg.child_frame_id = self.base_link_frame
            tf_msg.transform = t_world_to_base.transform
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PhysicalTfFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
