#!/usr/bin/env python3
"""
physical_tf_follower_node.py (v11 — fixed logging + stable TF)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState


# -------------------------
# helpers
# -------------------------

def pitch_from_quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(
        2.0 * (w * y - z * x),
        math.sqrt((1 - 2 * (y * y + z * z))**2 + (2 * (w * z + x * y))**2)
    )


def yaw_from_quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    )


def continuous_angle(new_angle, prev_angle):
    if prev_angle is None:
        return new_angle

    candidates = [
        new_angle,
        new_angle + 2 * math.pi,
        new_angle - 2 * math.pi,
    ]
    return min(candidates, key=lambda a: abs(a - prev_angle))


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


# -------------------------
# main node
# -------------------------

class PhysicalTfFollowerNode(Node):

    def __init__(self):
        super().__init__('physical_tf_follower_node')

        self.declare_parameter('mirror_to_joint_states', True)
        self.declare_parameter('publish_world_to_base_tf', True)
        self.declare_parameter('publish_rate', 30.0)

        self.declare_parameter('frame_world_offset', 'world_offset')
        self.declare_parameter('frame_body', 'kinematic_1005')
        self.declare_parameter('frame_boom', 'kinematic_1003')
        self.declare_parameter('frame_stick', 'kinematic_1002')
        self.declare_parameter('frame_bucket', 'kinematic_2001')

        self.declare_parameter('body_sign', 1.0)
        self.declare_parameter('boom_sign', 1.0)
        self.declare_parameter('stick_sign', 1.0)
        self.declare_parameter('bucket_sign', 1.0)

        self.declare_parameter('base_link_roll_correction', 0.0)
        self.declare_parameter('boom_offset', -2.30)

        self._mirror = self.get_parameter('mirror_to_joint_states').value
        self._pub_world = self.get_parameter('publish_world_to_base_tf').value
        rate_hz = self.get_parameter('publish_rate').value

        self._f_world = self.get_parameter('frame_world_offset').value
        self._f_body = self.get_parameter('frame_body').value
        self._f_boom = self.get_parameter('frame_boom').value
        self._f_stick = self.get_parameter('frame_stick').value
        self._f_bucket = self.get_parameter('frame_bucket').value

        self._s_body = self.get_parameter('body_sign').value
        self._s_boom = self.get_parameter('boom_sign').value
        self._s_stick = self.get_parameter('stick_sign').value
        self._s_bucket = self.get_parameter('bucket_sign').value

        self._roll_corr = self.get_parameter('base_link_roll_correction').value
        self._boom_offset = self.get_parameter('boom_offset').value

        self._prev_body = None
        self._prev_boom = None
        self._prev_stick = None
        self._prev_bucket = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        if self._mirror:
            self._js_pub = self.create_publisher(JointState, '/joint_states', 10)

        self._timer = self.create_timer(1.0 / rate_hz, self._update)

        self.get_logger().info("Physical TF follower v11 started")

    def _lookup(self, parent, child):
        try:
            return self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"TF {parent}->{child}: {e}", throttle_duration_sec=5.0)
            return None

    def _apply(self, raw, sign, prev):
        raw = sign * raw
        return continuous_angle(raw, prev)

    # -------------------------
    # BODY YAW (XY arc method)
    # -------------------------
    def _compute_body_yaw(self):
        body = self._lookup(self._f_world, self._f_body)
        stick = self._lookup(self._f_world, self._f_stick)

        if not body or not stick:
            return None

        bx, by = body.transform.translation.x, body.transform.translation.y
        sx, sy = stick.transform.translation.x, stick.transform.translation.y

        angle = math.atan2(sy - by, sx - bx)

        yaw = self._apply(angle, self._s_body, self._prev_body)
        self._prev_body = yaw
        return yaw

    # -------------------------
    # update loop
    # -------------------------
    def _update(self):

        now = self.get_clock().now().to_msg()

        body_yaw = self._compute_body_yaw()

        boom = self._lookup(self._f_body, self._f_boom)
        boom_angle = None
        if boom:
            boom_angle = self._apply(
                -pitch_from_quat(boom.transform.rotation) + self._boom_offset,
                self._s_boom,
                self._prev_boom
            )
            self._prev_boom = boom_angle

        stick = self._lookup(self._f_body, self._f_stick)
        stick_angle = None
        if stick:
            stick_angle = self._apply(
                pitch_from_quat(stick.transform.rotation),
                self._s_stick,
                self._prev_stick
            )
            self._prev_stick = stick_angle

        bucket = self._lookup(self._f_body, self._f_bucket)
        bucket_angle = None
        if bucket:
            bucket_angle = self._apply(
                pitch_from_quat(bucket.transform.rotation),
                self._s_bucket,
                self._prev_bucket
            )
            self._prev_bucket = bucket_angle

        # -------------------------
        # JointState publish
        # -------------------------
        if self._mirror:
            js = JointState()
            js.header.stamp = now

            names = []
            values = []

            if body_yaw is not None:
                names.append("body_rotation")
                values.append(body_yaw)

            if boom_angle is not None:
                names.append("boom_rotation")
                values.append(boom_angle)

            if stick_angle is not None:
                names.append("stick_rotation")
                values.append(stick_angle)

            if bucket_angle is not None:
                names.append("bucket_rotation")
                values.append(bucket_angle)

            if names:
                js.name = names
                js.position = values
                js.velocity = [0.0] * len(names)
                js.effort = [0.0] * len(names)
                self._js_pub.publish(js)

                # -------- DEGREE LOGGING (RESTORED) --------
                msg = []
                if body_yaw is not None:
                    msg.append(f"body={math.degrees(body_yaw):.1f}°")
                if boom_angle is not None:
                    msg.append(f"boom={math.degrees(boom_angle):.1f}°")
                if stick_angle is not None:
                    msg.append(f"stick={math.degrees(stick_angle):.1f}°")
                if bucket_angle is not None:
                    msg.append(f"bucket={math.degrees(bucket_angle):.1f}°")

                self.get_logger().info("  ".join(msg), throttle_duration_sec=1.0)

        # -------------------------
        # world → base_link
        # -------------------------
        if self._pub_world:
            base = self._lookup(self._f_world, self._f_body)

            if base and body_yaw is not None:
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = "world"
                t.child_frame_id = "base_link"

                t.transform.translation = base.transform.translation
                t.transform.rotation = rpy_to_quat(
                    self._roll_corr,
                    0.0,
                    body_yaw
                )

                self._tf_broadcaster.sendTransform(t)


# -------------------------
# main
# -------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PhysicalTfFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
