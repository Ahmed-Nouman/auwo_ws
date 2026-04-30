#!/usr/bin/env python3
"""
physical_tf_follower_node.py  (v10 — correct mapping from absolute pitch analysis)
===================================================================================
Joint mapping confirmed from absolute orientation analysis of tf_dump:

  BOOM   (1005→1003): PITCH  -75.6° to -56.1°  range=19.6°
  STICK  (1005→1002): PITCH  -34.4° to +4.8°   range=39.2°  (absolute vs body)
  BUCKET (1005→2001): PITCH  +39.5° to +89.4°  range=49.8°  (absolute vs body)
  BODY   (world_offset→1000): YAW  0° (not moved in this bag)

Key insight: stick and bucket are best measured as absolute pitch relative to
the body frame (kinematic_1005), not as relative rotations between adjacent
sensor frames which suffer from gimbal lock.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState


def pitch_from_quat(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(
        2.0*(w*y - z*x),
        math.sqrt((1 - 2*(y*y + z*z))**2 + (2*(w*z + x*y))**2)
    )


def yaw_from_quat(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(
        2.0*(w*z + x*y),
        1.0 - 2.0*(y*y + z*z)
    )


def continuous_angle(new_angle: float, prev_angle: float) -> float:
    candidates = [
        new_angle,
        new_angle + 2*math.pi,
        new_angle - 2*math.pi,
    ]
    return min(candidates, key=lambda a: abs(a - prev_angle))


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
    q = Quaternion()
    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q


class PhysicalTfFollowerNode(Node):

    def __init__(self):
        super().__init__('physical_tf_follower_node')

        self.declare_parameter('mirror_to_joint_states',   True)
        self.declare_parameter('publish_world_to_base_tf', True)
        self.declare_parameter('publish_rate',             30.0)

        self.declare_parameter('frame_world_offset', 'world_offset')
        self.declare_parameter('frame_body_6dof',    'kinematic_1005')
        self.declare_parameter('frame_body_yaw',     'kinematic_1000')
        self.declare_parameter('frame_boom',         'kinematic_1003')
        self.declare_parameter('frame_stick',        'kinematic_1002')
        self.declare_parameter('frame_bucket',       'kinematic_2001')

        self.declare_parameter('body_sign',   1.0)
        self.declare_parameter('boom_sign',   1.0)
        self.declare_parameter('stick_sign',  1.0)
        self.declare_parameter('bucket_sign', 1.0)
        self.declare_parameter('base_link_roll_correction', 0.0)

        # boom_offset: corrects the position after negating boom pitch.
        # Formula: URDF_boom = -novatron_pitch + boom_offset
        # Default = 2 * bag-data midpoint (-65.85° = -1.149 rad) = -2.30 rad.
        # Fine-tune: set boom_offset = 2 * (current novatron boom reading)
        # if the rest position looks slightly off.
        self.declare_parameter('boom_offset', -2.30)

        self._mirror    = self.get_parameter('mirror_to_joint_states').value
        self._pub_world = self.get_parameter('publish_world_to_base_tf').value
        rate_hz         = self.get_parameter('publish_rate').value

        self._f_world  = self.get_parameter('frame_world_offset').value
        self._f_body   = self.get_parameter('frame_body_6dof').value
        self._f_yaw    = self.get_parameter('frame_body_yaw').value
        self._f_boom   = self.get_parameter('frame_boom').value
        self._f_stick  = self.get_parameter('frame_stick').value
        self._f_bucket = self.get_parameter('frame_bucket').value

        self._s_body   = self.get_parameter('body_sign').value
        self._s_boom   = self.get_parameter('boom_sign').value
        self._s_stick  = self.get_parameter('stick_sign').value
        self._s_bucket = self.get_parameter('bucket_sign').value
        self._roll_corr = self.get_parameter('base_link_roll_correction').value
        self._boom_offset = self.get_parameter('boom_offset').value

        self._prev_body   = None
        self._prev_boom   = None
        self._prev_stick  = None
        self._prev_bucket = None

        self._tf_buffer      = tf2_ros.Buffer()
        self._tf_listener    = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        if self._mirror:
            self._js_pub = self.create_publisher(JointState, '/joint_states', 10)

        self._timer = self.create_timer(1.0 / rate_hz, self._update)

        self.get_logger().info(
            f"PhysicalTfFollowerNode v10 started\n"
            f"  body:   {self._f_world}→{self._f_yaw}         (yaw)\n"
            f"  boom:   {self._f_body}→{self._f_boom}  (pitch)\n"
            f"  stick:  {self._f_body}→{self._f_stick}  (pitch vs body)\n"
            f"  bucket: {self._f_body}→{self._f_bucket}  (pitch vs body)\n"
            f"  signs: body={self._s_body} boom={self._s_boom} "
            f"stick={self._s_stick} bucket={self._s_bucket}"
        )

    def _lookup(self, parent: str, child: str):
        try:
            return self._tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF {parent}→{child}: {e}",
                throttle_duration_sec=5.0
            )
            return None

    def _apply(self, raw, sign, prev):
        raw = sign * raw
        if prev is not None:
            raw = continuous_angle(raw, prev)
        return raw

    def _update(self):
        now = self.get_clock().now().to_msg()

        # body_rotation — yaw of kinematic_1000 in world_offset
        body_tf = self._lookup(self._f_world, self._f_yaw)
        body_yaw = None
        if body_tf:
            body_yaw = self._apply(
                yaw_from_quat(body_tf.transform.rotation),
                self._s_body, self._prev_body)
            self._prev_body = body_yaw

        # boom_rotation — Novatron ENU pitch convention is opposite to the URDF
        # Y-axis joint direction: physically raising the boom makes the Novatron
        # pitch less negative, but the URDF needs it more negative to go up.
        # Fix: negate the pitch and add boom_offset to restore correct position.
        # URDF_boom = -novatron_pitch + boom_offset
        boom_tf = self._lookup(self._f_body, self._f_boom)
        boom_angle = None
        if boom_tf:
            boom_angle = self._apply(
                -pitch_from_quat(boom_tf.transform.rotation) + self._boom_offset,
                self._s_boom, self._prev_boom)
            self._prev_boom = boom_angle

        # stick_rotation — pitch of kinematic_1002 relative to kinematic_1005
        # (absolute vs body avoids gimbal lock from adjacent frame lookup)
        stick_tf = self._lookup(self._f_body, self._f_stick)
        stick_angle = None
        if stick_tf:
            stick_angle = self._apply(
                pitch_from_quat(stick_tf.transform.rotation),
                self._s_stick, self._prev_stick)
            self._prev_stick = stick_angle

        # bucket_rotation — pitch of kinematic_2001 relative to kinematic_1005
        # (quick coupler frame, absolute vs body — confirmed 50° range of motion)
        bucket_tf = self._lookup(self._f_body, self._f_bucket)
        bucket_angle = None
        if bucket_tf:
            bucket_angle = self._apply(
                pitch_from_quat(bucket_tf.transform.rotation),
                self._s_bucket, self._prev_bucket)
            self._prev_bucket = bucket_angle

        # publish /joint_states
        if self._mirror:
            js = JointState()
            js.header.stamp = now
            joints, positions = [], []

            if body_yaw    is not None: joints.append('body_rotation');   positions.append(body_yaw)
            if boom_angle  is not None: joints.append('boom_rotation');   positions.append(boom_angle)
            if stick_angle is not None: joints.append('stick_rotation');  positions.append(stick_angle)
            if bucket_angle is not None:joints.append('bucket_rotation'); positions.append(bucket_angle)

            if joints:
                js.name     = joints
                js.position = positions
                js.velocity = [0.0] * len(joints)
                js.effort   = [0.0] * len(joints)
                self._js_pub.publish(js)

                parts = []
                if body_yaw    is not None: parts.append(f"body={math.degrees(body_yaw):.1f}°")
                if boom_angle  is not None: parts.append(f"boom={math.degrees(boom_angle):.1f}°")
                if stick_angle is not None: parts.append(f"stick={math.degrees(stick_angle):.1f}°")
                if bucket_angle is not None: parts.append(f"bucket={math.degrees(bucket_angle):.1f}°")
                if parts:
                    self.get_logger().info(
                        "  ".join(parts),
                        throttle_duration_sec=1.0
                    )

        # publish world → base_link
        if self._pub_world:
            b = self._lookup(self._f_world, self._f_body)
            if b:
                body_world_yaw = yaw_from_quat(b.transform.rotation)
                t = TransformStamped()
                t.header.stamp    = now
                t.header.frame_id = 'world'
                t.child_frame_id  = 'base_link'
                t.transform.translation = b.transform.translation
                t.transform.rotation = rpy_to_quat(
                    self._roll_corr, 0.0, body_world_yaw)
                self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalTfFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
