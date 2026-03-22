#!/usr/bin/env python3
"""Publish a coarse box CollisionObject for the dump truck into MoveIt's planning scene."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class TruckCollisionPublisher(Node):
    def __init__(self):
        super().__init__("truck_collision_object_publisher")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("object_id", "dump_truck_box")
        # Box center + full lengths (SolidPrimitive.BOX). Defaults avoid engulfing the excavator at the origin:
        # old lx=7 at px=4 spanned x in [0.5,7.5] and made dump_truck_box collide with shovel in home reach.
        self.declare_parameter("position_x", 6.25)
        self.declare_parameter("position_y", 3.0)
        self.declare_parameter("position_z", 0.95)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("box_length_x", 4.5)
        self.declare_parameter("box_length_y", 2.6)
        self.declare_parameter("box_length_z", 1.7)
        self.declare_parameter("republish_period_sec", 5.0)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(CollisionObject, "/collision_object", qos)
        period = float(self.get_parameter("republish_period_sec").value)
        self._timer = self.create_timer(period, self._publish)
        self._publish()

    def _publish(self):
        fid = self.get_parameter("frame_id").get_parameter_value().string_value
        oid = self.get_parameter("object_id").get_parameter_value().string_value
        px = self.get_parameter("position_x").get_parameter_value().double_value
        py = self.get_parameter("position_y").get_parameter_value().double_value
        pz = self.get_parameter("position_z").get_parameter_value().double_value
        yaw = self.get_parameter("yaw").get_parameter_value().double_value
        lx = self.get_parameter("box_length_x").get_parameter_value().double_value
        ly = self.get_parameter("box_length_y").get_parameter_value().double_value
        lz = self.get_parameter("box_length_z").get_parameter_value().double_value

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [lx, ly, lz]

        pose = Pose()
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.w = math.cos(yaw * 0.5)
        pose.orientation.z = math.sin(yaw * 0.5)

        msg = CollisionObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = fid
        msg.id = oid
        msg.operation = CollisionObject.ADD
        msg.primitives = [prim]
        msg.primitive_poses = [pose]

        self._pub.publish(msg)
        self.get_logger().info(
            "Published truck collision object '%s' in %s at (%.2f, %.2f, %.2f)"
            % (oid, fid, px, py, pz)
        )


def main():
    rclpy.init()
    node = TruckCollisionPublisher()
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
