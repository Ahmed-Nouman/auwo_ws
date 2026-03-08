#!/usr/bin/env python3
"""
Publish geometry_msgs/PoseStamped from sensor_msgs/Imu for RViz visualization.
Subscribes to /imu, publishes /imu_pose with orientation from IMU and fixed position.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


def main(args=None):
    rclpy.init(args=args)
    node = Node("imu_to_pose")

    node.declare_parameter("pose_frame_id", "world")
    node.declare_parameter("position_x", 0.5)
    node.declare_parameter("position_y", 0.0)
    node.declare_parameter("position_z", 1.5)

    frame_id = node.get_parameter("pose_frame_id").value
    px = node.get_parameter("position_x").value
    py = node.get_parameter("position_y").value
    pz = node.get_parameter("position_z").value

    pub = node.create_publisher(PoseStamped, "/imu_pose", 10)

    def cb(msg):
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = str(frame_id)
        out.pose.position.x = float(px)
        out.pose.position.y = float(py)
        out.pose.position.z = float(pz)
        out.pose.orientation.x = float(msg.orientation.x)
        out.pose.orientation.y = float(msg.orientation.y)
        out.pose.orientation.z = float(msg.orientation.z)
        out.pose.orientation.w = float(msg.orientation.w)
        pub.publish(out)

    node.create_subscription(Imu, "/imu", cb, 10)
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
