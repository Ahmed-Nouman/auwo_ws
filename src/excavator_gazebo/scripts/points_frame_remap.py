#!/usr/bin/env python3
"""
Republish PointCloud2 from /points with header.frame_id set to a TF-known frame
so RViz can transform to world (Gazebo often uses frames like excavator/body/lidar_sensor).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def main(args=None):
    rclpy.init(args=args)
    node = Node("points_frame_remap")

    node.declare_parameter("target_frame_id", "sensor_lidar_link")
    node.declare_parameter("input_topic", "/points")
    node.declare_parameter("output_topic", "/points_viz")

    target_frame = node.get_parameter("target_frame_id").value
    in_topic = node.get_parameter("input_topic").value
    out_topic = node.get_parameter("output_topic").value

    pub = node.create_publisher(PointCloud2, out_topic, 10)

    def cb(msg):
        msg.header.frame_id = str(target_frame)
        pub.publish(msg)

    node.create_subscription(PointCloud2, in_topic, cb, 10)
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
