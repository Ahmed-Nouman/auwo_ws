#!/usr/bin/env python3
"""
Publish excavator and truck robot_description to separate topics for RViz.
Reads URDF from files (paths passed as parameters) to avoid parameter size limits.
Also publish excavator to /robot_description so gz_ros_control gets the correct URDF.
"""
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

# Minimal truck URDF so /truck_robot_description topic always exists (single link at origin)
MINIMAL_TRUCK_URDF = """<?xml version="1.0"?>
<robot name="truck">
  <link name="base_link">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
  </link>
</robot>
"""


def main(args=None):
    rclpy.init(args=args)
    node = Node("publish_robot_descriptions")

    node.declare_parameter("excavator_description_file", "")
    node.declare_parameter("truck_description_file", "")

    excavator_path = node.get_parameter("excavator_description_file").value
    truck_path = node.get_parameter("truck_description_file").value

    # Handle list from launch (e.g. [path]) and strip
    if isinstance(excavator_path, list):
        excavator_path = excavator_path[0] if excavator_path else ""
    if isinstance(truck_path, list):
        truck_path = truck_path[0] if truck_path else ""
    if isinstance(excavator_path, str):
        excavator_path = excavator_path.strip()
    if isinstance(truck_path, str):
        truck_path = truck_path.strip()

    # Fallback if param empty (e.g. not expanded)
    if not truck_path:
        truck_path = "/tmp/truck_rviz.urdf"
        node.get_logger().warning("truck_description_file was empty, using fallback: %s" % truck_path)

    excavator = ""
    truck = ""
    if excavator_path:
        try:
            with open(excavator_path, "r") as f:
                excavator = f.read()
            node.get_logger().info("Loaded excavator URDF from %s (%d chars)" % (excavator_path, len(excavator)))
        except OSError as e:
            node.get_logger().error("Failed to read excavator file %s: %s" % (excavator_path, e))
    if truck_path and os.path.isfile(truck_path):
        try:
            with open(truck_path, "r") as f:
                truck = f.read()
            if "robot name=" in truck and len(truck) > 100:
                node.get_logger().info("Loaded truck URDF from %s (%d chars)" % (truck_path, len(truck)))
            else:
                truck = MINIMAL_TRUCK_URDF
                node.get_logger().warning("Truck file too short, using minimal URDF")
        except OSError as e:
            node.get_logger().error("Failed to read truck file %s: %s" % (truck_path, e))
            truck = MINIMAL_TRUCK_URDF
    else:
        node.get_logger().warning("Truck file not found: %s; publishing minimal truck URDF" % truck_path)
        truck = MINIMAL_TRUCK_URDF

    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    pub_excavator = node.create_publisher(String, "/excavator_robot_description", qos)
    pub_truck = node.create_publisher(String, "/truck_robot_description", qos)
    pub_robot_description = node.create_publisher(String, "/robot_description", qos)

    def publish_all():
        if excavator:
            msg = String()
            msg.data = excavator
            pub_excavator.publish(msg)
            # Only publish to /robot_description if this URDF has ros2_control (excavator has it, truck does not)
            if "ros2_control" in excavator:
                pub_robot_description.publish(msg)
        # Always publish truck so topic exists (full URDF or minimal)
        msg = String()
        msg.data = truck if truck else MINIMAL_TRUCK_URDF
        pub_truck.publish(msg)

    # Publish immediately for RViz
    if excavator:
        pub_excavator.publish(String(data=excavator))
    pub_truck.publish(String(data=truck if truck else MINIMAL_TRUCK_URDF))
    node.get_logger().info("Published robot descriptions (excavator and truck)")

    # Keep /robot_description with excavator (has ros2_control); something may publish truck there on spawn
    def publish_robot_description_for_control():
        if excavator and "ros2_control" in excavator:
            pub_robot_description.publish(String(data=excavator))

    publish_robot_description_for_control()
    node.create_timer(3.0, publish_robot_description_for_control)
    node.create_timer(5.0, publish_robot_description_for_control)
    node.create_timer(7.0, publish_robot_description_for_control)
    # Every 1s overwrite /robot_description so gz_ros_control always gets excavator if it subscribes late
    node.create_timer(1.0, publish_robot_description_for_control)

    # Republish every 2s for RViz
    node.create_timer(2.0, publish_all)

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
