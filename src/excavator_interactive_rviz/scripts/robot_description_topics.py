#!/usr/bin/env python3
"""
Publish robot_description from each robot_state_publisher to separate topics
so RViz can show both excavator and truck (each RobotModel on its own topic).
Uses GetParameters service to read from /robot_state_publisher and
/robot_state_publisher_truck, then publishes to /excavator_robot_description
and /truck_robot_description with latched QoS.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import ParameterType  # PARAMETER_STRING = 4


class RobotDescriptionTopicsNode(Node):
    def __init__(self):
        super().__init__("robot_description_topics")
        # Latched publisher so RViz gets the message when it subscribes late
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub_excavator = self.create_publisher(String, "/excavator_robot_description", qos)
        self.pub_truck = self.create_publisher(String, "/truck_robot_description", qos)
        self.timer = self.create_timer(1.0, self._publish_once)
        self._done = False
        self._excavator_desc = None
        self._truck_desc = None

    def _republish(self):
        if self._excavator_desc:
            msg = String()
            msg.data = self._excavator_desc
            self.pub_excavator.publish(msg)
        if self._truck_desc:
            msg = String()
            msg.data = self._truck_desc
            self.pub_truck.publish(msg)

    def _get_param_from_node(self, node_name: str, param_name: str):
        client = self.create_client(GetParameters, f"/{node_name}/get_parameters")
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        req = GetParameters.Request()
        req.names = [param_name]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or not future.result():
            return None
        res = future.result()
        if not res.values or res.values[0].type != ParameterType.PARAMETER_STRING:
            return None
        return res.values[0].string_value

    def _publish_once(self):
        if self._done:
            return
        excavator_desc = self._get_param_from_node("robot_state_publisher", "robot_description")
        if excavator_desc:
            msg = String()
            msg.data = excavator_desc
            self.pub_excavator.publish(msg)
            self.get_logger().info("Published excavator robot_description")
        truck_desc = self._get_param_from_node("robot_state_publisher_truck", "robot_description")
        if truck_desc:
            msg = String()
            msg.data = truck_desc
            self.pub_truck.publish(msg)
            self.get_logger().info("Published truck robot_description")
        if excavator_desc:
            self._excavator_desc = excavator_desc
            if truck_desc is not None:
                self._truck_desc = truck_desc
            if not self._done:
                self._done = True
                self.timer.cancel()
                self.timer = self.create_timer(10.0, self._republish)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionTopicsNode()
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
