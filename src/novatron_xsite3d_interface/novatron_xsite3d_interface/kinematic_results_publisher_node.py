import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import threading
import time
from novatron_xsite3d_interface.fb.xcrtipcmsg.fb.KinematicData import KinematicData
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion


class KinematicResultsPublisherNode(Node):
    """ROS2 node that subscribes to MQTT realtime kinematic data and publishes TF transforms."""
    
    def __init__(self):
        super().__init__('kinematic_results_publisher_node')
        
        # Declare and get parameters
        self.declare_parameter('mqtt_host', '127.0.0.1')
        self.declare_parameter('mqtt_port', 0)
        self.declare_parameter('mqtt_topic', 'ask_from_novatron')
        self.declare_parameter('mqtt_username', 'ask_from_novatron')
        self.declare_parameter('mqtt_password', 'ask_from_novatron')
        
        self.mqtt_host = self.get_parameter('mqtt_host').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.mqtt_topic = self.get_parameter('mqtt_topic').get_parameter_value().string_value
        self.mqtt_username = self.get_parameter('mqtt_username').get_parameter_value().string_value
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value
        
        # Shutdown flag
        self._shutdown_requested = False
        
        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Setup MQTT client with unique client_id to avoid conflicts
        unique_client_id = f"ros2_kinematic_realtime_{int(time.time() * 1000)}"
        print(f"Initializing MQTT client with ID: {unique_client_id}")
        self.mqtt_client = mqtt.Client(client_id=unique_client_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        
        # Start MQTT connection in background thread
        self.mqtt_thread = threading.Thread(target=self.mqtt_loop, daemon=True)
        self.mqtt_client.connect_async(self.mqtt_host, self.mqtt_port, keepalive=60)
        self.mqtt_thread.start()
        
        self.get_logger().info(f"MQTT client initialized with ID: {unique_client_id}")

    def on_connect(self, client, userdata, flags, rc):
        """Callback when MQTT connection is established."""
        self.get_logger().info(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(self.mqtt_topic)
        
    def on_disconnect(self, client, userdata, rc):
        """Callback when MQTT connection is lost."""
        self.get_logger().info(f"Disconnected from MQTT broker, reason code: {rc}")

    def on_message(self, client, userdata, message):
        """Process incoming MQTT messages and publish TF transforms."""
        now = self.get_clock().now().to_msg()
        
        try:
            # Parse FlatBuffer message as KinematicData
            kinematic_data = KinematicData.GetRootAs(message.payload, 0)
            
            # Access world_offset (Vector3d struct)
            world_offset = kinematic_data.WorldOffset()
            if world_offset is None:
                self.get_logger().warn("No world offset in message", throttle_duration_sec=10.0)
                return
            
            # Publish world offset transform
            # Coordinate transformation: swap X/Y and negate X
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "world"
            t.child_frame_id = "world_offset"
            t.transform.translation.x = world_offset.X()
            t.transform.translation.y = world_offset.Y()
            t.transform.translation.z = world_offset.Z()
            t.transform.rotation.w = 1.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            
            self.tf_broadcaster.sendTransform(t)
            
            # Check if results array exists
            if kinematic_data.ResultsLength() == 0:
                return
            
            # Publish transforms for each kinematic result
            for i in range(kinematic_data.ResultsLength()):
                result = kinematic_data.Results(i)
                if result is None:
                    continue
                
                # Access origin (Vector3 table)
                origin = result.Origin()
                if origin is None:
                    continue
                    
                # Access orientation (Quat table)
                orientation = result.Orientation()
                if orientation is None:
                    continue
                
                t.header.frame_id = "world_offset"
                t.child_frame_id = f"kinematic_{result.Id()}"
                t.transform.translation.x = origin.X()
                t.transform.translation.y = origin.Y()
                t.transform.translation.z = origin.Z()
                t.transform.rotation.x = orientation.X()
                t.transform.rotation.y = orientation.Y()
                t.transform.rotation.z = orientation.Z()
                t.transform.rotation.w = orientation.W()
                self.tf_broadcaster.sendTransform(t)
                
        except Exception as e:
            # Ignore errors during node shutdown
            if "destruction was requested" not in str(e):
                self.get_logger().error(f"Failed to process message: {e}")

    def mqtt_loop(self):
        """Run MQTT client loop in background thread."""
        self.mqtt_client.loop_forever()
    
    def shutdown(self):
        """Clean shutdown of MQTT client and node resources."""
        if self._shutdown_requested:
            return
        
        self._shutdown_requested = True
        print('Shutting down node...')
        
        # Stop MQTT client
        try:
            self.mqtt_client.disconnect()
            self.mqtt_client.loop_stop()
            print('MQTT client disconnected')
        except Exception as e:
            print(f'Error disconnecting MQTT: {e}')
        
        # Wait for MQTT thread to finish
        if self.mqtt_thread.is_alive():
            self.mqtt_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicResultsPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown of MQTT and node resources
        node.shutdown()
        node.destroy_node()
        
        # Only shutdown rclpy if context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
