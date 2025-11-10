#!/usr/bin/env python3
import sys, termios, tty, select, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray

HELP = "Keys: a/d body, w/s arm1, i/k arm2, j/l shovel, SPACE safe pose, q quit"

# --- Joint limits from your URDF ---
LIMS_MIN = [float('-inf'), -1.308, -2.428, -2.395]  # body is continuous
LIMS_MAX = [float('inf'),  -0.087, -0.085, -0.357]
SAFE_POSE = [0.0, -0.5, -1.0, -1.0]  # inside limits

def clamp(q):
    return [max(LIMS_MIN[i], min(LIMS_MAX[i], q[i])) for i in range(4)]

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_excavator')

        # Built-in defaults so you don't need --ros-args
        default_topic = '/arm_position_controller/commands'
        default_step = 0.01       # <= your requested default
        default_rate_hz = 20.0

        # Still declare ROS params (optional override via params/YAML if you ever want)
        self.declare_parameter('topic', default_topic)
        self.declare_parameter('step', default_step)
        self.declare_parameter('rate_hz', default_rate_hz)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value or default_topic
        self.step  = float(self.get_parameter('step').get_parameter_value().double_value or default_step)
        rate_hz    = float(self.get_parameter('rate_hz').get_parameter_value().double_value or default_rate_hz)

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.q = SAFE_POSE.copy()

        self._stop = threading.Event()

        # Timer publishes at a steady rate; guard against publishing after stop
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(f"Publishing to {self.topic} at {rate_hz} Hz (step={self.step})")
        self.get_logger().info(HELP)

        # Non-blocking keyboard reader in a thread
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

    def _publish(self):
        # Avoid "Destroyable ... destruction was requested" by not using node after stop
        if self._stop.is_set():
            return
        self.pub.publish(Float64MultiArray(data=self.q))

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self._stop.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                c = sys.stdin.read(1)
                if c == 'q':
                    self.get_logger().info("Quit")
                    self._stop.set()
                    return
                elif c == ' ':
                    self.q = SAFE_POSE.copy()
                elif c == 'a':
                    self.q[0] += self.step
                elif c == 'd':
                    self.q[0] -= self.step
                elif c == 'w':
                    self.q[1] = clamp([self.q[0], self.q[1] + self.step, self.q[2], self.q[3]])[1]
                elif c == 's':
                    self.q[1] = clamp([self.q[0], self.q[1] - self.step, self.q[2], self.q[3]])[1]
                elif c == 'i':
                    self.q[2] = clamp([self.q[0], self.q[1], self.q[2] + self.step, self.q[3]])[2]
                elif c == 'k':
                    self.q[2] = clamp([self.q[0], self.q[1], self.q[2] - self.step, self.q[3]])[2]
                elif c == 'j':
                    self.q[3] = clamp([self.q[0], self.q[1], self.q[2], self.q[3] + self.step])[3]
                elif c == 'l':
                    self.q[3] = clamp([self.q[0], self.q[1], self.q[2], self.q[3] - self.step])[3]
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = Teleop()
    exec = SingleThreadedExecutor()
    exec.add_node(node)
    try:
        while rclpy.ok() and not node._stop.is_set():
            exec.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop publishing before destroying the node to avoid "Destroyable..." warnings
        try:
            node._stop.set()
            if node.timer is not None:
                node.timer.cancel()
        except Exception:
            pass
        try:
            if node._kb_thread.is_alive():
                node._kb_thread.join(timeout=0.5)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

