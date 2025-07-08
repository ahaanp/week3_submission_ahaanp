#!/usr/bin/env python3
"""
Q2: Signal Switcher Node (S2)

This node subscribes to topic `/s1` and publishes the inverse signal to `/s2`.

- If `/s1` = "green", `/s2` = "red"
- If `/s1` = "red", `/s2` = "green"

Message Type:
- std_msgs/String
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class S2Publisher(Node):
    def __init__(self):
        super().__init__('s2_publisher')
        # Subscribe to `/s1`
        self.subscription = self.create_subscription(
            String,
            '/s1',
            self.listener_callback,
            10)
        # Publisher to `/s2`
        self.publisher_ = self.create_publisher(String, '/s2', 10)

    def listener_callback(self, msg):
        # Prepare inverse signal
        out_msg = String()
        if msg.data == 'green':
            out_msg.data = 'red'
        else:
            out_msg.data = 'green'
        # Publish to `/s2`
        self.publisher_.publish(out_msg)
        self.get_logger().info(f'S1 is "{msg.data}" → Publishing on /s2: "{out_msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = S2Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()