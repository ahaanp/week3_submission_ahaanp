#!/usr/bin/env python3
"""
Q2: Signal Publisher Node (S1)

Publishes "green" on `/s1` topic for the first 10 seconds,
then continuously publishes "red" afterwards.

Message Type:
- std_msgs/String
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class S1Publisher(Node):
    def __init__(self):
        super().__init__('s1_publisher')
        # Create publisher to `/s1` topic
        self.publisher_ = self.create_publisher(String, '/s1', 10)
        # Save the time at which node started
        self.start_time = self.get_clock().now()
        # Timer to publish every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Calculate time since node started (in seconds)
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        # Publish "green" for the first 10 seconds, then "red"
        msg = String()
        if elapsed_time < 10:
            msg.data = 'green'
        else:
            msg.data = 'red'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing on /s1: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = S1Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()