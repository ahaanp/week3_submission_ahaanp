#!/usr/bin/env python3
"""
Q1: Publisher Node

This ROS 2 node publishes the string "Hello World !" to the `/new` topic
at a fixed rate of 15 messages per second.

Message Type:
- std_msgs/String
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        # Create a publisher to topic `/new` using String messages
        self.publisher_ = self.create_publisher(String, '/new', 10)
        # Timer to trigger publishing every 1/15 seconds (i.e., 15 Hz)
        timer_period = 1.0 / 15  # 15 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Prepare the message
        msg = String()
        msg.data = 'Hello World !'
        # Publish the message
        self.publisher_.publish(msg)
        # Log output to console
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()