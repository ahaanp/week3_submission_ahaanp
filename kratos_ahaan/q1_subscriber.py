#!/usr/bin/env python3
"""
Q1: Subscriber Node

This ROS 2 node subscribes to the `/new` topic and prints the received
messages to the terminal.

Message Type:
- std_msgs/String
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloSubscriber(Node):
    def __init__(self):
        super().__init__('hello_subscriber')
        # Create a subscription to `/new` topic using String messages
        self.subscription = self.create_subscription(
            String,
            '/new',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        # Print the received message
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = HelloSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
