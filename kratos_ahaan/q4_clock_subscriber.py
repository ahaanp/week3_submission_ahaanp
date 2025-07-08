#!/usr/bin/env python3
"""
Clock Subscriber Node for Q4

Subscribes to `/clock` topic and prints the time. This ensures `/clock`
appears in rqt_graph as it creates a real subscriber node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ClockSubscriber(Node):
    def __init__(self):
        super().__init__('clock_subscriber')

        self.subscription = self.create_subscription(
            String,
            '/clock',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Clock time received: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ClockSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()