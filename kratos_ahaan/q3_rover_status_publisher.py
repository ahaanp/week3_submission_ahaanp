#!/usr/bin/env python3
"""
ROS 2 node to publish Mars Rover status using a custom message.
Includes:
- Velocity (Twist)
- Distance Traveled
- Position (Pose)
- Battery Level
- Time of Travel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from builtin_interfaces.msg import Duration
from kratos_ahaan.msg import RoverStatus  # Custom message import
import random
import time

class RoverStatusPublisher(Node):
    def __init__(self):
        super().__init__('rover_status_publisher')

        # Publisher to topic `/rover_status`
        self.publisher_ = self.create_publisher(RoverStatus, '/rover_status', 10)

        # Timer to publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_status)

        # Start time to calculate travel time
        self.start_time = self.get_clock().now()

        # Simulated distance and battery
        self.distance = 0.0
        self.battery = 100.0

    def publish_status(self):
        msg = RoverStatus()

        # Simulated velocity values (random for demo)
        msg.velocity.linear.x = random.uniform(0.0, 1.0)
        msg.velocity.angular.z = random.uniform(-0.5, 0.5)

        # Accumulate distance over time
        self.distance += msg.velocity.linear.x
        msg.distance_traveled = self.distance

        # Simulated pose (random for demo)
        msg.position.position.x = self.distance
        msg.position.position.y = random.uniform(-5.0, 5.0)

        # Simulated battery drain
        self.battery -= 0.1
        msg.battery_level = max(0.0, self.battery)

        # Calculate travel time since start
        duration = self.get_clock().now() - self.start_time
        msg.travel_time = Duration(sec=int(duration.nanoseconds / 1e9))

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published RoverStatus → Battery: {msg.battery_level:.1f}%, Distance: {msg.distance_traveled:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = RoverStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()