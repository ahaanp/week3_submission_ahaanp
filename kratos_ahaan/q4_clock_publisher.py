#!/usr/bin/env python3
"""
ROS 2 Clock Publisher Node

Publishes:
- /second : current seconds count [0-59]
- /minute : current minutes count [0-59]
- /hour   : current hours count [0+]
- /clock  : formatted time as string "HH:MM:SS"

Message Types:
- /second, /minute, /hour → std_msgs/Int32
- /clock → std_msgs/String
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')

        # Create publishers for hour, minute, second, and full time
        self.second_pub = self.create_publisher(Int32, '/second', 10)
        self.minute_pub = self.create_publisher(Int32, '/minute', 10)
        self.hour_pub = self.create_publisher(Int32, '/hour', 10)
        self.clock_pub = self.create_publisher(String, '/clock', 10)

        # Initialize time counters
        self.seconds = 0
        self.minutes = 0
        self.hours = 0

        # Timer triggers every 1 second
        self.timer = self.create_timer(1.0, self.update_time)

    def update_time(self):
        """
        Called every second. Updates time counters and publishes them.
        """
        self.seconds += 1

        # Handle minute rollover
        if self.seconds >= 60:
            self.seconds = 0
            self.minutes += 1

        # Handle hour rollover
        if self.minutes >= 60:
            self.minutes = 0
            self.hours += 1

        # Create and publish Int32 messages
        sec_msg = Int32()
        min_msg = Int32()
        hr_msg = Int32()

        sec_msg.data = self.seconds
        min_msg.data = self.minutes
        hr_msg.data = self.hours

        self.second_pub.publish(sec_msg)
        self.minute_pub.publish(min_msg)
        self.hour_pub.publish(hr_msg)

        # Format and publish full time string
        clock_msg = String()
        clock_msg.data = f"{self.hours:02d}:{self.minutes:02d}:{self.seconds:02d}"
        self.clock_pub.publish(clock_msg)

        self.get_logger().info(f'Time: {clock_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()