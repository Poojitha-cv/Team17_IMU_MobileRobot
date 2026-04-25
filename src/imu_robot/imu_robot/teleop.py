#!/usr/bin/env python3
"""
teleop.py — Auto-drives the robot forward onto the ramp.
Run in a second terminal after launching imu_demo.launch.py

Usage:
    python3 teleop.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def drive_forward(self, speed=0.3, duration=4.0):
        self.get_logger().info(f'Driving forward at {speed} m/s for {duration}s...')
        msg = Twist()
        msg.linear.x = speed
        end_time = time.time() + duration
        while time.time() < end_time:
            self.pub.publish(msg)
            time.sleep(0.1)
        # stop
        self.pub.publish(Twist())
        self.get_logger().info('Done. Check tilt_detector terminal for [TILT] DETECTED.')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    input('Press ENTER to auto-drive robot onto the ramp...\n')
    node.drive_forward(speed=0.3, duration=4.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
