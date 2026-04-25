#!/usr/bin/env python3
"""
obstacle_detector.py
Reads LaserScan data and publishes True on /obstacle_detected
when anything is closer than OBSTACLE_THRESHOLD in the frontal arc.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np


class ObstacleDetector(Node):

    OBSTACLE_THRESHOLD = 0.5   # metres – stop / avoid if closer than this
    FRONTAL_ARC_DEG   = 30     # ±30° in front of robot

    def __init__(self):
        super().__init__('obstacle_detector')

        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('ObstacleDetector node started.')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        # Replace inf/nan with a large number
        ranges = np.where(np.isfinite(ranges), ranges, 10.0)

        total = len(ranges)
        arc = int(self.FRONTAL_ARC_DEG / 360.0 * total)

        # Front arc: last `arc` beams + first `arc` beams (wraps around 0°)
        front = np.concatenate([ranges[-arc:], ranges[:arc]])

        obstacle = bool(np.min(front) < self.OBSTACLE_THRESHOLD)

        self.obstacle_pub.publish(Bool(data=obstacle))

        if obstacle:
            self.get_logger().warn(
                f'Obstacle! min_dist={np.min(front):.2f} m'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
