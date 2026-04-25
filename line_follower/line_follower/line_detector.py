#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineDetector(Node):

    def __init__(self):
        super().__init__('line_detector')
        self.bridge = CvBridge()
        self.error_pub = self.create_publisher(Float32, '/line_error', 10)
        self.debug_pub = self.create_publisher(Image, '/line_debug_image', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.get_logger().info('LineDetector started.')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # Use bottom 40% — floor region
        roi = frame[int(h * 0.6):h, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Threshold to isolate bright line from dark floor
        _, mask = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        error_msg = Float32()

        if contours:
            # At intersection multiple contours appear.
            # Pick the one whose centroid is closest to bottom-centre
            # (most directly ahead) — this keeps the robot going straight.
            roi_cx = roi.shape[1] / 2
            roi_cy = roi.shape[0]   # bottom of roi

            best = None
            best_score = float('inf')

            for c in contours:
                if cv2.contourArea(c) < 150:
                    continue
                M = cv2.moments(c)
                if M['m00'] == 0:
                    continue
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
                # Score = distance from bottom-centre (prefer lines straight ahead)
                score = abs(cx - roi_cx) + abs(cy - roi_cy) * 0.5
                if score < best_score:
                    best_score = score
                    best = (cx, cy, cv2.contourArea(c))

            if best:
                cx, cy, area = best
                error = (cx - roi_cx) / roi_cx
                error_msg.data = float(error)
                self.get_logger().info(
                    f'Line error={error:.3f} area={area:.0f}',
                    throttle_duration_sec=1.0)
                cv2.circle(roi, (int(cx), int(cy)), 8, (0, 0, 255), -1)
                self.error_pub.publish(error_msg)
                self.debug_pub.publish(
                    self.bridge.cv2_to_imgmsg(
                        cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), 'bgr8'))
                return

        # Line not found
        error_msg.data = 9.0
        self.error_pub.publish(error_msg)
        self.get_logger().warn('Line not found', throttle_duration_sec=2.0)
        self.debug_pub.publish(
            self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
