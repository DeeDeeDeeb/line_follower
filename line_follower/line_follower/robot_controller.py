#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from enum import Enum, auto
import time


class Mode(Enum):
    LINE_FOLLOW    = auto()
    AVOID_DODGE    = auto()
    AVOID_PASS     = auto()
    AVOID_REJOIN   = auto()
    COAST          = auto()
    SEARCH         = auto()


class RobotController(Node):

    LINEAR_SPEED    = 0.10
    ANGULAR_GAIN    = 0.6
    ANGULAR_DAMP    = 0.4

    COAST_SPEED     = 0.10
    COAST_DURATION  = 2.5   # 0.3m gap at 0.10m/s needs ~3s — safe margin

    DODGE_LINEAR    =  0.10
    DODGE_ANGULAR   =  0.5
    DODGE_DURATION  =  1.5

    PASS_LINEAR     =  0.12
    PASS_ANGULAR    =  0.0
    PASS_DURATION   =  2.0

    REJOIN_LINEAR   =  0.08
    REJOIN_ANGULAR  = -0.4

    SEARCH_ANGULAR  =  0.25

    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32, '/line_error', self.line_error_cb, 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_cb, 10)

        self.mode         = Mode.LINE_FOLLOW
        self.line_error   = 9.0
        self.obstacle     = False
        self.mode_start   = time.time()
        self.last_error   = 0.0
        self.prev_angular = 0.0

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('RobotController started.')

    def line_error_cb(self, msg):
        if abs(msg.data) < 2.0:
            self.last_error = msg.data
        self.line_error = msg.data

    def obstacle_cb(self, msg):
        self.obstacle = msg.data

    def line_detected(self):
        return abs(self.line_error) < 2.0

    def smooth_angular(self, target):
        s = self.ANGULAR_DAMP * self.prev_angular + (1 - self.ANGULAR_DAMP) * target
        self.prev_angular = s
        return s

    def set_mode(self, mode):
        self.get_logger().info(f'{self.mode.name} → {mode.name}')
        self.mode = mode
        self.mode_start = time.time()

    def control_loop(self):
        twist = Twist()
        elapsed = time.time() - self.mode_start

        if self.mode == Mode.LINE_FOLLOW:
            if self.obstacle:
                self.set_mode(Mode.AVOID_DODGE)
                self.prev_angular = 0.0
            elif not self.line_detected():
                self.set_mode(Mode.COAST)

        elif self.mode == Mode.AVOID_DODGE:
            if elapsed >= self.DODGE_DURATION:
                self.set_mode(Mode.AVOID_PASS)

        elif self.mode == Mode.AVOID_PASS:
            if elapsed >= self.PASS_DURATION:
                self.set_mode(Mode.AVOID_REJOIN)

        elif self.mode == Mode.AVOID_REJOIN:
            if self.line_detected() and not self.obstacle:
                self.set_mode(Mode.LINE_FOLLOW)

        elif self.mode == Mode.COAST:
            if self.line_detected():
                self.get_logger().info(f'Line found after {elapsed:.1f}s')
                self.set_mode(Mode.LINE_FOLLOW)
            elif elapsed > self.COAST_DURATION:
                self.set_mode(Mode.SEARCH)

        elif self.mode == Mode.SEARCH:
            if self.line_detected() and not self.obstacle:
                self.set_mode(Mode.LINE_FOLLOW)

        if self.mode == Mode.LINE_FOLLOW:
            twist.linear.x  = self.LINEAR_SPEED
            twist.angular.z = self.smooth_angular(-self.ANGULAR_GAIN * self.line_error)

        elif self.mode == Mode.AVOID_DODGE:
            twist.linear.x  = self.DODGE_LINEAR
            twist.angular.z = self.DODGE_ANGULAR

        elif self.mode == Mode.AVOID_PASS:
            twist.linear.x  = self.PASS_LINEAR
            twist.angular.z = self.PASS_ANGULAR

        elif self.mode == Mode.AVOID_REJOIN:
            twist.linear.x  = self.REJOIN_LINEAR
            twist.angular.z = self.REJOIN_ANGULAR

        elif self.mode == Mode.COAST:
            twist.linear.x  = self.COAST_SPEED
            twist.angular.z = -self.ANGULAR_GAIN * self.last_error * 0.3
            self.get_logger().info(
                f'COAST {elapsed:.1f}s / {self.COAST_DURATION}s',
                throttle_duration_sec=0.5)

        elif self.mode == Mode.SEARCH:
            twist.linear.x  = 0.0
            twist.angular.z = self.SEARCH_ANGULAR

        self.cmd_pub.publish(twist)

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
