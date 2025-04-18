#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio
import time
import math
from enum import Enum
from gpiozero import Servo
from sensor_msgs.msg import LaserScan

class State(Enum):
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    REVERSE = 4
    IDLE = 5

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.get_logger().info("Controller node with steering + avoidance has started!")

        # --- Servo setup for steering (front wheels) ---
        self.servo = Servo(6)  # Adjust if needed
        self.center_steering()

        # --- Motor setup (rear wheels) ---
        self.PWM_PIN = 12
        self.DIR_PIN_A = 5
        self.FREQUENCY = 100
        self.FORWARD_DUTY = 20
        self.REVERSE_DUTY = 20

        self.h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self.h, self.DIR_PIN_A)
        lgpio.gpio_claim_output(self.h, self.PWM_PIN)

        # Start driving forward
        self.motor_running = False
        self.state = State.FORWARD
        self.last_turn_left = False  # alternate turns

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.navigation_loop)

    def center_steering(self):
        self.servo.mid()
        self.get_logger().info("Steering: Center")

    def steer_left(self):
        self.servo.min()
        self.get_logger().info("Steering: Left")

    def steer_right(self):
        self.servo.max()
        self.get_logger().info("Steering: Right")

    def drive_forward(self):
        lgpio.gpio_write(self.h, self.DIR_PIN_A, 1)
        lgpio.tx_pwm(self.h, self.PWM_PIN, self.FREQUENCY, self.FORWARD_DUTY)
        self.motor_running = True
        self.get_logger().info("Driving forward")

    def drive_reverse(self):
        lgpio.gpio_write(self.h, self.DIR_PIN_A, 0)
        lgpio.tx_pwm(self.h, self.PWM_PIN, self.FREQUENCY, self.REVERSE_DUTY)
        self.motor_running = True
        self.get_logger().warn("Reversing")

    def stop_motor(self):
        lgpio.tx_pwm(self.h, self.PWM_PIN, 0, 0)
        self.motor_running = False
        self.get_logger().warn("Motor stopped")

    def scan_callback(self, msg: LaserScan):
        # Check ±15° forward region
        angle_range = 15
        angle_increment_deg = abs(msg.angle_increment * 180.0 / math.pi)
        half_range = int(angle_range / angle_increment_deg)
        center_index = len(msg.ranges) // 2
        front_ranges = msg.ranges[center_index - half_range:center_index + half_range]
        front_ranges = [r for r in front_ranges if not math.isnan(r)]
        if not front_ranges:
            return
        self.min_distance = min(front_ranges)

    def navigation_loop(self):
        if not hasattr(self, 'min_distance'):
            return  # no LiDAR data yet

        # Main state machine
        if self.state == State.FORWARD:
            if self.min_distance < 0.5:
                self.stop_motor()
                self.state = State.TURN_LEFT if not self.last_turn_left else State.TURN_RIGHT
            else:
                if not self.motor_running:
                    self.center_steering()
                    self.drive_forward()

        elif self.state == State.TURN_LEFT:
            self.get_logger().info("Avoiding left...")
            self.steer_left()
            self.drive_forward()
            time.sleep(1)
            if self.min_distance >= 0.5:
                self.center_steering()
                self.state = State.FORWARD
                self.last_turn_left = True
            else:
                self.state = State.TURN_RIGHT  # try opposite

        elif self.state == State.TURN_RIGHT:
            self.get_logger().info("Avoiding right...")
            self.steer_right()
            self.drive_forward()
            time.sleep(1)
            if self.min_distance >= 0.5:
                self.center_steering()
                self.state = State.FORWARD
                self.last_turn_left = False
            else:
                self.state = State.REVERSE  # fallback

        elif self.state == State.REVERSE:
            self.get_logger().warn("Reversing to reattempt avoidance...")
            self.center_steering()
            self.drive_reverse()
            time.sleep(1.5)
            self.stop_motor()
            # alternate turn direction
            self.state = State.TURN_LEFT if not self.last_turn_left else State.TURN_RIGHT

    def cleanup(self):
        self.stop_motor()
        self.center_steering()
        lgpio.gpiochip_close(self.h)
        self.get_logger().info("Cleanup complete.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()

if __name__ == '__main__':
    main()
