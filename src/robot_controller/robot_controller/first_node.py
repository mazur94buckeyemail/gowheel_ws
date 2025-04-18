#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
import lgpio
from gpiozero import Servo
import time
import math
from enum import Enum
import os

class Mode(Enum):
    LINE_FOLLOW = 1
    OBSTACLE_AVOIDANCE = 2
    SEARCH_FOR_LINE = 3
    HALT = 4

class IntegratedControllerNode(Node):
    def __init__(self):
        super().__init__('integrated_controller_node')
        self.get_logger().info('Integrated Controller Node started!')

        # Servo setup
        self.servo = Servo(6)
        self.center_steering()

        # Motor setup
        self.PWM_PIN = 12
        self.DIR_PIN_A = 5
        self.FREQUENCY = 100
        self.FORWARD_DUTY = 20
        self.REVERSE_DUTY = 12
        self.h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self.h, self.DIR_PIN_A)
        lgpio.gpio_claim_output(self.h, self.PWM_PIN)

        # ROS topics
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.min_distance = float('inf')
        self.green_line_detected = False
        self.motor_running = False
        self.mode = Mode.LINE_FOLLOW
        self.last_turn_left = False
        self.avoidance_attempts = 0
        self.max_avoidance_attempts = 5
        self.obstacle_detected = False  # <-- new tracking flag

    def is_display_available(self):
        return os.environ.get('DISPLAY') is not None

    def center_steering(self):
        self.servo.mid()
        self.get_logger().info('Steering: Center')

    def steer_left(self):
        self.servo.min()
        self.get_logger().info('Steering: Left')

    def steer_right(self):
        self.servo.max()
        self.get_logger().info('Steering: Right')

    def drive_forward(self):
        lgpio.gpio_write(self.h, self.DIR_PIN_A, 1)
        lgpio.tx_pwm(self.h, self.PWM_PIN, self.FREQUENCY, self.FORWARD_DUTY)
        self.motor_running = True
        self.get_logger().info('Driving forward')

    def drive_reverse(self):
        lgpio.gpio_write(self.h, self.DIR_PIN_A, 0)
        lgpio.tx_pwm(self.h, self.PWM_PIN, self.FREQUENCY, self.REVERSE_DUTY)
        self.motor_running = True
        self.get_logger().warn('Reversing')

    def stop_motor(self):
        if self.motor_running:
            lgpio.tx_pwm(self.h, self.PWM_PIN, self.FREQUENCY, 0)
            self.motor_running = False
            time.sleep(1)
            self.get_logger().warn('Motor stopped')

    def zero_components_and_pause(self):
        self.stop_motor()
        self.center_steering()
        self.get_logger().info('Zeroing components...')
        time.sleep(2)

    def scan_callback(self, msg):
        angle_range = 30
        angle_increment_deg = abs(msg.angle_increment * 180.0 / math.pi)
        half_range = int(angle_range / angle_increment_deg)
        center_index = len(msg.ranges) // 2
        front_ranges = msg.ranges[center_index - half_range:center_index + half_range]
        front_ranges = [r for r in front_ranges if not math.isnan(r)]
        if front_ranges:
            self.min_distance = min(front_ranges)
            self.obstacle_detected = self.min_distance < 0.5
        else:
            self.obstacle_detected = False

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.green_line_detected = False

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                self.green_line_detected = True
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                cy = y + h // 2
                width = cv_image.shape[1]
                image_center = width // 2
                error = (cx - image_center) / image_center
                steering_strength = 0.8
                if abs(error) < 0.05:
                    self.servo.value = 0.0
                else:
                    self.servo.value = max(min(error * steering_strength, 1.0), -1.0)
                self.get_logger().info(f'Line at x={cx}, y={cy}, width={width}, steering={self.servo.value:.2f}')

                if self.is_display_available():
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

    def loop(self):
        self.get_logger().info(f"Current min_distance: {self.min_distance}")
        if self.mode == Mode.LINE_FOLLOW:
            if self.obstacle_detected:
                self.stop_motor()
                self.mode = Mode.OBSTACLE_AVOIDANCE
            elif self.green_line_detected:
                if not self.motor_running:
                    self.drive_forward()
            else:
                self.stop_motor()
                self.mode = Mode.SEARCH_FOR_LINE

        elif self.mode == Mode.OBSTACLE_AVOIDANCE:
            if self.last_turn_left:
                self.steer_right()
            else:
                self.steer_left()
            self.drive_forward()
            time.sleep(1)
            self.stop_motor()

            if not self.obstacle_detected:
                self.get_logger().info('Obstacle cleared. Switching to SEARCH_FOR_LINE')
                self.zero_components_and_pause()
                self.mode = Mode.SEARCH_FOR_LINE
                self.avoidance_attempts = 0
                self.last_turn_left = not self.last_turn_left
            else:
                self.avoidance_attempts += 1
                self.get_logger().warn(f'Avoidance attempt #{self.avoidance_attempts}')
                if self.avoidance_attempts >= self.max_avoidance_attempts:
                    self.get_logger().error('Too many avoidance attempts. Stopping all movement.')
                    self.stop_motor()
                    self.center_steering()
                    self.mode = Mode.HALT
                else:
                    self.drive_reverse()
                    time.sleep(1.5)
                    self.stop_motor()
                    self.zero_components_and_pause()
                    self.last_turn_left = not self.last_turn_left

        elif self.mode == Mode.SEARCH_FOR_LINE:
            self.steer_left() if self.last_turn_left else self.steer_right()
            self.drive_forward()
            time.sleep(2)
            self.stop_motor()
            if self.green_line_detected:
                self.center_steering()
                self.mode = Mode.LINE_FOLLOW

        elif self.mode == Mode.HALT:
            self.stop_motor()
            self.center_steering()
            self.get_logger().fatal("Robot halted due to repeated avoidance failures.")

    def cleanup(self):
        self.stop_motor()
        self.center_steering()
        lgpio.gpiochip_close(self.h)
        self.get_logger().info('Shutdown complete.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()

if __name__ == '__main__':
    main()
