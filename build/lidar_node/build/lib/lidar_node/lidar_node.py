#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class lidar_Node(Node):
  def __init__(self):
    super().__init__("lidar_node")
    #self.get_logger().info("Lidar_node has started!")
    self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
    )
    self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    self.get_logger().info("Lidar Node for Obstacle avoidance has Started!")

    
  def scan_callback(self, msg: LaserScan):
    # Narrow scan region in front of the robot (e.g., -15° to +15°)
    angle_range = 15
    min_distance = float('inf')
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment

    center_index = len(msg.ranges) // 2
    half_range = int(angle_range / math.degrees(angle_increment))

    front_ranges = msg.ranges[center_index - half_range:center_index + half_range]
    min_distance = min(front_ranges)

    cmd = Twist()

    if min_distance < 0.5:  # If obstacle too close
        self.get_logger().warn(f"Obstacle detected at {min_distance:.2f}m! Turning.")
        # Check left vs right
        left_avg = sum(msg.ranges[-90:-45]) / len(msg.ranges[-90:-45])
        right_avg = sum(msg.ranges[45:90]) / len(msg.ranges[45:90])
        if left_avg > right_avg:
            cmd.angular.z = 0.5  # Turn left
        else:
            cmd.angular.z = -0.5  # Turn right
    else:
        cmd.linear.x = 0.2  # Move forward

    self.cmd_pub.publish(cmd)  
  


def main(args=None):
  rclpy.init(args=args)
  #
  #
  #nodes are not the files or files its self but inside
  #of the file, usually beginers we use one node
  # per program but we can actuall just use
  #OOP to creat multiple nodes inside of one program
  #we will most likey take this approach
  node = lidar_Node()
  rclpy.spin(node)
  rclpy.shutdown

if __name__ == '__main__':
  main()
