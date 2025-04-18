#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch_ros.actions import Node
class launch_node(Node):  
  def __init__(self):
    super().__init__("launch_node")
    self.get_logger().info("Launch_node has started!")


  def launch_robot():
    return LaunchDescription([
        Node(
            package='servo_node',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),
        Node(
            package='lidar_node',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='robot_controller',
            executable='first_node',
            name='controller_node',
            output='screen'
        )
    ])

def main(args=None):
  rclpy.init(args=args)
  #
  #
  #nodes are not the files or files its self but inside
  #of the file, usually beginers we use one node
  # per program but we can actuall just use
  #OOP to creat multiple nodes inside of one program
  #we will most likey take this approach
  node = launch_node()
  rclpy.spin(node)





  rclpy.shutdown

if __name__ == '__main__':
  main()
