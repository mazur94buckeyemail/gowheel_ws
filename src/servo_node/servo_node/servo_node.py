#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class servo_Node(Node):
  def __init__(self):
    super().__init__("servo_node")
    self.get_logger().info("Servo_node has started!")


def main(args=None):
  rclpy.init(args=args)
  #
  #
  #nodes are not the files or files its self but inside
  #of the file, usually beginers we use one node
  # per program but we can actuall just use
  #OOP to creat multiple nodes inside of one program
  #we will most likey take this approach
  node = servo_Node()
  rclpy.spin(node)





  rclpy.shutdown

if __name__ == '__main__':
  main()
