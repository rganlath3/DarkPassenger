#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create node
    node = Node("py_test")
    node.get_logger().info("Hello world")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()