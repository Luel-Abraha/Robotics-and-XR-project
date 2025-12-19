#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ProjectNode(Node):

    def __init__(self):
        super().__init__("project_node")
        self.get_logger().info("Hello world from the Python node project_node")


def main(args=None):
    rclpy.init(args=args)

    project_node = ProjectNode()

    try:
        rclpy.spin(project_node)
    except KeyboardInterrupt:
        pass

    project_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
