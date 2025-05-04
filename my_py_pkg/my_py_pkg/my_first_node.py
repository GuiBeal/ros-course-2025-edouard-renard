#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MyFirstNode(Node):
    def __init__(self):
        super().__init__("py_test")

        self.counter_ = 0

        self.get_logger().info("Hello world")
        self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Hello " + str(self.counter_))
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)

    node = MyFirstNode()
    rclpy.spin(node=node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
