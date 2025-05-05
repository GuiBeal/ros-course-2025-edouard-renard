#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus


class HardwareStatusPublisherNode(Node):
    def __init__(self):
        super().__init__("hardware_status_publisher")

        self.hw_status_pub_ = self.create_publisher(
            msg_type=HardwareStatus, topic="hardware_status", qos_profile=10
        )
        self.timer_ = self.create_timer(
            timer_period_sec=1, callback=self.publish_hw_status
        )

        self.get_logger().info("Hardware Status Publisher started.")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 43.7
        msg.are_motors_ready = True
        msg.debug_message = "Nothing special"
        self.hw_status_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = HardwareStatusPublisherNode()
    rclpy.spin(node=node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
