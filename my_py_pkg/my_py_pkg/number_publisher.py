#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from example_interfaces.msg import Int64


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")

        self.declare_parameter(name="number", value=9)
        self.declare_parameter(name="timer_period", value=1.0)

        self.number_ = self.get_parameter(name="number").value
        self.timer_period_ = self.get_parameter(name="timer_period").value

        self.add_post_set_parameters_callback(self.callback_parameters)

        self.publisher_ = self.create_publisher(
            msg_type=Int64, topic="number", qos_profile=10
        )
        self.timer_ = self.create_timer(
            timer_period_sec=self.timer_period_, callback=self.publish_number
        )

        self.get_logger().info("Number Publisher started.")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)

    def callback_parameters(self, params: list[Parameter]):
        for param in params:
            if param.name == "number":
                self.number_ = param.value
            if param.name == "timer_period":
                self.timer_period_ = param.value
                self.timer_.destroy()
                self.timer_ = self.create_timer(
                    timer_period_sec=self.timer_period_, callback=self.publish_number
                )


def main(args=None):
    rclpy.init(args=args)

    node = NumberPublisherNode()
    rclpy.spin(node=node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
