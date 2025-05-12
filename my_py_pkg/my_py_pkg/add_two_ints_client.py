#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial


class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")

        self.client_ = self.create_client(srv_type=AddTwoInts, srv_name="add_two_ints")

        self.get_logger().info("Add Two Ints Client started.")

    def call_add_two_ints(self, a, b):
        while not self.client_.wait_for_service(timeout_sec=1):
            self.get_logger().warn("Waiting for Add Two Ints Server...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future_response = self.client_.call_async(request)
        future_response.add_done_callback(
            partial(self.callback_call_add_two_ints, request=request)
        )

    def callback_call_add_two_ints(self, future_response, request):
        response = future_response.result()

        self.get_logger().info(
            str(request.a) + " + " + str(request.b) + " = " + str(response.sum)
        )


def main(args=None):
    rclpy.init(args=args)

    node = AddTwoIntsClientNode()
    node.call_add_two_ints(4, 7)
    node.call_add_two_ints(156, 89)
    node.call_add_two_ints(1, -2)
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
