#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


def main(args=None):
    rclpy.init(args=args)

    node = Node("add_two_ints_test_client")

    client = node.create_client(srv_type=AddTwoInts, srv_name="add_two_ints")

    while not client.wait_for_service(timeout_sec=1):
        node.get_logger().warn("Waiting for Add Two Ints Server...")

    request = AddTwoInts.Request()
    request.a = 4
    request.b = 7

    future_response = client.call_async(request)
    rclpy.spin_until_future_complete(node, future_response)

    response = future_response.result()

    node.get_logger().info(
        str(request.a) + " + " + str(request.b) + " = " + str(response.sum)
    )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
