#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<rclcpp::Node>("add_two_ints_test_client");
  auto pClient = pNode->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  while (!pClient->wait_for_service(1s))
  {
    RCLCPP_WARN(pNode->get_logger(), "Waiting for Add Two Ints Server...");
  }

  auto pRequest = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  pRequest->a = 4;
  pRequest->b = 7;

  auto future_response = pClient->async_send_request(pRequest);
  rclcpp::spin_until_future_complete(pNode, future_response);

  auto pResponse = future_response.get();

  RCLCPP_INFO(pNode->get_logger(), "%ld + %ld = %ld", pRequest->a, pRequest->b, pResponse->sum);

  rclcpp::shutdown();
  return 0;
}
