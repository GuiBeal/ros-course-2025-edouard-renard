#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
  AddTwoIntsClientNode()
      : Node("add_two_ints_client")
  {
    pClient_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    RCLCPP_INFO(this->get_logger(), "Add Two Ints Client started.");
  }

  void callAddTwoInts(const int a, const int b)
  {
    auto pRequest = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    pRequest->a = a;
    pRequest->b = b;

    while (!pClient_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for Add Two Ints Server...");
    }

    pClient_->async_send_request(pRequest, std::bind(&AddTwoIntsClientNode::callbackCallAddTwoInts, this, _1));
  }

private:
  void callbackCallAddTwoInts(
      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureWithRequest future_response)
  {
    auto pRequest = future_response.get().first;
    auto pResponse = future_response.get().second;

    RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", pRequest->a, pRequest->b, pResponse->sum);
  }

  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr pClient_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<AddTwoIntsClientNode>();
  pNode->callAddTwoInts(4, 7);
  pNode->callAddTwoInts(156, 89);
  pNode->callAddTwoInts(1, -2);
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
