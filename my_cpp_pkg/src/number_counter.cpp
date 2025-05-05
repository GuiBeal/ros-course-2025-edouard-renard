#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int64.hpp>
#include <example_interfaces/srv/set_bool.hpp>

using namespace std::placeholders;

class NumberCounterNode : public rclcpp::Node
{
public:
  NumberCounterNode()
      : Node("number_counter")
  {
    pSubscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
        "number", 10, std::bind(&NumberCounterNode::callbackNumber, this, _1));

    pPublisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

    pServer_ = this->create_service<example_interfaces::srv::SetBool>(
        "reset_counter", std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Number Counter started.");
  }

private:
  void callbackNumber(const example_interfaces::msg::Int64::SharedPtr pMsg)
  {
    counter_ += pMsg->data;

    auto msg = example_interfaces::msg::Int64();
    msg.data = counter_;
    pPublisher_->publish(msg);
  }

  void callbackResetCounter(
      const example_interfaces::srv::SetBool::Request::SharedPtr pRequest,
      const example_interfaces::srv::SetBool::Response::SharedPtr pResponse)
  {
    RCLCPP_INFO(this->get_logger(), "Reset Counter service called.");
    if (pRequest->data)
    {
      counter_ = 0;
      pResponse->success = true;
      pResponse->message = std::string("Counter reset.");
      RCLCPP_INFO(this->get_logger(), "Counter reset.");
    }
    else
    {
      pResponse->success = false;
      pResponse->message = std::string("Counter not reset.");
      RCLCPP_INFO(this->get_logger(), "Counter not reset.");
    }
  }

  int64_t counter_ = 0;

  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr pSubscriber_;
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr pPublisher_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr pServer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<NumberCounterNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
