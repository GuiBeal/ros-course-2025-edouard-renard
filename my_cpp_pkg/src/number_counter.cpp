#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int64.hpp>

using namespace std::placeholders;

class NumberCounterNode : public rclcpp::Node
{
public:
  NumberCounterNode()
      : Node("number_counter")
  {
    pSubscriber_ = this->create_subscription<example_interfaces::msg::Int64>("number", 10, std::bind(&NumberCounterNode::callbackNumber, this, _1));
    pPublisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);

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

  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr pSubscriber_;
  long int counter_ = 0;

  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr pPublisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<NumberCounterNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
