#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>

using namespace std::placeholders;

class SmartphoneNode : public rclcpp::Node
{
public:
  SmartphoneNode()
      : Node("smartphone")
  {
    pSubscriber_ = this->create_subscription<example_interfaces::msg::String>("robot_news", 10, std::bind(&SmartphoneNode::callbackRobotNews, this, _1));

    RCLCPP_INFO(this->get_logger(), "Smartphone started.");
  }

private:
  void callbackRobotNews(const example_interfaces::msg::String::SharedPtr pMsg)
  {
    RCLCPP_INFO(this->get_logger(), pMsg->data.c_str());
  }

  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr pSubscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<SmartphoneNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
