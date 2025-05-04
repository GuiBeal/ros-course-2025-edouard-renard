#include <rclcpp/rclcpp.hpp>

class MyFirstNode : public rclcpp::Node
{
public:
  MyFirstNode()
      : Node("cpp_test")
  {
    RCLCPP_INFO(this->get_logger(), "Hello world");
    pTimer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyFirstNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    RCLCPP_INFO(this->get_logger(), "Hello %d", counter_++);
  }

  rclcpp::TimerBase::SharedPtr pTimer_;
  int counter_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<MyFirstNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
