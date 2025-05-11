#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>

using namespace std::chrono_literals;

class RobotNewsStationNode : public rclcpp::Node
{
public:
  RobotNewsStationNode()
      : Node("robot_news_station")
  {
    this->declare_parameter("robot_name", "R2D2");

    robot_name_ = this->get_parameter("robot_name").as_string();

    pPublisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
    pTimer_ = this->create_wall_timer(0.5s, std::bind(&RobotNewsStationNode::publishNews, this));

    RCLCPP_INFO(this->get_logger(), "Robot News Station started.");
  }

private:
  void publishNews()
  {
    auto msg = example_interfaces::msg::String();
    msg.data = std::string("Hello, this is ") + robot_name_ + std::string(" from Robot News Station!");

    pPublisher_->publish(msg);
  }

  std::string robot_name_;
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr pPublisher_;
  rclcpp::TimerBase::SharedPtr pTimer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<RobotNewsStationNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
