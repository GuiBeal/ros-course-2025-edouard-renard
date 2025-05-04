#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int64.hpp>

using namespace std::chrono_literals;

class NumberPublisherNode : public rclcpp::Node
{
public:
  NumberPublisherNode()
      : Node("number_publisher")
  {
    pPublisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&NumberPublisherNode::publishNumber, this));

    RCLCPP_INFO(this->get_logger(), "Number publisher started.");
  }

private:
  void publishNumber()
  {
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    pPublisher_->publish(msg);
  }

  long int number_ = 9L;
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr pPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<NumberPublisherNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
