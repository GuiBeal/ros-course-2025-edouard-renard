#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/int64.hpp>

using namespace std::placeholders;

class NumberPublisherNode : public rclcpp::Node
{
public:
  NumberPublisherNode()
      : Node("number_publisher")
  {
    this->declare_parameter("number", 9);
    this->declare_parameter("timer_period", 1.0);

    number_ = this->get_parameter("number").as_int();
    timerPeriod_ = this->get_parameter("timer_period").as_double();

    pCallbackParams_ = this->add_post_set_parameters_callback(
        std::bind(&NumberPublisherNode::callbackParameters, this, _1));

    pPublisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timerPeriod_),
        std::bind(&NumberPublisherNode::publishNumber, this));

    RCLCPP_INFO(this->get_logger(), "Number publisher started.");
  }

private:
  void publishNumber()
  {
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    pPublisher_->publish(msg);
  }

  void callbackParameters(const std::vector<rclcpp::Parameter> &params)
  {
    for (const auto &param : params)
    {
      if (param.get_name() == "number")
      {
        number_ = param.as_int();
      }
      else if (param.get_name() == "timer_period")
      {
        timerPeriod_ = param.as_double();
        timer_->cancel();
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timerPeriod_),
            std::bind(&NumberPublisherNode::publishNumber, this));
      }
    }
  }

  int64_t number_;
  double timerPeriod_;

  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr pPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  PostSetParametersCallbackHandle::SharedPtr pCallbackParams_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<NumberPublisherNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
