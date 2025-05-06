#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/msg/led_state_array.hpp>
#include <my_robot_interfaces/srv/set_led.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class LedPanelNode : public rclcpp::Node
{
public:
  LedPanelNode()
      : Node("led_panel")
  {
    this->declare_parameter("led_states", std::vector<int64_t>{0L, 0L, 0L});

    ledStates_ = std::move(this->get_parameter("led_states").as_integer_array());
    if (!std::all_of(
            ledStates_.begin(), ledStates_.end(),
            [](const int &value)
            { return value == 0 || value == 1; }))
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid LED state in led_states parameter.");
      return;
    }

    pPublisher_ = this->create_publisher<my_robot_interfaces::msg::LedStateArray>("led_panel_state", 10);
    pTimer_ = this->create_wall_timer(5s, std::bind(&LedPanelNode::publishLedState, this));

    pServer_ = this->create_service<my_robot_interfaces::srv::SetLed>(
        "set_led", std::bind(&LedPanelNode::callbackSetLed, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "LED Panel started.");
  }

private:
  void publishLedState()
  {
    auto msg = my_robot_interfaces::msg::LedStateArray();
    msg.led_states = ledStates_;
    pPublisher_->publish(msg);
  }

  void callbackSetLed(
      const my_robot_interfaces::srv::SetLed::Request::SharedPtr pRequest,
      const my_robot_interfaces::srv::SetLed::Response::SharedPtr pResponse)
  {
    if (0 > pRequest->led_number || static_cast<std::size_t>(pRequest->led_number) >= ledStates_.size())
    {
      pResponse->success = false;
      pResponse->message = "Invalid LED number.";
      return;
    }

    if (pRequest->state != 0 && pRequest->state != 1)
    {
      pResponse->success = false;
      pResponse->message = "Invalid LED state.";
      return;
    }

    ledStates_.at(pRequest->led_number) = pRequest->state;
    pResponse->success = true;
    pResponse->message = "LED state updated";

    publishLedState();
  }

  std::vector<int64_t> ledStates_;

  rclcpp::Publisher<my_robot_interfaces::msg::LedStateArray>::SharedPtr pPublisher_;
  rclcpp::TimerBase::SharedPtr pTimer_;

  rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr pServer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<LedPanelNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
