#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/srv/set_led.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class BatteryNode : public rclcpp::Node
{
public:
  BatteryNode()
      : Node("battery")
  {
    pClient_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

    timeLastBatteryStateChanged_ = getCurrentTimeSeconds();
    pTimer_ = this->create_wall_timer(0.1s, std::bind(&BatteryNode::checkBatterState, this));

    RCLCPP_INFO(this->get_logger(), "Battery started.");
  }

private:
  double getCurrentTimeSeconds() { return this->get_clock()->now().seconds(); }

  void checkBatterState()
  {
    const double timeNow = getCurrentTimeSeconds();

    if (bateryState_.compare("full") == 0 && timeNow - timeLastBatteryStateChanged_ > 4)
    {
      bateryState_ = "empty";
      RCLCPP_INFO(this->get_logger(), "Battery is empty! Recharging...");
      callSetLed(2, 1);
      timeLastBatteryStateChanged_ = timeNow;
      return;
    }

    if (bateryState_.compare("empty") == 0 && timeNow - timeLastBatteryStateChanged_ > 6)
    {
      bateryState_ = "full";
      RCLCPP_INFO(this->get_logger(), "Battery is full.");
      callSetLed(2, 0);
      timeLastBatteryStateChanged_ = timeNow;
      return;
    }
  }

  void callSetLed(const int64_t ledNumber, const int64_t state)
  {
    while (!pClient_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for Set LED Server...");
    }

    auto pRequest = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
    pRequest->led_number = ledNumber;
    pRequest->state = state;

    pClient_->async_send_request(pRequest, std::bind(&BatteryNode::callbackCallSetLed, this, _1));
  }

  void callbackCallSetLed(rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture futureResponse)
  {
    auto pResponse = futureResponse.get();
    if (pResponse->success)
    {
      RCLCPP_INFO(this->get_logger(), "LED set.");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "LED not set.");
    }
  }

  rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr pClient_;
  rclcpp::TimerBase::SharedPtr pTimer_;

  std::string bateryState_ = "full";
  double timeLastBatteryStateChanged_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<BatteryNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
