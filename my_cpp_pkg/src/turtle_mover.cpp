#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <my_robot_interfaces/msg/turtle_array.hpp>
#include <my_robot_interfaces/srv/catch_turtle.hpp>

using namespace std::placeholders;

float angleDifference(const float a, const float b)
{
  float diff = a - b;
  if (diff > M_PI)
    diff -= 2 * M_PI;
  else if (diff <= -M_PI)
    diff += 2 * M_PI;
  return diff;
}

class TurtleMoverNode : public rclcpp::Node
{
public:
  TurtleMoverNode()
      : Node("turtle_mover")
  {
    this->declare_parameter("timer_period", 0.25);
    this->declare_parameter("distance_tolerance", 0.1);
    this->declare_parameter("linear_gain", 1.0);
    this->declare_parameter("angular_gain", 1.0);
    this->declare_parameter("turtle_name", "turtle1");
    this->declare_parameter("catch_closest_turtle_first", false);

    timerPeriod_ = this->get_parameter("timer_period").as_double();
    distanceTolerance_ = this->get_parameter("distance_tolerance").as_double();
    gainProportionalLinear_ = this->get_parameter("linear_gain").as_double();
    gainProportionalAngular_ = this->get_parameter("angular_gain").as_double();
    turtleName_ = this->get_parameter("turtle_name").as_string();
    catchClosestFirst_ = this->get_parameter("catch_closest_turtle_first").as_bool();

    pPoseSubscriber_ = this->create_subscription<turtlesim::msg::Pose>(
        turtleName_ + "/pose", 10, std::bind(&TurtleMoverNode::callbackPose, this, _1));
    pAliveTurtlesSubscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
        "/alive_turtles", 10, std::bind(&TurtleMoverNode::callbackAliveTurtles, this, _1));

    pCommandPublisher_ = this->create_publisher<geometry_msgs::msg::Twist>(turtleName_ + "/cmd_vel", 10);

    pCommandTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(timerPeriod_), std::bind(&TurtleMoverNode::callbackMove, this));

    pCatchTurtleClient_ = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");

    RCLCPP_INFO(this->get_logger(), "Turtle Mover started.");
  }

private:
  void callbackPose(const turtlesim::msg::Pose::ConstSharedPtr pPose)
  {
    pTurtlePose_ = pPose;
  }

  void callbackAliveTurtles(const my_robot_interfaces::msg::TurtleArray::ConstSharedPtr pTurtles)
  {
    if (pTurtles->data.empty())
    {
      pObjectiveTurtle_ = nullptr;
      return;
    }

    if (catchClosestFirst_)
    {
      auto it = std::min_element(pTurtles->data.begin(), pTurtles->data.end(),
                                 [this](const my_robot_interfaces::msg::Turtle &turtleA,
                                        const my_robot_interfaces::msg::Turtle &turtleB)
                                 { return std::sqrt(std::pow(turtleA.x - pTurtlePose_->x, 2) +
                                                    std::pow(turtleA.y - pTurtlePose_->y, 2)) <
                                          std::sqrt(std::pow(turtleB.x - pTurtlePose_->x, 2) +
                                                    std::pow(turtleB.y - pTurtlePose_->y, 2)); });

      pObjectiveTurtle_ = std::make_shared<my_robot_interfaces::msg::Turtle>(*it);
    }
    else
    {
      pObjectiveTurtle_ = std::make_shared<my_robot_interfaces::msg::Turtle>(pTurtles->data.front());
    }
  }

  void callbackMove()
  {
    if (!pTurtlePose_ || !pObjectiveTurtle_)
    {
      return;
    }

    const auto xError = pObjectiveTurtle_->x - pTurtlePose_->x;
    const auto yError = pObjectiveTurtle_->y - pTurtlePose_->y;
    const auto distanceError = std::sqrt(std::pow(xError, 2) + std::pow(yError, 2));

    auto msgCommand = geometry_msgs::msg::Twist();
    if (fabs(distanceError) < distanceTolerance_)
    {
      msgCommand.linear.x = 0;
      msgCommand.angular.y = 0;

      callCatchTurtle(pObjectiveTurtle_);
    }
    else
    {
      const auto thetaReference = std::atan2(yError, xError);
      const auto thetaError = angleDifference(thetaReference, pTurtlePose_->theta);

      msgCommand.linear.x = gainProportionalLinear_ * distanceError;
      msgCommand.angular.z = gainProportionalAngular_ * thetaError;
    }

    pCommandPublisher_->publish(msgCommand);
  }

  void callCatchTurtle(my_robot_interfaces::msg::Turtle::SharedPtr pTurtle)
  {
    if (!pTurtle)
    { // TODO change to assert
      return;
    }

    auto pRequest = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
    pRequest->name = pTurtle->name;

    pCatchTurtleClient_->async_send_request(pRequest);
  }

  double timerPeriod_;

  std::string turtleName_;
  turtlesim::msg::Pose::ConstSharedPtr pTurtlePose_ = nullptr;

  my_robot_interfaces::msg::Turtle::SharedPtr pObjectiveTurtle_ = nullptr;

  bool catchClosestFirst_;

  double distanceTolerance_;
  double gainProportionalLinear_;
  double gainProportionalAngular_;

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pPoseSubscriber_;
  rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr pAliveTurtlesSubscriber_;

  rclcpp::TimerBase::SharedPtr pCommandTimer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pCommandPublisher_;

  rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedPtr pCatchTurtleClient_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<TurtleMoverNode>();
  rclcpp::spin(pNode);

  return 0;
}
