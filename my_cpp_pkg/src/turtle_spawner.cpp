#include <random>

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/kill.hpp>
#include <my_robot_interfaces/msg/turtle_array.hpp>
#include <my_robot_interfaces/srv/catch_turtle.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

enum Turtle : unsigned char
{
  name = 0,
  x = 1,
  y = 2,
};

class TurtleSpawnerNode : public rclcpp::Node
{
public:
  TurtleSpawnerNode()
      : Node("turtle_spawner"), distributionPosition_(0.0, 11.0), distributionAngle_(-M_PI_2, M_PI_2)
  {
    this->declare_parameter("spawn_frequency", 0.5);
    this->declare_parameter("turtle_name_prefix", "turtle");

    spawnFrequency_ = this->get_parameter("spawn_frequency").as_double();
    turtleNamePrefix_ = this->get_parameter("turtle_name_prefix").as_string();

    pTurtlePublisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10);

    pSpawnTimer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / spawnFrequency_),
                                           std::bind(&TurtleSpawnerNode::callSpawn, this));
    pSpawnClient_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

    pCacthTurtleServer_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
        "catch_turtle",
        std::bind(&TurtleSpawnerNode::callbackCatchTurtle, this, _1, _2));

    pKillClient_ = this->create_client<turtlesim::srv::Kill>("/kill");

    RCLCPP_INFO(this->get_logger(), "Turtle Spawner started.");
  }

private:
  void callSpawn()
  {
    while (!pSpawnClient_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for Spawn Server...");
    }

    auto pRequest = std::make_shared<turtlesim::srv::Spawn::Request>();
    pRequest->x = distributionPosition_(gen);
    pRequest->y = distributionPosition_(gen);
    pRequest->theta = distributionAngle_(gen);
    pRequest->name = turtleNamePrefix_ + std::to_string(spawnCount_++);

    pSpawnClient_->async_send_request(pRequest, std::bind(&TurtleSpawnerNode::callbackCallSpawn, this, _1));
  }

  void callbackCallSpawn(const rclcpp::Client<turtlesim::srv::Spawn>::SharedFutureWithRequest futureRequestResponse)
  {
    const auto pRequest = futureRequestResponse.get().first;
    const auto pResponse = futureRequestResponse.get().second;

    if (pResponse->name.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Failed to spawn turtle.");
      return;
    }

    auto turtle = my_robot_interfaces::msg::Turtle();
    turtle.name = pResponse->name;
    turtle.x = pRequest->x;
    turtle.y = pRequest->y;

    RCLCPP_INFO(this->get_logger(), "Spawned turtle '%s' at (%.3f,%.3f)",
                turtle.name.c_str(), turtle.x, turtle.y);

    aliveTurtles_.data.emplace_back(std::move(turtle));
    publishTurtles();
  }

  void publishTurtles()
  {
    pTurtlePublisher_->publish(aliveTurtles_);
  }

  void callbackCatchTurtle(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr pRequest,
                           const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr pResponse)
  {
    const auto it = std::find_if(aliveTurtles_.data.begin(), aliveTurtles_.data.end(),
                                 [pRequest](const my_robot_interfaces::msg::Turtle &turtle)
                                 { return turtle.name == pRequest->name; });
    if (it == aliveTurtles_.data.end())
    {
      pResponse->success = false;
      return;
    }

    while (!pKillClient_->wait_for_service(1s))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for Kill Server...");
    }

    auto pKillRequest = std::make_shared<turtlesim::srv::Kill::Request>();
    pKillRequest->name = pRequest->name;
    pKillClient_->async_send_request(pKillRequest);

    aliveTurtles_.data.erase(it);

    pResponse->success = true;

    publishTurtles();
  }

  my_robot_interfaces::msg::TurtleArray aliveTurtles_;

  std::default_random_engine gen;
  std::uniform_real_distribution<float> distributionPosition_;
  std::uniform_real_distribution<float> distributionAngle_;

  rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr pTurtlePublisher_;

  double spawnFrequency_;
  rclcpp::TimerBase::SharedPtr pSpawnTimer_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr pSpawnClient_;

  std::string turtleNamePrefix_;
  uint64_t spawnCount_ = 1;

  rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr pCacthTurtleServer_;

  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr pKillClient_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<TurtleSpawnerNode>();
  rclcpp::spin(pNode);

  return 0;
}
