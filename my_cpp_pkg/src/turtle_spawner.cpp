#include <random>

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <my_robot_interfaces/msg/turtle_array.hpp>

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

    spawnFrequency_ = this->get_parameter("spawn_frequency").as_double();

    pTurtlePublisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turtles", 10);

    pSpawnTimer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / spawnFrequency_),
                                           std::bind(&TurtleSpawnerNode::callSpawn, this));
    pSpawnClient_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

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

    pSpawnClient_->async_send_request(pRequest, std::bind(&TurtleSpawnerNode::callbackCallSpawn, this, _1));
  }

  void callbackCallSpawn(const rclcpp::Client<turtlesim::srv::Spawn>::SharedFutureWithRequest futureRequestResponse)
  {
    const auto pRequest = futureRequestResponse.get().first;
    const auto pResponse = futureRequestResponse.get().second;

    if (pResponse->name.empty()) {
      RCLCPP_WARN(this->get_logger(), "Failed to spawn turtle.");
      return;
    }

    aliveTurtles_.emplace_back(std::make_tuple(pResponse->name, pRequest->x, pRequest->y));
    publishTurtles();

    RCLCPP_INFO(this->get_logger(), "Spawned turtle '%s' at (%.3f,%.3f)",
                pResponse->name.c_str(), pRequest->x, pRequest->y);
  }

  void publishTurtles()
  {
    auto msg = my_robot_interfaces::msg::TurtleArray();

    for (const auto &aliveTurtle : aliveTurtles_)
    {
      auto turtle = my_robot_interfaces::msg::Turtle();
      turtle.name = std::get<Turtle::name>(aliveTurtle);
      turtle.x = std::get<Turtle::x>(aliveTurtle);
      turtle.y = std::get<Turtle::y>(aliveTurtle);

      msg.data.emplace_back(std::move(turtle));
    }

    pTurtlePublisher_->publish(msg);
  }

  std::vector<std::tuple<std::string, float, float>> aliveTurtles_;

  std::default_random_engine gen;
  std::uniform_real_distribution<float> distributionPosition_;
  std::uniform_real_distribution<float> distributionAngle_;

  rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr pTurtlePublisher_;

  double spawnFrequency_;
  rclcpp::TimerBase::SharedPtr pSpawnTimer_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr pSpawnClient_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<TurtleSpawnerNode>();
  rclcpp::spin(pNode);

  return 0;
}
