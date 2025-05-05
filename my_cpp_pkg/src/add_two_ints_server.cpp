#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

using namespace std::placeholders;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
  AddTwoIntsServerNode()
      : Node("add_two_ints_server")
  {
    pServer_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints", std::bind(&AddTwoIntsServerNode::callbackAddTwoInts, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Add Two Ints Server started.");
  }

private:
  void callbackAddTwoInts(
      const example_interfaces::srv::AddTwoInts::Request::SharedPtr pRequest,
      const example_interfaces::srv::AddTwoInts::Response::SharedPtr pResponse)
  {
    pResponse->sum = pRequest->a + pRequest->b;

    RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", pRequest->a, pRequest->b, pResponse->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr pServer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto pNode = std::make_shared<AddTwoIntsServerNode>();
  rclcpp::spin(pNode);

  rclcpp::shutdown();
  return 0;
}
