#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

// Joint states Subscriber
class JointStateSubscriber : public rclcpp::Node
{
  public:
    JointStateSubscriber()
    : Node("joint_state_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&JointStateSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::JointState & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "-----Joint Positions-----\n");
      for (int i = 0; i < 4; i++)
        RCLCPP_INFO(this->get_logger(), "Joint Position %d: '%6f'\n", i+1, msg.position[i]);
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

// Joint states Publisher
class PositionControllerPublisher : public rclcpp::Node
{
  public:
    PositionControllerPublisher()
    : Node("position_controller_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&PositionControllerPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto commands = std_msgs::msg::Float64MultiArray();
      commands.data = {1.0, 1.0, 1.0, 1.0};  // Initialize all values at once
      RCLCPP_INFO(this->get_logger(), "Publishing: '%zu'", count_++);
      publisher_->publish(commands);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create executors for the nodes
  auto joint_state_subscriber = std::make_shared<JointStateSubscriber>();
  auto position_controller_publisher = std::make_shared<PositionControllerPublisher>();

  // Spin both nodes using a MultiThreadedExecutor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(joint_state_subscriber);
  executor.add_node(position_controller_publisher);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}