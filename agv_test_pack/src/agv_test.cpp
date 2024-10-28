#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "agv_test_pack/agv_test.hpp"
#include "rclcpp/rclcpp.hpp"
#include "actionlib_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

AgvTest::AgvTest(const rclcpp::NodeOptions &node_options)
    : Node("agv_test", node_options),
      move_status_(0),
      status_index_(0)
{
  RCLCPP_INFO(this->get_logger(), "Wheel Drive Controller initialized");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  goal_status_subscriber_ = this->create_subscription<actionlib_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", QOS_RKL10V, std::bind(&AgvTest::goal_status_callback, this, std::placeholders::_1));
  goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
  // geometry_msgs/msg/PoseStamped

  // timer_ = this->create_wall_timer(10ms, std::bind(&AgvTest::publish_states, this));
}

AgvTest::~AgvTest() {}

void AgvTest::goal_status_callback(const actionlib_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "aaaa");
  RCLCPP_INFO(get_logger(), "bbbb");
  if (msg->status_list.empty())
  {
    return;
  }
  move_status_ = msg->status_list[status_index_].status;
  RCLCPP_INFO(get_logger(), "I heard status: %d", move_status_);
  if (status_index_ == 6 || status_index_ == 4)
  {
    status_index_++;
  }
  // if (move_status_ == 3)
  //   RCLCPP_INFO(get_logger(), "I heard status: %d", move_status_)
}

void AgvTest::publish_states()
{
  rclcpp::Time now = this->now();

  // // Joint states 퍼블리시
  // joint_state_.header.stamp = now;
  // joint_state_.name = {"steering_joint", "drive_wheel_joint"};
  // joint_state_.position = {0.0, wheel_position};
  // joint_state_.velocity = {0.0, wheel_velocity};

  // goal_pose_publisher_->publish(goal_pose_);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvTest>());
  rclcpp::shutdown();
  return 0;
}
