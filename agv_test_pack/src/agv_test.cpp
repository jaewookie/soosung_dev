#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "agv_test_pack/agv_test.hpp"

using namespace std::chrono_literals;

AgvTest::AgvTest(const rclcpp::NodeOptions &node_options)
    : Node("agv_test", node_options),
      move_status_(0),
      node_name_(""),
      x_(0.0),
      y_(0.0),
      z_(0.0),
      w_(0.0),
      drive_mode_(1),
      sending_drive_mode_(1)
{
  RCLCPP_INFO(this->get_logger(), "AGV Driver initialized");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  goal_status_subscriber_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", QOS_RKL10V, std::bind(&AgvTest::goal_status_callback, this, std::placeholders::_1));
  goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

  drive_mode_subscriber_ = this->create_subscription<agv_msgs::msg::AgvDrive>(
      "/agv_state", QOS_RKL10V, std::bind(&AgvTest::agv_status_callback, this, std::placeholders::_1));
  drive_mode_publisher_ = this->create_publisher<agv_msgs::msg::AgvDrive>("/agv_state", 10);
}

AgvTest::~AgvTest() {}

// void AgvTest::get_node_inform()
// {
//   // 사용자로부터 좌표 입력 받기
//   std::cout << "Enter the node: ";
//   std::cin >> node_name_;
//   publish_goal();
// }

void AgvTest::publish_goal()
{
  auto message = geometry_msgs::msg::PoseStamped();
  message.header.stamp = this->now();
  message.header.frame_id = "map";

  // 입력 받은 목표 위치 설정
  message.pose.position.x = x_;
  message.pose.position.y = y_;
  message.pose.position.z = 0.0;

  // 회전 없이 목표지점 설정
  message.pose.orientation.x = 0.0;
  message.pose.orientation.y = 0.0;
  message.pose.orientation.z = w_;
  message.pose.orientation.w = 0.0;

  goal_pose_publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Published goal: x=%f, y=%f", message.pose.position.x, message.pose.position.y);
}

void AgvTest::goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  if (msg->status_list.empty())
  {
    publish_goal();
    return;
  }
  else
  {
    move_status_ = msg->status_list.back().status;
  }
  // RCLCPP_INFO(get_logger(), "I heard status: %d", move_status_);
  if (move_status_ == 6 || move_status_ == 4)
  {
    // RCLCPP_INFO(this->get_logger(), "Reached");
    publish_states();
    // get_node_inform();
  }
}

void AgvTest::agv_status_callback(const agv_msgs::msg::AgvDrive::SharedPtr msg)
{
  drive_mode_ = msg->drive_state;
  RCLCPP_INFO(this->get_logger(), "%d", drive_mode_);
  agv_action(drive_mode_);
}

void AgvTest::publish_states()
{
  auto message = agv_msgs::msg::AgvDrive();
  message.drive_state = sending_drive_mode_;
  drive_mode_publisher_->publish(message);
}

void AgvTest::agv_action(int drive_mode_)
{
  agv_mode = drive_mode_;
  if (agv_mode == 1)
  {
    sending_drive_mode_ = 2;
    x_ = 2.0;
    y_ = -1.5;
    z_ = -1.574/2;
    w_ = 0.7;
  }
  else if (agv_mode == 2)
  {
    sending_drive_mode_ = 3;
    x_ = 4.0;
    y_ = 0.0;
    z_ = -1.574;
    w_ = 0.0;
  }
  else if (agv_mode == 5)
  {
    sending_drive_mode_ = 6;
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    w_ = 0.0;
  }
  // else if (agv_mode == 1)
  // {
  //   sending_drive_mode_ = 2;
  //   x_ = 2.0;
  //   y_ = -2.0;
  //   z_ = -0.714;
  //   w_ = 0.7;
  // }
  // else if (agv_mode == 3)
  // {
  //   sending_drive_mode_ = 4;
  //   x_ = 2.0;
  //   y_ = -2.0;
  //   z_ = -0.714;
  //   w_ = 0.7;
  // }
  auto message = geometry_msgs::msg::PoseStamped();
  message.header.stamp = this->now();
  message.header.frame_id = "map";

  // 입력 받은 목표 위치 설정
  message.pose.position.x = x_;
  message.pose.position.y = y_;
  message.pose.position.z = 0.0;

  // 회전 없이 목표지점 설정
  message.pose.orientation.x = 0.0;
  message.pose.orientation.y = 0.0;
  message.pose.orientation.z = z_;
  message.pose.orientation.w = w_;

  goal_pose_publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Published goal: x=%f, y=%f", message.pose.position.x, message.pose.position.y);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvTest>());
  rclcpp::shutdown();
  return 0;
}
