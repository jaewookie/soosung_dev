#ifndef AgvTest__AgvTest_HPP_
#define AgvTest__AgvTest_HPP_

#include <cmath>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include <agv_msgs/msg/agv_drive.hpp>

#define DRIVE_WHEEL_RADIUS 0.155 // 미터

class AgvTest : public rclcpp::Node
{
public:
  explicit AgvTest(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
  virtual ~AgvTest();

private:
  void goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  void agv_status_callback(const agv_msgs::msg::AgvDrive::SharedPtr msg);
  void publish_states();
  void get_node_inform();
  void publish_goal();
  void agv_action(int agv_mode_);

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_subscriber_;

  rclcpp::Publisher<agv_msgs::msg::AgvDrive>::SharedPtr drive_mode_publisher_;
  rclcpp::Subscription<agv_msgs::msg::AgvDrive>::SharedPtr drive_mode_subscriber_;

  int move_status_;

  int sending_drive_mode_;
  int drive_mode_;
  int agv_mode;

  std::string node_name_;

  float x_dep_;
  float y_dep_;
  float z_dep_;
  float w_dep_;
  float x_des_;
  float y_des_;
  float z_des_;
  float w_des_;
  float x_;
  float y_;
  float z_;
  float w_;

  geometry_msgs::msg::PoseStamped goal_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif // AgvTest__AgvTest_HPP_
