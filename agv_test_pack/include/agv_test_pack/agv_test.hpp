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
#include "actionlib_msgs/msg/goal_status_array.hpp"

#define DRIVE_WHEEL_RADIUS 0.155 // 미터

class AgvTest : public rclcpp::Node
{
public:
  explicit AgvTest(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
  virtual ~AgvTest();

private:
  void goal_status_callback(const actionlib_msgs::msg::GoalStatusArray::SharedPtr msg);
  void publish_states();

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Subscription<actionlib_msgs::msg::GoalStatusArray>::SharedPtr goal_status_subscriber_;

  int move_status_;
  int status_index_;

  geometry_msgs::msg::PoseStamped goal_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif // AgvTest__AgvTest_HPP_
