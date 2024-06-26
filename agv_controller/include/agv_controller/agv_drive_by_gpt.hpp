#ifndef AgvDriveGPT__AgvDriveGPT_HPP_
#define AgvDriveGPT__AgvDriveGPT_HPP_

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
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#define DRIVE_WHEEL_RADIUS 0.155 // 미터

class AgvDriveGPT : public rclcpp::Node
{
public:
  explicit AgvDriveGPT(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
  virtual ~AgvDriveGPT();

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publish_states();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  float lin_vel_;
  float ang_vel_;
  double wheel_radius_;

  sensor_msgs::msg::JointState joint_state_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::TransformStamped odom_tf_;

  rclcpp::TimerBase::SharedPtr timer_;
};
#endif // AgvDriveGPT__AgvDriveGPT_HPP_
