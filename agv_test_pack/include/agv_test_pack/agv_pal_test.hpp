#ifndef AgvPalTest__AgvPalTest_HPP_
#define AgvPalTest__AgvPalTest_HPP_

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
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <agv_msgs/msg/fork_control.hpp>
#include <agv_msgs/msg/agv_drive.hpp>

#define DRIVE_WHEEL_RADIUS 0.155 // 미터

class AgvPalTest : public rclcpp::Node
{
public:
  explicit AgvPalTest(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
  virtual ~AgvPalTest();

private:
  void agv_status_callback(const agv_msgs::msg::AgvDrive::SharedPtr msg);
  void bumper_l_status_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg);
  void bumper_r_status_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg);
  void fork_height_status_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_cmd_vel();
  void bumper_checker(bool bumper_l_state, bool bumper_r_state);
  void publish_states();
  void fork_action();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_publisher_;
  rclcpp::Publisher<agv_msgs::msg::ForkControl>::SharedPtr mast_publisher_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr bumper_l_status_subscriber_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr bumper_r_status_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  rclcpp::Publisher<agv_msgs::msg::AgvDrive>::SharedPtr drive_mode_publisher_;
  rclcpp::Subscription<agv_msgs::msg::AgvDrive>::SharedPtr drive_mode_subscriber_;

  bool bumper_l;
  bool bumper_r;

  bool bumper_react_;

  int sending_drive_mode_;
  int drive_mode_;
  int agv_mode;

  // agv 속도
  float x_;
  float fork_height_;

  geometry_msgs::msg::Twist cmd_vel_;
  agv_msgs::msg::ForkControl fork_vel_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif // AgvPalTest__AgvPalTest_HPP_
