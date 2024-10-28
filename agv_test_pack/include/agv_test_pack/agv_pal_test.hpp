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
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "actionlib_msgs/msg/goal_status_array.hpp"

#define DRIVE_WHEEL_RADIUS 0.155 // 미터

class AgvPalTest : public rclcpp::Node
{
public:
  explicit AgvPalTest(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
  virtual ~AgvPalTest();

private:
  void bumper_status_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg);
  void publish_states();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_publisher_;
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr bumper_l_status_subscriber_;

  bool bumper_l;

  geometry_msgs::msg::Twist stop_vel_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif // AgvPalTest__AgvPalTest_HPP_
