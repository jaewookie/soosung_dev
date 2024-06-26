// Copyright 2021 OROCA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AgvDrive__AgvDrive_HPP_
#define AgvDrive__AgvDrive_HPP_

#include <cmath>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#define DRIVE_WHEEL_RADIUS 0.155 // mm
#define MAX_WHEEL_ACCEL 0
#define MAX_WHEEL_DECEL 0
#define MAX_WHEEL_SPEED_TOL 0.01

class AgvDrive : public rclcpp::Node
{
public:
  using DrivingMotorRPM = std_msgs::msg::String;
  using JointState = sensor_msgs::msg::JointState;

  explicit AgvDrive(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
  virtual ~AgvDrive();

  float agv_drive_motor_formula(const float &a, const float &c);

private:
  void publish_drive_rpm();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmdvel_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr agv_joint_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string drive_joint_ = "drive_wheel_joint";
  std::vector<std::string> joint_name_;

  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::vector<geometry_msgs::msg::TransformStamped> tf_stamped_list_;

  // cmd_vel to Drive_motor_RPM

  float lin_vel_;
  float ang_vel_;

  float drive_rpm_;

  float current_pose_;

  // std::string argument_formula_;
  // std::vector<std::string> operator_;
};
#endif // AgvDrive__AgvDrive_HPP_
