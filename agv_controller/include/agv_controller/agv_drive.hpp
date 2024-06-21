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

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#define DRIVE_WHEEL_RADIUS 0.155 // mm

class AgvDrive : public rclcpp::Node
{
public:
  using DrivingMotorRPM = std_msgs::msg::String;

  explicit AgvDrive(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
  virtual ~AgvDrive();

  float agv_drive_motor_formula(const float &a, const float &b, const float &c);

private:
  void publish_drive_rpm();
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmdvel_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr drive_rpm_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  float lin_vel_;
  float ang_vel_;

  float drive_rpm_;

  std::string argument_formula_;
  std::vector<std::string> operator_;
};
#endif // AgvDrive__AgvDrive_HPP_
