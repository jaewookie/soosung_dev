// Copyright 2019 ROBOTIS CO., LTD.
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
//
// Authors: Taehun Lim (Darby), Ryan Shim

#include "agv_gazebo/agv_drive.hpp"

#include <memory>

using namespace std::chrono_literals;

AgvDrive::AgvDrive()
    : Node("agv_drive_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  // scan_data_[0] = 0.0;
  // scan_data_[1] = 0.0;
  // scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise subscribers
  // scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  //     "scan",
  //     rclcpp::SensorDataQoS(),
  //     std::bind(
  //         &AgvDrive::scan_callback,
  //         this,
  //         std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos, std::bind(&AgvDrive::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&AgvDrive::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Agv simulation node has been initialised");
}

AgvDrive::~AgvDrive()
{
  RCLCPP_INFO(this->get_logger(), "Agv simulation node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void AgvDrive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

// void AgvDrive::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   uint16_t scan_angle[3] = {0, 30, 330};

//   for (int num = 0; num < 3; num++)
//   {
//     if (std::isinf(msg->ranges.at(scan_angle[num])))
//     {
//       scan_data_[num] = msg->range_max;
//     }
//     else
//     {
//       scan_data_[num] = msg->ranges.at(scan_angle[num]);
//     }
//   }
// }

void AgvDrive::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void AgvDrive::update_callback()
{
  static uint8_t agv_state_num = 0;
  double escape_range = 30.0 * DEG2RAD;
  // double check_forward_dist = 0.7;
  // double check_side_dist = 0.6;

  switch (agv_state_num)
  {
  // case GET_AGV_DIRECTION:
  //   if (scan_data_[CENTER] > check_forward_dist)
  //   {
  //     if (scan_data_[LEFT] < check_side_dist)
  //     {
  //       prev_robot_pose_ = robot_pose_;
  //       agv_state_num = AGV_RIGHT_TURN;
  //     }
  //     else if (scan_data_[RIGHT] < check_side_dist)
  //     {
  //       prev_robot_pose_ = robot_pose_;
  //       agv_state_num = AGV_LEFT_TURN;
  //     }
  //     else
  //     {
  //       agv_state_num = AGV_DRIVE_FORWARD;
  //     }
  //   }

  //   if (scan_data_[CENTER] < check_forward_dist)
  //   {
  //     prev_robot_pose_ = robot_pose_;
  //     agv_state_num = AGV_RIGHT_TURN;
  //   }
  //   break;

  case AGV_DRIVE_FORWARD:
    update_cmd_vel(LINEAR_VELOCITY, 0.0);
    agv_state_num = GET_AGV_DIRECTION;
    break;

  case AGV_RIGHT_TURN:
    if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
    {
      agv_state_num = GET_AGV_DIRECTION;
    }
    else
    {
      update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY);
    }
    break;

  case AGV_LEFT_TURN:
    if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
    {
      agv_state_num = GET_AGV_DIRECTION;
    }
    else
    {
      update_cmd_vel(0.0, ANGULAR_VELOCITY);
    }
    break;

  default:
    agv_state_num = GET_AGV_DIRECTION;
    break;
  }
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvDrive>());
  rclcpp::shutdown();

  return 0;
}