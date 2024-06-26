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
// Authors: Yoonseok Pyo, Ryan Shim

#include "agv_controller/agv_drive_fake.hpp"

#include <memory>
#include <string>

using namespace std::chrono_literals;

AgvFake::AgvFake()
: Node("agv_fake")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  init_variables();

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", qos);

  // Initialise subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", \
    qos, \
    std::bind(
      &AgvFake::command_velocity_callback, \
      this, \
      std::placeholders::_1));

  /************************************************************
  ** initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&AgvFake::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Agv fake node has been initialised");
}

AgvFake::~AgvFake()
{
  RCLCPP_INFO(this->get_logger(), "Agv fake node has been terminated");
}

void AgvFake::init_variables()
{

  // Initialise variables
  wheel_radius_ = 0.133;
  wheel_speed_cmd_[STEERING] = 0.0;
  wheel_speed_cmd_[DRIVE] = 0.0;
  goal_linear_velocity_ = 0.0;
  goal_angular_velocity_ = 0.0;
  cmd_vel_timeout_ = 1.0;
  last_position_[STEERING] = 0.0;
  last_position_[DRIVE] = 0.0;
  last_velocity_[STEERING] = 0.0;
  last_velocity_[DRIVE] = 0.0;

  joint_states_.header.frame_id="base_footprint";
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";

  // TODO(Will Son): Find more accurate covariance
  // double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
  //                       0, 0.1,   0,   0,   0, 0,
  //                       0,   0, 1e6,   0,   0, 0,
  //                       0,   0,   0, 1e6,   0, 0,
  //                       0,   0,   0,   0, 1e6, 0,
  //                       0,   0,   0,   0,   0, 0.2};
  // memcpy(&(odom_.pose.covariance), pcov, sizeof(double)*36);
  // memcpy(&(odom_.twist.covariance), pcov, sizeof(double)*36);

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back("steering_joint");
  joint_states_.name.push_back("drive_wheel_joint");
  joint_states_.position.resize(2, 0.0);
  joint_states_.velocity.resize(2, 0.0);
  joint_states_.effort.resize(2, 0.0);

  prev_update_time_ = this->now();
  last_cmd_vel_time_ = this->now();
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void AgvFake::command_velocity_callback(
  const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
  last_cmd_vel_time_ = this->now();

  goal_linear_velocity_ = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[STEERING] = 0;
  wheel_speed_cmd_[DRIVE] = goal_linear_velocity_;
}

/********************************************************************************
** Update functions
********************************************************************************/
void AgvFake::update_callback()
{
  rclcpp::Time time_now = this->now();
  rclcpp::Duration duration(time_now - prev_update_time_);
  prev_update_time_ = time_now;

  // zero-ing after timeout (stop the robot if no cmd_vel)
  if ((time_now - last_cmd_vel_time_).nanoseconds() / 1e9 > cmd_vel_timeout_) {
    wheel_speed_cmd_[STEERING] = 0.0;
    wheel_speed_cmd_[DRIVE] = 0.0;
  }

  // odom
  update_odometry(duration);
  odom_.header.stamp = time_now;
  odom_pub_->publish(odom_);

  // joint_states
  update_joint_state();
  joint_states_.header.stamp = time_now;
  joint_states_pub_->publish(joint_states_);

  // tf
  geometry_msgs::msg::TransformStamped odom_tf;
  update_tf(odom_tf);
  tf2_msgs::msg::TFMessage odom_tf_msg;
  odom_tf_msg.transforms.push_back(odom_tf);
  tf_pub_->publish(odom_tf_msg);
}

bool AgvFake::update_odometry(const rclcpp::Duration & duration)
{
  double wheel_d;  // rotation value of wheel [rad]
  double delta_s;
  double v[2], w[2];
  double step_time = duration.nanoseconds() / 1e9;  // [sec]

  wheel_d = 0.0;
  delta_s = 0.0;

  // v = translational velocity [m/s]
  // w = rotational velocity [rad/s]

  v[DRIVE] = wheel_speed_cmd_[DRIVE];
  w[DRIVE] = v[DRIVE] / wheel_radius_;

  last_velocity_[DRIVE] = w[DRIVE];

  wheel_d = w[DRIVE] * step_time;

  if (isnan(wheel_d)) {
    wheel_d = 0.0;
  }

  last_position_[DRIVE] += wheel_d;

  delta_s = wheel_radius_ * wheel_d;

  // compute odometric pose
  odom_pose_[0] += 0.0;
  odom_pose_[1] += delta_s;
  odom_pose_[2] += 0.0;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = 0.0;     // v
  odom_vel_[1] = delta_s / step_time;
  odom_vel_[2] = 0.0;  // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_pose_[2]);

  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  return true;
}

void AgvFake::update_joint_state()
{
  joint_states_.position[STEERING] = last_position_[STEERING];
  joint_states_.position[DRIVE] = last_position_[DRIVE];
  joint_states_.velocity[STEERING] = last_velocity_[STEERING];
  joint_states_.velocity[DRIVE] = last_velocity_[DRIVE];
}

void AgvFake::update_tf(geometry_msgs::msg::TransformStamped & odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvFake>());
  rclcpp::shutdown();

  return 0;
}
