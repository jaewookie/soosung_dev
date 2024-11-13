#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "agv_test_pack/agv_pal_test.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

AgvPalTest::AgvPalTest(const rclcpp::NodeOptions &node_options)
    : Node("agv_pal_test", node_options),
      bumper_l(false),
      bumper_r(false),
      x_(0.0),
      fork_height_(0.0),
      bumper_react_(false)
{
  RCLCPP_INFO(this->get_logger(), "Wheel Drive Controller initialized");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  bumper_l_status_subscriber_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
      "/bumper_l/bumper_states", QOS_RKL10V, std::bind(&AgvPalTest::bumper_l_status_callback, this, std::placeholders::_1));
  bumper_r_status_subscriber_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
      "/bumper_r/bumper_states", QOS_RKL10V, std::bind(&AgvPalTest::bumper_r_status_callback, this, std::placeholders::_1));
  stop_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  mast_publisher_ = this->create_publisher<agv_msgs::msg::ForkControl>("/mast_vel", 10);

  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", QOS_RKL10V, std::bind(&AgvPalTest::fork_height_status_callback, this, std::placeholders::_1));

  drive_mode_subscriber_ = this->create_subscription<agv_msgs::msg::AgvDrive>(
      "/agv_state", QOS_RKL10V, std::bind(&AgvPalTest::agv_status_callback, this, std::placeholders::_1));
  drive_mode_publisher_ = this->create_publisher<agv_msgs::msg::AgvDrive>("/agv_state", 10);

  timer_ = this->create_wall_timer(10ms, std::bind(&AgvPalTest::fork_action, this));
}

AgvPalTest::~AgvPalTest() {}

void AgvPalTest::fork_height_status_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name[0] == "fork_joint")
  {
    fork_height_ = msg->position[0];
  }
  RCLCPP_INFO(this->get_logger(), "%f", fork_height_);
}

void AgvPalTest::fork_action()
{
  agv_mode = drive_mode_;
  if (agv_mode == 3)
  {
    bumper_react_ = false;
    if (fork_height_ < -0.1)
    {
      x_ = -0.1;
      fork_vel_.fork_vel = 0.0;
    }
    else
    {
      fork_vel_.fork_vel = -0.25;
    }
    publish_cmd_vel();
  }

  else if (agv_mode == 4)
  {
    if (fork_height_ > 0.4)
    {
      sending_drive_mode_ = 5;
      // sending_drive_mode_ = 6; // test
      fork_vel_.fork_vel = 0.0;
      publish_states();
    }
    else
    {
      fork_vel_.fork_vel = 0.25;
      x_ = 0.0;
    }
    publish_cmd_vel();
  }
  else if (agv_mode == 6)
  {
    if (fork_height_ < -0.1)
    {
      sending_drive_mode_ = 7;
      fork_vel_.fork_vel = 0.0;
      publish_states();
    }
    else
    {
      x_ = 0.0;
      fork_vel_.fork_vel = -0.25;
    }
    publish_cmd_vel();
  }
  else if (agv_mode == 7)
  {
    x_ = 0.1;
    sending_drive_mode_ = 1;
    publish_cmd_vel();
    sleep(4);
    if (fork_height_ < 0.0)
    {
      fork_vel_.fork_vel = 0.25;
    }else{
      fork_vel_.fork_vel = 0.0;
    }
    publish_states();
  }
  mast_publisher_->publish(fork_vel_);
}

void AgvPalTest::bumper_l_status_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
{
  if (!msg->states.empty())
  {
    bumper_l = true;
    bumper_checker(bumper_l, bumper_r);
  }
  else
  {
    bumper_l = false;
    return;
  }
}

void AgvPalTest::bumper_r_status_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
{
  if (!msg->states.empty())
  {
    bumper_r = true;
    bumper_checker(bumper_l, bumper_r);
  }
  else
  {
    bumper_r = false;
    return;
  }
}

void AgvPalTest::bumper_checker(bool bumper_l_state, bool bumper_r_state)
{
  if (bumper_l_state == true && bumper_r_state == true)
  {
    if (bumper_react_ == false)
    {
      bumper_react_ = true;
      sending_drive_mode_ = 4;
      publish_states();
    }
  }
  else if (bumper_l_state == false && bumper_r_state == true)
  {
    cmd_vel_.linear.x = x_;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    cmd_vel_.angular.z = -0.01;
    stop_publisher_->publish(cmd_vel_);
  }
  else if (bumper_l_state == true && bumper_r_state == false)
  {
    cmd_vel_.linear.x = x_;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.linear.z = 0.0;
    cmd_vel_.angular.x = 0.0;
    cmd_vel_.angular.y = 0.0;
    cmd_vel_.angular.z = 0.01;
    stop_publisher_->publish(cmd_vel_);
  }
}

void AgvPalTest::publish_cmd_vel()
{
  sleep(1);
  // fork 높이 바닥 = -0.11 상차 = 0.4 fork_joint
  cmd_vel_.linear.x = x_;
  cmd_vel_.linear.y = 0.0;
  cmd_vel_.linear.z = 0.0;
  cmd_vel_.angular.x = 0.0;
  cmd_vel_.angular.y = 0.0;
  cmd_vel_.angular.z = 0.0;
  stop_publisher_->publish(cmd_vel_);
}

void AgvPalTest::agv_status_callback(const agv_msgs::msg::AgvDrive::SharedPtr msg)
{
  drive_mode_ = msg->drive_state;
  RCLCPP_INFO(this->get_logger(), "%d", drive_mode_);
  // publish_cmd_vel();
}

void AgvPalTest::publish_states()
{
  auto message = agv_msgs::msg::AgvDrive();
  message.drive_state = sending_drive_mode_;
  drive_mode_publisher_->publish(message);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvPalTest>());
  rclcpp::shutdown();
  return 0;
}
