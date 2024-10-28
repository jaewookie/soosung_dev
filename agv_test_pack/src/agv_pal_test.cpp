#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "agv_test_pack/agv_pal_test.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/contacts_state.hpp"
#include "actionlib_msgs/msg/goal_status_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

AgvPalTest::AgvPalTest(const rclcpp::NodeOptions &node_options)
    : Node("agv_pal_test", node_options),
      bumper_l(false)
{
  RCLCPP_INFO(this->get_logger(), "Wheel Drive Controller initialized");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  bumper_l_status_subscriber_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
      "/bumper_l/bumper_states", QOS_RKL10V, std::bind(&AgvPalTest::bumper_status_callback, this, std::placeholders::_1));
  stop_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

AgvPalTest::~AgvPalTest() {}

void AgvPalTest::bumper_status_callback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
{
  if(msg->states.empty()){
    RCLCPP_INFO(this->get_logger(),"is empty");
  }else{
    RCLCPP_INFO(this->get_logger(),"%s", msg->states[0].info.c_str());
    stop_vel_.linear.x = 0.0;
    stop_vel_.linear.y = 0.0;
    stop_vel_.linear.z = 0.0;
    stop_vel_.angular.x = 0.0;
    stop_vel_.angular.y = 0.0;
    stop_vel_.angular.z = 0.0;
    stop_publisher_->publish(stop_vel_);
  }
  // if(msg->states[0].info == ""){
  //   RCLCPP_INFO(this->get_logger(),"is empty");
  // }else{
  //   RCLCPP_INFO(this->get_logger(),"is not empty");
  // }
}

void AgvPalTest::publish_states()
{
  rclcpp::Time now = this->now();

  // // Joint states 퍼블리시
  // joint_state_.header.stamp = now;
  // joint_state_.name = {"steering_joint", "drive_wheel_joint"};
  // joint_state_.position = {0.0, wheel_position};
  // joint_state_.velocity = {0.0, wheel_velocity};

  // goal_pose_publisher_->publish(goal_pose_);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvPalTest>());
  rclcpp::shutdown();
  return 0;
}
