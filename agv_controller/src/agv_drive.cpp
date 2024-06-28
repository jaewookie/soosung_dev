#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "agv_controller/agv_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

AgvDrive::AgvDrive(const rclcpp::NodeOptions &node_options)
    : Node("agv_drive", node_options),
      lin_vel_(0.0), // 초기 선형 속도는 0으로 설정
      ang_vel_(0.0), // 초기 각속도는 0으로 설정
      wheel_radius_(DRIVE_WHEEL_RADIUS)
{
  RCLCPP_INFO(this->get_logger(), "Wheel Drive Controller initialized");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  cmdvel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", QOS_RKL10V, std::bind(&AgvDrive::cmd_vel_callback, this, std::placeholders::_1));

  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  timer_ = this->create_wall_timer(10ms, std::bind(&AgvDrive::publish_states, this));
}

AgvDrive::~AgvDrive() {}

void AgvDrive::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  lin_vel_ = msg->linear.x;  // cmd_vel로부터 선형 속도를 갱신
  ang_vel_ = msg->angular.z; // cmd_vel로부터 각속도를 갱신, 현재 사용하지 않음
}

void AgvDrive::publish_states()
{
  rclcpp::Time now = this->now();
  double dt = 0.01; // 시간 간격

  static double wheel_position = 0.0;
  double wheel_velocity = lin_vel_ / wheel_radius_;

  wheel_position += wheel_velocity * dt;

  // Joint states 퍼블리시
  joint_state_.header.stamp = now;
  joint_state_.name = {"steering_joint", "drive_wheel_joint"};
  joint_state_.position = {0.0, wheel_position};
  joint_state_.velocity = {0.0, wheel_velocity};

  joint_state_publisher_->publish(joint_state_);

  // Odometry 퍼블리시

  static double x = 0.0, y = 0.0, theta = 0.0;

  double delta_x = 0;
  double delta_y = lin_vel_ * dt;
  double delta_theta = ang_vel_ * dt; // 각속도는 현재 사용하지 않음

  y += delta_y;
  theta += delta_theta;

  odom_.header.stamp = now;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";

  odom_.pose.pose.position.x = 0.0;
  odom_.pose.pose.position.y = y;
  odom_.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  tf2::convert(q, odom_.pose.pose.orientation);

  odom_.twist.twist.linear.y = lin_vel_;
  odom_.twist.twist.angular.z = ang_vel_; // 각속도는 현재 사용하지 않음

  odom_publisher_->publish(odom_);

  // TF 퍼블리시
  odom_tf_.header.stamp = now;
  odom_tf_.header.frame_id = "odom";
  odom_tf_.child_frame_id = "base_footprint";
  odom_tf_.transform.translation.x = 0.0;
  odom_tf_.transform.translation.y = y;
  odom_tf_.transform.translation.z = 0.0;
  tf2::convert(q, odom_.pose.pose.orientation);

  tf_broadcaster_->sendTransform(odom_tf_);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvDrive>());
  rclcpp::shutdown();
  return 0;
}
