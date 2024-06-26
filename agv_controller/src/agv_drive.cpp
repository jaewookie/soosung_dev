#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "agv_controller/agv_drive.hpp"

using namespace std::chrono_literals;

AgvDrive::AgvDrive(const rclcpp::NodeOptions &node_options)
    : Node("agv_drive", node_options),
      lin_vel_(0.0),
      ang_vel_(0.0),
      drive_rpm_(0.0),
      current_pose_(0.0)
{
  RCLCPP_INFO(this->get_logger(), "Wheel Drive Controller");

  // QOS setting
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
      rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // cmd_vel 수신 및 모터 RPM 계산
  cmdvel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      QOS_RKL10V,
      [this](const geometry_msgs::msg::Twist::SharedPtr twist_msg) -> void
      {
        lin_vel_ = twist_msg->linear.x;
        ang_vel_ = twist_msg->angular.z;
        drive_rpm_ = this->agv_drive_motor_formula(lin_vel_, DRIVE_WHEEL_RADIUS);
      });

  // tf2
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  auto broadcast = [this]() -> void
  {
    static double x = 0.0;

    tf_stamped_list_.clear();

    geometry_msgs::msg::TransformStamped tf_stamped;

    tf_stamped.header.stamp = this->now();
    tf_stamped.header.frame_id = "base_footprint";
    tf_stamped.child_frame_id = "base_link";
    tf_stamped.transform.translation.x = 0.0;
    tf_stamped.transform.translation.y = x;
    tf_stamped.transform.translation.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, 0);

    tf_stamped.transform.rotation.x = quaternion.x();
    tf_stamped.transform.rotation.y = quaternion.y();
    tf_stamped.transform.rotation.z = quaternion.z();
    tf_stamped.transform.rotation.w = quaternion.w();

    tf_stamped_list_.push_back(tf_stamped);

    tf_stamped.header.frame_id = "base_link";
    tf_stamped.child_frame_id = "drive_wheel";
    tf_stamped.transform.translation.x = 0.0;
    tf_stamped.transform.translation.y = x;
    tf_stamped.transform.translation.z = 0.0;

    quaternion.setRPY((0.01 * M_PI * drive_rpm_ / 30), 0, 0);

    tf_stamped.transform.rotation.x = quaternion.x();
    tf_stamped.transform.rotation.y = quaternion.y();
    tf_stamped.transform.rotation.z = quaternion.z();
    tf_stamped.transform.rotation.w = quaternion.w();

    tf_stamped_list_.push_back(tf_stamped);

    x += 1;

    tf_broadcaster_->sendTransform(tf_stamped_list_);
  };
  timer_ = this->create_wall_timer(10ms, broadcast);

  // joint_state Publish
  agv_joint_publisher_ = this->create_publisher<JointState>("joint_states", QOS_RKL10V);
  timer_ = this->create_wall_timer(10ms, std::bind(&AgvDrive::publish_drive_rpm, this));
}

AgvDrive::~AgvDrive()
{
}

void AgvDrive::publish_drive_rpm()
{
  sensor_msgs::msg::JointState joint_msg;

// joint_msg.position stacking!

  joint_msg.header.frame_id = "";
  joint_msg.header.stamp = this->now();
  joint_msg.name.push_back(drive_joint_);

  joint_msg.position.push_back((0.01 * M_PI * drive_rpm_ / 30));
  joint_msg.velocity.push_back(drive_rpm_);

  agv_joint_publisher_->publish(joint_msg);
}

float AgvDrive::agv_drive_motor_formula(
    const float &lin_vel,
    const float &wheel_radius)
{
  float wheel_rpm;

  wheel_rpm = ((lin_vel * 60) / (M_PI * wheel_radius * 2));

  return wheel_rpm;
}
