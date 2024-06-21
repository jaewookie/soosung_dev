#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "agv_controller/agv_drive.hpp"

using namespace std::chrono_literals;

AgvDrive::AgvDrive(const rclcpp::NodeOptions & node_options)
    : Node("agv_drive", node_options),
      lin_vel_(0.0),
      ang_vel_(0.0),
      drive_rpm_(0.0)
{
  RCLCPP_INFO(this->get_logger(), "Run Wheel Drive");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  cmdvel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    QOS_RKL10V,
    [this](const geometry_msgs::msg::Twist::SharedPtr twist_msg) -> void
    {
      lin_vel_ = twist_msg->linear.x;
      ang_vel_ = twist_msg->angular.z;

      RCLCPP_INFO(this->get_logger(), "Subscribed velocity of Linear: %.2f", lin_vel_);
      RCLCPP_INFO(this->get_logger(), "Subscribed velocity of Angular: %.2f", ang_vel_);

      drive_rpm_ = this->agv_drive_motor_formula(lin_vel_, ang_vel_, DRIVE_WHEEL_RADIUS);
    }
  );

  drive_rpm_publisher_ = this->create_publisher<DrivingMotorRPM>("motor_rpm", QOS_RKL10V);

  timer_ = this->create_wall_timer(1s, std::bind(&AgvDrive::publish_drive_rpm, this));
}

AgvDrive::~AgvDrive()
{
}

void AgvDrive::publish_drive_rpm()
{
  std_msgs::msg::String msg;
  msg.data = std::to_string(drive_rpm_);
  RCLCPP_INFO(this->get_logger(), "Drive Motor RPM = %s", msg.data.c_str());
  drive_rpm_publisher_->publish(msg);
}

float AgvDrive::agv_drive_motor_formula(
  const float & lin_vel,
  const float & ang_vel,
  const float & wheel_radius)
  {
    float wheel_rpm;

    wheel_rpm = (lin_vel/(2*3.14*wheel_radius));

    return wheel_rpm;
  }

// using std::placeholders::_1;

// class AgvVelSubscriber : public rclcpp::Node
// {
//   float* lin_vel;

// public:
//   AgvVelSubscriber()
//       : Node("AgvVel_subscriber")
//   {
//     auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

//     AgvVel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
//         "/cmd_vel",
//         qos_profile,
//         std::bind(&AgvVelSubscriber::cmdvel_callback, this, _1));
//   }

// private:
//   void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg) const
//   {
//     RCLCPP_INFO(this->get_logger(), "Received message: '%f'", twist_msg->linear.x);
//   }

//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr AgvVel_subscriber_;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<AgvVelSubscriber>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
