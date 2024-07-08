#ifndef TRICYCLE_AGV_DRIVE_HPP
#define TRICYCLE_AGV_DRIVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <mutex>

class TricycleAGVDrive : public rclcpp::Node
{
public:
    explicit TricycleAGVDrive(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
    virtual ~TricycleAGVDrive();

private:
void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
void OnUpdate();
void MotorController(double target_speed, double dt);
void UpdateOdometry(double dt);
void publishOdometryMsg(const rclcpp::Time &current_time);

//sub and pub
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_subscriber_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

//tf
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//msgs
geometry_msgs::msg::Twist cmd_;
nav_msgs::msg::Odometry odom_;
geometry_msgs::msg::Pose2D pose_encoder_;

//utill
std::mutex lock_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Time last_update_time_;

//var
  float lin_vel_;
  float ang_vel_;
  double wheel_radius_;
  // motor control
  float max_wheel_accel_;
  double current_speed_;
  double drive_wheel_speed_;
  double drive_rpm_;
  // 시뮬레이션용
  double front_wheel_speed_;
  double front_wheel_l_pos_x_;
  double front_wheel_r_pos_x_;
  double front_wheel_pos_y_;
};

#endif // TRICYCLE_AGV_DRIVE_HPP
