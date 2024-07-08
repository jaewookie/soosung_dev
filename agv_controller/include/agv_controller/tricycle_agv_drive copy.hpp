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
    TricycleAGVDrive();

private:
    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void onUpdate();
    void updateOdometry(const rclcpp::Time &current_time);
    void publishOdometryMsg(const rclcpp::Time &current_time);
    void publishWheelTransforms(const rclcpp::Time &current_time);
    void publishWheelJointStates(const rclcpp::Time &current_time);
    void motorController(double target_speed, double dt);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double drive_wheel_radius_;
    double front_wheel_radius_;
    double max_wheel_accel_;
    double max_wheel_decel_;
    double max_wheel_speed_tol_;
    double wheel_separation_;

    geometry_msgs::msg::Twist cmd_;
    sensor_msgs::msg::JointState joint_state_;
    geometry_msgs::msg::Pose2D pose_;

    std::mutex lock_;

    rclcpp::Time last_update_time_;

    nav_msgs::msg::Odometry odom_;

    bool publish_wheel_tf_;
    bool publish_wheel_joint_state_;
    bool publish_odom_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // TRICYCLE_AGV_DRIVE_HPP
