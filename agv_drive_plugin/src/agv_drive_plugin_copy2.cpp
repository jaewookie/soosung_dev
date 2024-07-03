#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "agv_drive_plugin/agv_drive_plugin.hpp"

#include <memory>

namespace gazebo_ros
{

  class AgvDrivePluginPrivate
  {
  public:
    // ROS node for communication, managed by gazebo_ros.
    gazebo_ros::Node::SharedPtr ros_node_;

    // The joint that controls the movement of the belt:
    gazebo::physics::JointPtr drive_wheel_joint_;
    gazebo::physics::JointPtr steering_joint_;

    // Additional parametres:
    double lin_vel_ = 0.0;
    double ang_vel_ = 0.0;
    double wheel_radius_ = 0.133;
    double wheel_base_ = 0.5;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    rclcpp::Time last_time_;

    // PUBLISH ConveyorBelt status:
    void PublishStatus(); // Method to publish status.
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // WORLD UPDATE event:
    void OnUpdate();
    void OnCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    rclcpp::Time last_publish_time_;
    int update_ns_;
    gazebo::event::ConnectionPtr update_connection_; // Connection to world update event. Callback is called while this is alive.
  };

  AgvDrivePlugin::AgvDrivePlugin()
      : impl_(std::make_unique<AgvDrivePluginPrivate>())
  {
  }

  AgvDrivePlugin::~AgvDrivePlugin()
  {
  }

  void AgvDrivePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    // Create ROS2 node:
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // OBTAIN -> BELT JOINT:
    impl_->drive_wheel_joint_ = _model->GetJoint("drive_wheel_joint");
    impl_->steering_joint_ = _model->GetJoint("steering_joint");

    if (!impl_->drive_wheel_joint_ || !impl_->steering_joint_)
    {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "joint not found, unable to start conveyor plugin.");
      return;
    }

    // Create status publisher
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&AgvDrivePluginPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

    impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    impl_->odom_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    impl_->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
        impl_->ros_node_);

    double publish_rate = 1000;

    impl_->update_ns_ = int((1 / publish_rate) * 1e9);

    impl_->last_publish_time_ = impl_->ros_node_->get_clock()->now();

    // Create a connection so the OnUpdate function is called at every simulation iteration.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&AgvDrivePluginPrivate::OnUpdate, impl_.get()));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "GAZEBO ConveyorBelt plugin loaded successfully.");
  }

  void AgvDrivePluginPrivate::OnCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    lin_vel_ = msg->linear.x;
    ang_vel_ = msg->angular.z;
  }

  void AgvDrivePluginPrivate::OnUpdate()
  {
    gazebo::common::Time current_time = _model->get_clock()->now();
    double dt = (current_time - last_time_);
    last_time_ = current_time;

    // Calculate the wheel velocity and steering angle
    double wheel_velocity = lin_vel_ / wheel_radius_;
    double steering_angle = atan(wheel_base_ * ang_vel_ / lin_vel_);

    // Set joint positions
    drive_wheel_joint_->SetVelocity(0, wheel_velocity);
    steering_joint_->SetPosition(0, steering_angle);

    // Publish joint states
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = node_->get_clock()->now();
    joint_state_msg.name = {"steering_joint", "drive_wheel_joint"};
    joint_state_msg.position = {steering_joint_->Position(0), drive_wheel_joint_->Position(0)};
    joint_state_msg.velocity = {0.0, wheel_velocity};
    joint_state_pub_->publish(joint_state_msg);

    // Publish odometry
    double delta_x = lin_vel_ * cos(yaw_) * dt;
    double delta_y = lin_vel_ * sin(yaw_) * dt;
    double delta_yaw = ang_vel_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    yaw_ += delta_yaw;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = node_->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    odom_msg.twist.twist.linear.x = lin_vel_;
    odom_msg.twist.twist.angular.z = ang_vel_;
    odom_pub_->publish(odom_msg);

    // Publish transform
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = node_->get_clock()->now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(odom_tf);
  }

  GZ_REGISTER_model_PLUGIN(AgvDrivePlugin)
} // namespace gazebo_ros
