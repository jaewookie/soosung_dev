#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sdf/sdf.hh>

namespace gazebo
{
  class AgvDrivePlugin : public ModelPlugin
  {
  public:
    AgvDrivePlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      // Initialize ROS node
      // node_ = gazebo_ros::Node::Get(sdf);

      // Get the joints
      // drive_wheel_joint_ = model->GetJoint("drive_wheel_joint");
      // steering_joint_ = model->GetJoint("steering_joint");

      // if (!drive_wheel_joint_ || !steering_joint_)
      // {
      //   RCLCPP_ERROR(node_->get_logger(), "Could not find joints");
      //   return;
      // }

      // ROS2 publishers and subscribers
      /*
      cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
          "/cmd_vel", 10, std::bind(&AgvDrivePlugin::OnCmdVel, this, std::placeholders::_1));
*/
      // joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

      // odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

      // tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

      // update_connection_ = event::Events::ConnectWorldUpdateBegin(
      // std::bind(&AgvDrivePlugin::OnUpdate, this));

      RCLCPP_INFO(node_->get_logger(), "AgvDrivePlugin loaded successfully");
    }

  private:
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      RCLCPP_INFO(node_->get_logger(), "aaaaaa");
      lin_vel_ = msg->linear.x;
      ang_vel_ = msg->angular.z;
      RCLCPP_INFO(node_->get_logger(), "aaaaaa2");
    }

    void OnUpdate()
    {
      RCLCPP_INFO(node_->get_logger(), "bbbbbbbb");
      common::Time current_time = model_->GetWorld()->SimTime();
      RCLCPP_INFO(node_->get_logger(), "cccccc");
      double dt = (current_time - last_time_).Double();
      RCLCPP_INFO(node_->get_logger(), "ddddd");
      last_time_ = current_time;
      RCLCPP_INFO(node_->get_logger(), "eeeee");

      // Calculate the wheel velocity and steering angle
      RCLCPP_INFO(node_->get_logger(), "bbbbbbbb2");
      double wheel_velocity = lin_vel_ / wheel_radius_;
      double steering_angle = atan(wheel_base_ * ang_vel_ / lin_vel_);

      // Set joint positions
      RCLCPP_INFO(node_->get_logger(), "bbbbbbbb3");
      drive_wheel_joint_->SetVelocity(0, wheel_velocity);
      steering_joint_->SetPosition(0, steering_angle);

      // Publish joint states
      RCLCPP_INFO(node_->get_logger(), "bbbbbbbb4");
      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = node_->get_clock()->now();
      joint_state_msg.name = {"steering_joint", "drive_wheel_joint"};
      joint_state_msg.position = {steering_joint_->Position(0), drive_wheel_joint_->Position(0)};
      joint_state_msg.velocity = {0.0, wheel_velocity};
      joint_state_pub_->publish(joint_state_msg);

      // Publish odometry
      RCLCPP_INFO(node_->get_logger(), "bbbbbbbb5");
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
      RCLCPP_INFO(node_->get_logger(), "bbbbbbbb6");
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

    // rclcpp::Node::SharedPtr node_;
    // physics::ModelPtr model_;
    // physics::JointPtr drive_wheel_joint_;
    // physics::JointPtr steering_joint_;
    // event::ConnectionPtr update_connection_;

    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double lin_vel_ = 0.0;
    double ang_vel_ = 0.0;
    double wheel_radius_ = 0.133;
    double wheel_base_ = 0.5;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    common::Time last_time_;
  };

  GZ_REGISTER_MODEL_PLUGIN(AgvDrivePlugin)
} // namespace gazebo
