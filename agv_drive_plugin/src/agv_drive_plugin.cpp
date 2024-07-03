#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
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
    enum
    {
      DRIVE_WHEEL,

      FRONT_WHEEL_LEFT,

      FRONT_WHEEL_RIGHT
    };

    // ROS node for communication, managed by gazebo_ros.
    gazebo_ros::Node::SharedPtr ros_node_;
    sensor_msgs::msg::JointState joint_state_;
    nav_msgs::msg::Odometry odom_;

    // The joint that controls the movement of the belt:
    std::vector<gazebo::physics::JointPtr> joints_;

    gazebo::physics::JointPtr drive_wheel_joint_;
    // gazebo::physics::JointPtr steering_joint_; // 추후 이용
    gazebo::physics::JointPtr front_wheel_l_joint_;
    gazebo::physics::JointPtr front_wheel_r_joint_;

    std::string odometry_frame_;
    std::string robot_base_frame_;

    // Additional parametres:
    double lin_vel_ = 0.0;
    double ang_vel_ = 0.0;
    double drive_wheel_radius_ = 0.31;
    double front_wheel_radius_ = 0.25;
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
    gazebo::common::Time last_publish_time_;
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

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Operate AgvDrivePlugin");

    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();

    impl_->joints_.resize(3); // 조향 추가시 4로 변경

    // OBTAIN -> JOINTS:
    impl_->joints_[AgvDrivePluginPrivate::DRIVE_WHEEL] = _model->GetJoint("drive_wheel_joint");
    impl_->joints_[AgvDrivePluginPrivate::FRONT_WHEEL_LEFT] = _model->GetJoint("front_wheel_l_joint");
    impl_->joints_[AgvDrivePluginPrivate::FRONT_WHEEL_RIGHT] = _model->GetJoint("front_wheel_r_joint");
    // impl_->joints_[3] = _model->GetJoint("steering_joint");

    if (!impl_->joints_[AgvDrivePluginPrivate::DRIVE_WHEEL] || (!impl_->joints_[AgvDrivePluginPrivate::FRONT_WHEEL_LEFT] || !impl_->joints_[AgvDrivePluginPrivate::FRONT_WHEEL_RIGHT]))
    {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "joint not found, unable to start conveyor plugin.");
      return;
    }

    auto publish_rate = _sdf->Get<double>("update_rate", 100).first;

    impl_->update_ns_ = 1 / publish_rate;

    impl_->last_publish_time_ = _model->GetWorld()->SimTime();

    // Create status publisher
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)), std::bind(&AgvDrivePluginPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

    impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", qos.get_publisher_qos("joint_states", rclcpp::QoS(1000)));

    impl_->odom_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    impl_->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
        impl_->ros_node_);


    // Initialize joint state message
    impl_->joint_state_.name.resize(impl_->joints_.size());
    impl_->joint_state_.position.resize(impl_->joints_.size());
    impl_->joint_state_.velocity.resize(impl_->joints_.size());
    impl_->joint_state_.effort.resize(impl_->joints_.size());
    for (std::size_t i = 0; i < impl_->joints_.size(); i++)
    {
      impl_->joint_state_.name[i] = impl_->joints_[i]->GetName();
    }

    // Initialize odom message
    impl_->odom_.header.frame_id = impl_->odometry_frame_;
    impl_->odom_.child_frame_id = impl_->robot_base_frame_;
    impl_->odom_.pose.covariance[0] = 0.00001;
    impl_->odom_.pose.covariance[7] = 0.00001;
    impl_->odom_.pose.covariance[14] = 1000000000000.0;
    impl_->odom_.pose.covariance[21] = 1000000000000.0;
    impl_->odom_.pose.covariance[28] = 1000000000000.0;
    impl_->odom_.pose.covariance[35] = 0.001;

    // Create a connection so the OnUpdate function is called at every simulation iteration.

    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&AgvDrivePluginPrivate::OnUpdate, impl_.get()));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "GAZEBO Driving plugin loaded successfully.");
  }

  void AgvDrivePluginPrivate::OnCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    lin_vel_ = msg->linear.x;
    ang_vel_ = msg->angular.z;
  }

  void AgvDrivePluginPrivate::OnUpdate()
  {
    rclcpp::Time current_time = ros_node_->get_clock()->now();
    double dt = (current_time.seconds() - last_time_.seconds());
    last_time_ = current_time;

    // Calculate the wheel velocity and steering angle
    double drvie_wheel_velocity = lin_vel_ / drive_wheel_radius_;
    double front_wheel_velocity = lin_vel_ / front_wheel_radius_;
    double steering_angle = atan(wheel_base_ * ang_vel_ / lin_vel_);

    // Set joint positions
    static double drvie_wheel_position = 0.0;
    static double front_wheel_position = 0.0;

    drvie_wheel_position += drvie_wheel_velocity * dt;
    front_wheel_position += front_wheel_velocity * dt;

    // drive_wheel_joint_->SetVelocity(0, drvie_wheel_velocity);
    // steering_joint_->SetPosition(0, steering_angle);

    // Publish joint states
    joint_state_.header.stamp = ros_node_->get_clock()->now();
    for (std::size_t i = 0; i < joints_.size(); i++)
    {
      if (i == 0)
      {
        joint_state_.position[i] = drvie_wheel_position;
        joint_state_.velocity[i] = drvie_wheel_velocity;
        joint_state_.effort[i] = 0.0;
      }
      else
      {
        joint_state_.position[i] = front_wheel_position;
        joint_state_.velocity[i] = front_wheel_velocity;
        joint_state_.effort[i] = 0.0;
      }
    }
    joint_state_pub_->publish(joint_state_);

    // Publish odometry
    // double delta_x = lin_vel_ * cos(yaw_) * dt;
    // double delta_y = lin_vel_ * sin(yaw_) * dt;
    // double delta_yaw = ang_vel_ * dt;

    double delta_x = 0;
    double delta_y = lin_vel_ * dt;
    double delta_yaw = ang_vel_ * dt;

    // x_ += delta_x;
    y_ += delta_y;
    yaw_ += delta_yaw;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = ros_node_->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    tf2::convert(q, odom_.pose.pose.orientation);
    odom_msg.twist.twist.linear.x = lin_vel_;
    odom_msg.twist.twist.angular.z = ang_vel_;
    odom_pub_->publish(odom_msg);

    // Publish transform
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = ros_node_->get_clock()->now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = 0.0;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    tf2::convert(q, odom_.pose.pose.orientation);
    tf_broadcaster_->sendTransform(odom_tf);

    for (auto &joint : joints_)
    {
      std::string frame = joint->GetName();
      std::string parent_frame = joint->GetParent()->GetName();

      ignition::math::Pose3d pose = joint->GetChild()->RelativePose();

      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = current_time;
      msg.header.frame_id = parent_frame;
      msg.child_frame_id = frame;
      msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
      msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());
      tf_broadcaster_->sendTransform(msg);
    }
    // for (std::size_t i = 0; i < joints_.size(); i++)
    // {
    //   if (i == 0)
    //   {
    //     std::string frame = "drive_wheel_join";
    //     std::string parent_frame = "steering_part";

    //     geometry_msgs::msg::TransformStamped msg;
    //     msg.header.stamp = ros_node_->get_clock()->now();
    //     msg.header.frame_id = parent_frame;
    //     msg.child_frame_id = frame;
    //     msg.transform.translation.x = 0.0;
    //     msg.transform.translation.y = y_;
    //     msg.transform.translation.z = 0.0;
    //     tf2::convert(q, odom_.pose.pose.orientation);
    //     tf_broadcaster_->sendTransform(msg);
    //   }
    //   else if (i == 1)
    //   {
    //     std::string frame = "front_wheel_l_joint";
    //     std::string parent_frame = "base_link";

    //     geometry_msgs::msg::TransformStamped msg;
    //     msg.header.stamp = ros_node_->get_clock()->now();
    //     msg.header.frame_id = parent_frame;
    //     msg.child_frame_id = frame;
    //     msg.transform.translation.x = 0.0;
    //     msg.transform.translation.y = y_;
    //     msg.transform.translation.z = 0.0;
    //     tf2::convert(q, odom_.pose.pose.orientation);
    //     tf_broadcaster_->sendTransform(msg);
    //   }
    //   else
    //   {
    //     std::string frame = "front_wheel_r_joint";
    //     std::string parent_frame = "base_link";

    //     geometry_msgs::msg::TransformStamped msg;
    //     msg.header.stamp = ros_node_->get_clock()->now();
    //     msg.header.frame_id = parent_frame;
    //     msg.child_frame_id = frame;
    //     msg.transform.translation.x = 0.0;
    //     msg.transform.translation.y = y_;
    //     msg.transform.translation.z = 0.0;
    //     tf2::convert(q, odom_.pose.pose.orientation);
    //     tf_broadcaster_->sendTransform(msg);
    //   }
    //   // ignition::math::Pose3d pose = joint->GetChild()->RelativePose();
    // }

    // for (auto &joint : joints_)
    // {
    //   std::string frame = joint->GetName();
    //   std::string parent_frame = joint->GetParent()->GetName();

    //   ignition::math::Pose3d pose = joint->GetChild()->RelativePose();

    //   geometry_msgs::msg::TransformStamped msg;
    //   msg.header.stamp = ros_node_->get_clock()->now();
    //   msg.header.frame_id = parent_frame;
    //   msg.child_frame_id = frame;
    //   msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
    //   msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());
    //   transform_broadcaster_->sendTransform(msg);
    // }
  }

  GZ_REGISTER_MODEL_PLUGIN(AgvDrivePlugin)
} // namespace gazebo_ros
