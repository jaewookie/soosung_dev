#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <agv_msgs/msg/fork_control.hpp>
// #include <agv_msgs/msg/AgvMsg.hpp>
#include "agv_drive_plugin/fork_control_plugin.hpp"

#include <memory>

namespace gazebo_ros
{

  class ForkControlPluginPrivate
  {
  public:

    void OnUpdate(const gazebo::common::UpdateInfo &_info);
    // void OnCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg); // ok
    void OnMastCon(const agv_msgs::msg::ForkControl::ConstSharedPtr msg);
    // void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    // void UpdateOdometryEncoder(const gazebo::common::Time &_current_time); // ok
    // void PublishOdometryMsg(const gazebo::common::Time &_current_time);    // ok
    void PublishMastsTf(const gazebo::common::Time &_current_time);
    void PublishmastJointState(const gazebo::common::Time &_current_time);
    void MotorController(double target_speed, double dt); // ok

    // ROS node for communication, managed by gazebo_ros.
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<agv_msgs::msg::ForkControl>::SharedPtr mast_vel_sub_;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    gazebo::event::ConnectionPtr update_connection_; // Connection to world update event. Callback is called while this is alive.

    // double drive_mast_radius_ = 0.16;
    // double front_mast_radius_ = 0.125;
    double max_mast_accel_;
    double max_mast_decel_;
    double max_mast_speed_tol_;
    double max_mast_torque_;
    // double max_steering_angle_tol_;
    // double max_steering_speed_;
    // double mast_separation_;

    geometry_msgs::msg::Twist cmd_;
    double mast_vel;
    // sensor_msgs::msg::Imu imu_;
    sensor_msgs::msg::JointState joint_state_;
    std::vector<gazebo::physics::JointPtr> joints_;

    gazebo::physics::ModelPtr model_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex lock_;

    double update_period_;
    // geometry_msgs::msg::Pose2D pose_encoder_;
    // std::string odometry_frame_;

    // gazebo::common::Time last_odom_update_;
    gazebo::common::Time last_actuator_update_;

    // OdomSource odom_source_;

    // nav_msgs::msg::Odometry odom_;

    std::string robot_base_frame_;

    bool publish_mast_tf_;
    bool publish_mast_joint_state_;
    bool publish_odom_;
  };

  ForkControlPlugin::ForkControlPlugin()
      : impl_(std::make_unique<ForkControlPluginPrivate>())
  {
  }

  ForkControlPlugin::~ForkControlPlugin()
  {
  }

  void ForkControlPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    impl_->model_ = _model;

    // Create ROS2 node:
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Operate ForkControlPlugin");

    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();

    // impl_->odometry_frame_ = "odom";

    impl_->robot_base_frame_ = "base_footprint";

    // impl_->odom_source_ = ForkControlPluginPrivate::ENCODER;

    impl_->publish_mast_tf_ = true;

    impl_->publish_mast_joint_state_ = true;

    // impl_->publish_odom_ = true;

    // 가제보 플러그인 태그
    // impl_->mast_separation_ = _sdf->Get<double>("mast_separation_", 0.9).first;
    impl_->max_mast_accel_ = _sdf->Get<double>("max_mast_acceleration", 0).first;
    impl_->max_mast_decel_ = _sdf->Get<double>(
                                     "max_mast_deceleration", impl_->max_mast_accel_)
                                 .first;
    impl_->max_mast_speed_tol_ = _sdf->Get<double>("max_mast_speed_tolerance", 0.01).first;
    // impl_->max_steering_speed_ = _sdf->Get<double>("max_steering_speed_", 0).first;
    // impl_->max_steering_angle_tol_ = _sdf->Get<double>("max_steering_angle_tol_", 0.05).first;

    impl_->joints_.resize(1); // 조향 추가시 4로 변경

    // OBTAIN -> JOINTS:
    impl_->joints_[0] = _model->GetJoint("fork_joint");
    // impl_->joints_[0] = _model->GetJoint("drive_mast_joint");
    // impl_->joints_[ForkControlPluginPrivate::FRONT_mast_LEFT] = _model->GetJoint("front_mast_l_joint");
    // impl_->joints_[ForkControlPluginPrivate::FRONT_mast_RIGHT] = _model->GetJoint("front_mast_r_joint");

    impl_->max_mast_torque_ = 104;
    impl_->joints_[0]->SetParam("fmax", 0, impl_->max_mast_torque_);
    // impl_->joints_[0]->SetParam("fmax", 0, impl_->max_mast_torque_);

    auto publish_rate = 100;

    impl_->update_period_ = 1 / publish_rate;

    impl_->last_actuator_update_ = _model->GetWorld()->SimTime();

    // Create status publisher
    impl_->mast_vel_sub_ = impl_->ros_node_->create_subscription<agv_msgs::msg::ForkControl>(
        "mast_vel", qos.get_subscription_qos("mast_vel", rclcpp::QoS(1)), std::bind(&ForkControlPluginPrivate::OnMastCon, impl_.get(), std::placeholders::_1));

    // impl_->imu_sub_ = impl_->ros_node_->create_subscription<sensor_msgs::msg::Imu>(
    //     "imu", qos.get_subscription_qos("imu", rclcpp::QoS(1)), std::bind(&ForkControlPluginPrivate::OnImu, impl_.get(), std::placeholders::_1));

    impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", qos.get_publisher_qos("joint_states", rclcpp::QoS(1000)));

    // impl_->odom_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
    //     "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

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

    // // Initialize odom message
    // impl_->odom_.header.frame_id = impl_->odometry_frame_;
    // impl_->odom_.child_frame_id = impl_->robot_base_frame_;
    // impl_->odom_.pose.covariance[0] = 0.00001;
    // impl_->odom_.pose.covariance[7] = 0.00001;
    // impl_->odom_.pose.covariance[14] = 1000000000000.0;
    // impl_->odom_.pose.covariance[21] = 1000000000000.0;
    // impl_->odom_.pose.covariance[28] = 1000000000000.0;
    // impl_->odom_.pose.covariance[35] = 0.001;

    // Create a connection so the OnUpdate function is called at every simulation iteration.

    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&ForkControlPluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "GAZEBO Fork House plugin loaded successfully.");
  }

  void ForkControlPlugin::Reset()
  {
    std::lock_guard<std::mutex> scoped_lock(impl_->lock_);

    if (impl_->joints_[0] && impl_->joints_[0])
    {
      gazebo::common::Time current_time = impl_->joints_[0]->GetWorld()->SimTime();
      impl_->joints_[0]->SetParam("fmax", 0, impl_->max_mast_torque_);
      // impl_->joints_[0]->SetParam("fmax", 0, impl_->max_mast_torque_);
      impl_->joints_[0]->SetParam("vel", 0, 0.0);
      // impl_->joints_[0]->SetParam("vel", 0, 0.0);

      impl_->last_actuator_update_ = current_time;
      // impl_->last_odom_update_ = current_time;
    }
    // impl_->pose_encoder_.x = 0;
    // impl_->pose_encoder_.y = 0;
    // impl_->pose_encoder_.theta = 0;
    // impl_->cmd_.linear.x = 0;
    // impl_->cmd_.angular.z = 0;
  }

  // Link간 연결을 보여주는 Joint_state
  void ForkControlPluginPrivate::PublishmastJointState(
      const gazebo::common::Time &_current_time)
  {
    joint_state_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

    for (std::size_t i = 0; i < joints_.size(); i++)
    {
      // 가제보상에서 데이터를 가지고와 joint_state에 입력
      // 가상의 센서에서 데이터를 가지고 오는 것과 같음
      joint_state_.position[i] = joints_[i]->Position(0);
      joint_state_.velocity[i] = joints_[i]->GetVelocity(0);
      joint_state_.effort[i] = joints_[i]->GetForce(0);
    }
    joint_state_pub_->publish(joint_state_);
  }

  // joint와 Link 연결 TF
  void ForkControlPluginPrivate::PublishMastsTf(const gazebo::common::Time &_current_time)
  {
    rclcpp::Time current_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

    for (auto &joint : joints_)
    {
      // sdf파일에서 가지고오는 정보들
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
  }

  void ForkControlPluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
  {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("ForkControlPluginPrivate::OnUpdate");
    IGN_PROFILE_BEGIN("UpdateOdometryEncoder");
#endif
    gazebo::common::Time current_time = _info.simTime;
    // if (odom_source_ == ENCODER)
    // {
    //   UpdateOdometryEncoder(current_time);
    // }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    double seconds_since_last_update = (current_time - last_actuator_update_).Double();

    if (seconds_since_last_update < update_period_)
    {
      return;
    }

    //     if (publish_odom_)
    //     {
    // #ifdef IGN_PROFILER_ENABLE
    //       IGN_PROFILE_BEGIN("PublishOdometryMsg");
    // #endif
    //       PublishOdometryMsg(current_time);
    // #ifdef IGN_PROFILER_ENABLE
    //       IGN_PROFILE_END();
    // #endif
    //     }

    if (publish_mast_tf_)
    {
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("PublishMastsTf");
#endif
      PublishMastsTf(current_time);
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
    }

    if (publish_mast_joint_state_)
    {
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("PublishmastJointState");
#endif
      PublishmastJointState(current_time);
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
    }

    std::unique_lock<std::mutex> lock(lock_);
    double target_mast_speed = mast_vel;
    // double target_steering_angle = cmd_.angular.z;
    lock.unlock();

#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("MotorController");
#endif
    MotorController(
        target_mast_speed, seconds_since_last_update);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    //  RCLCPP_INFO(ros_node_->get_logger(),
    //  "v = %f, w = %f ", target_mast_rotation_speed, target_steering_angle);

    last_actuator_update_ = _info.simTime;
  }

  void ForkControlPluginPrivate::MotorController(
      double target_speed, double dt)
  {
    double applied_speed = target_speed;

    double current_speed = joints_[0]->GetVelocity(0);

    if (max_mast_accel_ > 0 || max_mast_decel_ > 0)
    {
      double diff_speed = current_speed - target_speed;
      if (fabs(diff_speed) < max_mast_speed_tol_)
      {
        applied_speed = current_speed;
      }
      else if (-diff_speed > max_mast_accel_ * dt)
      {
        applied_speed = current_speed + max_mast_accel_ * dt;
      }
      else if (diff_speed > max_mast_decel_ * dt)
      {
        applied_speed = current_speed - max_mast_decel_ * dt;
      }
    }

    joints_[0]->SetParam("vel", 0, applied_speed);
  }

  void ForkControlPluginPrivate::OnMastCon(const agv_msgs::msg::ForkControl::ConstSharedPtr agv_msg)
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    mast_vel = agv_msg->fork_vel;
    RCLCPP_INFO(ros_node_->get_logger(), "get data : [%f]", mast_vel);
  }

  GZ_REGISTER_MODEL_PLUGIN(ForkControlPlugin)
} // namespace gazebo_ros
