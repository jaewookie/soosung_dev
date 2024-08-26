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
    void OnMastCon(const agv_msgs::msg::ForkControl::ConstSharedPtr msg);
    void PublishMastsTf(const gazebo::common::Time &_current_time);
    void PublishmastJointState(const gazebo::common::Time &_current_time);
    void MotorVelController(double target_speed, double dt); // ok

    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<agv_msgs::msg::ForkControl>::SharedPtr mast_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    gazebo::event::ConnectionPtr update_connection_; // Connection to world update

    double max_mast_accel_;
    double max_mast_decel_;
    double max_mast_speed_tol_;
    double max_mast_torque_;
    double max_mast_speed_;

    double mast_vel = 0.0;
    sensor_msgs::msg::JointState joint_state_;
    std::vector<gazebo::physics::JointPtr> joints_;

    gazebo::physics::ModelPtr model_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex lock_;

    double update_period_;

    gazebo::common::Time last_actuator_update_;

    std::string robot_base_frame_;

    bool publish_mast_tf_;
    bool publish_mast_joint_state_;
    bool publish_odom_;
    bool motor_pose_con_;
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

    impl_->robot_base_frame_ = "base_footprint";

    impl_->publish_mast_tf_ = true;

    impl_->publish_mast_joint_state_ = true;

    // 가제보 플러그인 태그
    impl_->max_mast_accel_ = _sdf->Get<double>("max_mast_acceleration", 0).first;
    impl_->max_mast_decel_ = _sdf->Get<double>(
                                     "max_mast_deceleration", impl_->max_mast_accel_)
                                 .first;
    impl_->max_mast_speed_tol_ = _sdf->Get<double>("max_mast_speed_tolerance", 0.01).first;
    impl_->max_mast_speed_ = _sdf->Get<double>("max_mast_speed", 0).first;
    impl_->motor_pose_con_ = _sdf->Get<bool>("motor_pose_con", false).first;

    impl_->joints_.resize(1);

    // OBTAIN -> JOINTS:
    impl_->joints_[0] = _model->GetJoint("fork_joint");

    impl_->max_mast_torque_ = 300;
    impl_->joints_[0]->SetParam("fmax", 0, impl_->max_mast_torque_);

    auto publish_rate = 100;

    impl_->update_period_ = 1 / publish_rate;

    impl_->last_actuator_update_ = _model->GetWorld()->SimTime();

    // Create status publisher
    impl_->mast_vel_sub_ = impl_->ros_node_->create_subscription<agv_msgs::msg::ForkControl>(
        "mast_vel", qos.get_subscription_qos("mast_vel", rclcpp::QoS(1)), std::bind(&ForkControlPluginPrivate::OnMastCon, impl_.get(), std::placeholders::_1));

    impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", qos.get_publisher_qos("joint_states", rclcpp::QoS(1000)));

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
      impl_->joints_[0]->SetParam("vel", 0, 0.0);

      impl_->last_actuator_update_ = current_time;
    }
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
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    double seconds_since_last_update = (current_time - last_actuator_update_).Double();

    if (seconds_since_last_update < update_period_)
    {
      return;
    }
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
    lock.unlock();
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("MotorVelController");
#endif
    MotorVelController(target_mast_speed, seconds_since_last_update);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    last_actuator_update_ = _info.simTime;
  }

  void ForkControlPluginPrivate::MotorVelController(
      double target_speed, double dt)
  {
    // 중력 보정
    double gravity_torque = joints_[0]->GetChild()->GetInertial()->Mass() * 9.81;

    joints_[0]->SetForce(0, gravity_torque);

    // 위치 변환 계산
    static double current_pose = joints_[0]->Position(0);

    double applied_speed = target_speed;
    double applied_pose = current_pose;

    applied_pose += (applied_speed * dt);

    if (applied_pose > 1.5)
    {
      applied_pose = 1.5;
    }
    else if (applied_pose < -0.11)
    {
      applied_pose = -0.11;
    }
    else if (applied_speed == 0)
    {
      applied_pose = current_pose;
    }

    // 변환 위치 및 속도 적용
    joints_[0]->SetPosition(0, applied_pose, true);
    joints_[0]->SetParam("vel", 0, applied_speed);

    current_pose = applied_pose;

    // RCLCPP_INFO(ros_node_->get_logger(), "[%f]", joints_[0]->Position(0));
  }

  void ForkControlPluginPrivate::OnMastCon(const agv_msgs::msg::ForkControl::ConstSharedPtr agv_msg)
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    mast_vel = agv_msg->fork_vel;
  }

  GZ_REGISTER_MODEL_PLUGIN(ForkControlPlugin)
} // namespace gazebo_ros
