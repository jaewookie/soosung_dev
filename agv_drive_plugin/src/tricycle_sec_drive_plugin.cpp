#include <math.h>
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

#include "agv_drive_plugin/tricycle_sec_drive_plugin.hpp"

#include <memory>

namespace gazebo_ros
{

  class TricycleSecDrivePluginPrivate
  {
  public:
    enum
    {
      STEERING,

      DRIVE_WHEEL,

      FRONT_WHEEL_LEFT,

      FRONT_WHEEL_RIGHT
    };

    void OnUpdate(const gazebo::common::UpdateInfo &_info);
    void OnCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg); // ok
    void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    // void PublishWheelsTf(const gazebo::common::Time &_current_time);
    void PublishWheelJointState(const gazebo::common::Time &_current_time);
    void MotorController(double target_speed, double target_angle, double dt); // ok

    // ROS node for communication, managed by gazebo_ros.
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    gazebo::event::ConnectionPtr update_connection_; // Connection to world update event. Callback is called while this is alive.

    double drive_wheel_radius_ = 0.16;
    double front_wheel_radius_ = 0.125;
    double max_wheel_accel_;
    double max_wheel_decel_;
    double max_wheel_speed_tol_;
    double max_wheel_torque_;
    double max_steering_angle_tol_;
    double max_steering_speed_;
    double wheel_separation_;

    geometry_msgs::msg::Twist cmd_;
    sensor_msgs::msg::Imu imu_;
    sensor_msgs::msg::JointState joint_state_;
    std::vector<gazebo::physics::JointPtr> joints_;

    gazebo::physics::ModelPtr model_;

    // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex lock_;

    double update_period_;
    geometry_msgs::msg::Pose2D pose_encoder_;

    gazebo::common::Time last_actuator_update_;

    std::string robot_base_frame_;

    // bool publish_wheel_tf_;
    bool publish_wheel_joint_state_;
  };

  TricycleSecDrivePlugin::TricycleSecDrivePlugin()
      : impl_(std::make_unique<TricycleSecDrivePluginPrivate>())
  {
  }

  TricycleSecDrivePlugin::~TricycleSecDrivePlugin()
  {
  }

  void TricycleSecDrivePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    impl_->model_ = _model;

    // Create ROS2 node:
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Operate TricycleSecDrivePlugin");

    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();

    impl_->robot_base_frame_ = "base_footprint";

    // impl_->publish_wheel_tf_ = true;

    impl_->publish_wheel_joint_state_ = true;

    // 가제보 플러그인 태그
    impl_->wheel_separation_ = _sdf->Get<double>("wheel_separation_", 0.9).first;
    impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0).first;
    impl_->max_wheel_decel_ = _sdf->Get<double>(
                                      "max_wheel_deceleration", impl_->max_wheel_accel_)
                                  .first;
    impl_->max_wheel_speed_tol_ = _sdf->Get<double>("max_wheel_speed_tolerance", 0.01).first;
    impl_->max_steering_speed_ = _sdf->Get<double>("max_steering_speed_", 0).first;
    impl_->max_steering_angle_tol_ = _sdf->Get<double>("max_steering_angle_tol_", 0.05).first;

    impl_->joints_.resize(4); // 조향 추가시 4로 변경

    // OBTAIN -> JOINTS:
    impl_->joints_[TricycleSecDrivePluginPrivate::STEERING] = _model->GetJoint("steering_joint");
    impl_->joints_[TricycleSecDrivePluginPrivate::DRIVE_WHEEL] = _model->GetJoint("drive_wheel_joint");
    impl_->joints_[TricycleSecDrivePluginPrivate::FRONT_WHEEL_LEFT] = _model->GetJoint("front_wheel_l_joint");
    impl_->joints_[TricycleSecDrivePluginPrivate::FRONT_WHEEL_RIGHT] = _model->GetJoint("front_wheel_r_joint");

    impl_->max_wheel_torque_ = 104;
    impl_->joints_[TricycleSecDrivePluginPrivate::DRIVE_WHEEL]->SetParam("fmax", 0, impl_->max_wheel_torque_);
    impl_->joints_[TricycleSecDrivePluginPrivate::STEERING]->SetParam("fmax", 0, impl_->max_wheel_torque_);

    auto publish_rate = 100;

    impl_->update_period_ = 1 / publish_rate;

    impl_->last_actuator_update_ = _model->GetWorld()->SimTime();

    // Create status publisher
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)), std::bind(&TricycleSecDrivePluginPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

    impl_->imu_sub_ = impl_->ros_node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu", qos.get_subscription_qos("imu", rclcpp::QoS(1)), std::bind(&TricycleSecDrivePluginPrivate::OnImu, impl_.get(), std::placeholders::_1));

    impl_->joint_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", qos.get_publisher_qos("joint_states", rclcpp::QoS(1000)));

    // impl_->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    //     impl_->ros_node_);

    // Initialize joint state message
    impl_->joint_state_.name.resize(impl_->joints_.size());
    impl_->joint_state_.position.resize(impl_->joints_.size());
    impl_->joint_state_.velocity.resize(impl_->joints_.size());
    impl_->joint_state_.effort.resize(impl_->joints_.size());
    for (std::size_t i = 0; i < impl_->joints_.size(); i++)
    {
      impl_->joint_state_.name[i] = impl_->joints_[i]->GetName();
    }

    // Create a connection so the OnUpdate function is called at every simulation iteration.

    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&TricycleSecDrivePluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "GAZEBO Driving plugin loaded successfully.");
  }

  void TricycleSecDrivePlugin::Reset()
  {
    std::lock_guard<std::mutex> scoped_lock(impl_->lock_);

    if (impl_->joints_[TricycleSecDrivePluginPrivate::DRIVE_WHEEL] && impl_->joints_[TricycleSecDrivePluginPrivate::STEERING])
    {
      gazebo::common::Time current_time =
          impl_->joints_[TricycleSecDrivePluginPrivate::DRIVE_WHEEL]->GetWorld()->SimTime();
      impl_->joints_[TricycleSecDrivePluginPrivate::DRIVE_WHEEL]->SetParam(
          "fmax", 0, impl_->max_wheel_torque_);
      impl_->joints_[TricycleSecDrivePluginPrivate::STEERING]->SetParam(
          "fmax", 0, impl_->max_wheel_torque_);
      impl_->joints_[TricycleSecDrivePluginPrivate::DRIVE_WHEEL]->SetParam("vel", 0, 0.0);
      impl_->joints_[TricycleSecDrivePluginPrivate::STEERING]->SetParam("vel", 0, 0.0);

      impl_->last_actuator_update_ = current_time;
    }
    impl_->pose_encoder_.x = 0;
    impl_->pose_encoder_.y = 0;
    impl_->pose_encoder_.theta = 0;
    impl_->cmd_.linear.x = 0;
    impl_->cmd_.angular.z = 0;
  }

  // Link간 연결을 보여주는 Joint_state
  void TricycleSecDrivePluginPrivate::PublishWheelJointState(
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

  // // joint와 Link 연결 TF
  // void TricycleSecDrivePluginPrivate::PublishWheelsTf(const gazebo::common::Time &_current_time)
  // {
  //   rclcpp::Time current_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  //   for (auto &joint : joints_)
  //   {
  //     // sdf파일에서 가지고오는 정보들
  //     std::string frame = joint->GetName();
  //     std::string parent_frame = joint->GetParent()->GetName();

  //     ignition::math::Pose3d pose = joint->GetChild()->RelativePose();

  //     geometry_msgs::msg::TransformStamped msg;
  //     msg.header.stamp = current_time;
  //     msg.header.frame_id = parent_frame;
  //     msg.child_frame_id = frame;
  //     msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
  //     msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());
  //     tf_broadcaster_->sendTransform(msg);
  //   }
  // }

  void TricycleSecDrivePluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
  {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("TricycleSecDrivePluginPrivate::OnUpdate");
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

    //     if (publish_wheel_tf_)
    //     {
    // #ifdef IGN_PROFILER_ENABLE
    //       IGN_PROFILE_BEGIN("PublishWheelsTf");
    // #endif
    //       PublishWheelsTf(current_time);
    // #ifdef IGN_PROFILER_ENABLE
    //       IGN_PROFILE_END();
    // #endif
    //     }
    if (publish_wheel_joint_state_)
    {
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("PublishWheelJointState");
#endif
      PublishWheelJointState(current_time);
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
    }
    std::unique_lock<std::mutex> lock(lock_);
    // RCLCPP_INFO(ros_node_->get_logger(), "a: [%f]", cmd_.linear.x);
    double target_wheel_rotation_speed = cmd_.linear.x / drive_wheel_radius_;
    double target_steering_angle = cmd_.angular.z;
    lock.unlock();

#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("MotorController");
#endif
    MotorController(
        target_wheel_rotation_speed, target_steering_angle, seconds_since_last_update);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    //  RCLCPP_INFO(ros_node_->get_logger(),
    //  "v = %f, w = %f ", target_wheel_rotation_speed, target_steering_angle);

    last_actuator_update_ = _info.simTime;
  }

  void TricycleSecDrivePluginPrivate::MotorController(
      double target_speed, double target_angle, double dt)
  {
    double applied_speed = target_speed;
    double applied_angle = target_angle;

    double current_speed = joints_[DRIVE_WHEEL]->GetVelocity(0);

    if (max_wheel_accel_ > 0 || max_wheel_decel_ > 0)
    {
      double diff_speed = current_speed - target_speed;
      if (fabs(diff_speed) < max_wheel_speed_tol_)
      {
        applied_speed = current_speed;
      }
      else if (-diff_speed > max_wheel_accel_ * dt)
      {
        applied_speed = current_speed + max_wheel_accel_ * dt;
      }
      else if (diff_speed > max_wheel_decel_ * dt)
      {
        applied_speed = current_speed - max_wheel_decel_ * dt;
      }
    }

    // SetParam으로 주행 바퀴의 현재 속도 입력
    // (실제에서는 모터 드라이브 혹은 컨트롤러와 통신하는 것과 같음)
    joints_[DRIVE_WHEEL]->SetParam("vel", 0, applied_speed);

    double current_angle = joints_[STEERING]->Position(0);

    double diff_angle = current_angle - target_angle;
    double applied_steering_speed = 0;

    if (max_steering_speed_ > 0)
    {
      // this means we will steer using steering speed
      if (fabs(diff_angle) < max_steering_angle_tol_)
      {
        // we're withing angle tolerance
        applied_steering_speed = 0;
      }
      else if (diff_angle != target_speed)
      {
        // steer toward target angle
        if (diff_angle > 0)
        {
          applied_steering_speed = -max_steering_speed_;
        }
        else if (diff_angle < 0)
        {
          applied_steering_speed = max_steering_speed_;
        }
        applied_angle = current_angle + applied_steering_speed * dt;
      }
      else
      {
        // steer toward target angle
        applied_steering_speed = -max_steering_speed_;
      }

      joints_[STEERING]->SetPosition(0, applied_angle, true);
    }
  }
  void TricycleSecDrivePluginPrivate::OnCmdVel(
      const geometry_msgs::msg::Twist::ConstSharedPtr cmd_msg)
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    cmd_.linear.x = cmd_msg->linear.x;
    cmd_.angular.z = cmd_msg->angular.z;
    if (fabs(cmd_.linear.x) < 0.1 && fabs(cmd_.angular.z) > max_steering_angle_tol_)
    {
      if(cmd_.linear.x<0){
        cmd_.linear.x = -0.5;
      }else{
        cmd_.linear.x = 0.5;
      }
      cmd_.angular.z = atan(1.5 / (cmd_.linear.x / cmd_.angular.z));
    }
  }

  void TricycleSecDrivePluginPrivate::OnImu(
      const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    imu_.orientation.x = imu_msg->orientation.x;
    imu_.orientation.y = imu_msg->orientation.y;
    imu_.orientation.z = imu_msg->orientation.z;
    imu_.orientation.w = imu_msg->orientation.w;
  }
  GZ_REGISTER_MODEL_PLUGIN(TricycleSecDrivePlugin)
} // namespace gazebo_ros
