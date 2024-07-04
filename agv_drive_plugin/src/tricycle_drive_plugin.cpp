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

#include "agv_drive_plugin/tricycle_drive_plugin.hpp"

#include <memory>

namespace gazebo_ros
{

  class TricycleDrivePluginPrivate
  {
  public:
    enum OdomSource
    {
      /// Use an ancoder
      ENCODER,

      /// Use ground truth from simulation world
      WORLD,
    };

    enum
    {
      DRIVE_WHEEL,

      FRONT_WHEEL_LEFT,

      FRONT_WHEEL_RIGHT
    };

    void OnUpdate(const gazebo::common::UpdateInfo &_info);
    void OnCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void UpdateOdometryEncoder(const gazebo::common::Time &_current_time);
    void PublishOdometryMsg(const gazebo::common::Time &_current_time);
    void PublishWheelsTf(const gazebo::common::Time &_current_time);
    void PublishWheelJointState(const gazebo::common::Time &_current_time);
    void MotorController(double target_speed, double dt);

    // ROS node for communication, managed by gazebo_ros.
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    gazebo::event::ConnectionPtr update_connection_; // Connection to world update event. Callback is called while this is alive.

    double drive_wheel_radius_ = 0.31;
    double front_wheel_radius_ = 0.25;
    double max_wheel_accel_;
    double max_wheel_decel_;
    double max_wheel_speed_tol_;
    double max_wheel_torque_;
    double wheel_separation_;

    geometry_msgs::msg::Twist cmd_;
    sensor_msgs::msg::JointState joint_state_;
    std::vector<gazebo::physics::JointPtr> joints_;

    gazebo::physics::ModelPtr model_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex lock_;

    double update_period_;
    geometry_msgs::msg::Pose2D pose_encoder_;
    std::string odometry_frame_;

    gazebo::common::Time last_odom_update_;
    gazebo::common::Time last_actuator_update_;

    OdomSource odom_source_;

    nav_msgs::msg::Odometry odom_;

    std::string robot_base_frame_;

    bool publish_wheel_tf_;
    bool publish_wheel_joint_state_;
    bool publish_odom_;
  };

  TricycleDrivePlugin::TricycleDrivePlugin()
      : impl_(std::make_unique<TricycleDrivePluginPrivate>())
  {
  }

  TricycleDrivePlugin::~TricycleDrivePlugin()
  {
  }

  void TricycleDrivePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {

    impl_->model_ = _model;

    // Create ROS2 node:
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);


    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Operate TricycleDrivePlugin");

    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();

    impl_->odometry_frame_ = "odom";

    impl_->robot_base_frame_ = "base_footprint";

    impl_->odom_source_ = TricycleDrivePluginPrivate::ENCODER;

    impl_->publish_wheel_tf_ = true;

    impl_->publish_wheel_joint_state_ = true;

    impl_->publish_odom_ = true;

    // 가제보 플러그인 태그
    impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0).first;
    impl_->max_wheel_decel_ = _sdf->Get<double>(
                                      "max_wheel_deceleration", impl_->max_wheel_accel_)
                                  .first;
    impl_->max_wheel_speed_tol_ = _sdf->Get<double>("max_wheel_speed_tolerance", 0.01).first;

    impl_->wheel_separation_ = _sdf->Get<double>("wheel_separation", 0.9).first;

    impl_->joints_.resize(3); // 조향 추가시 4로 변경

    // OBTAIN -> JOINTS:
    impl_->joints_[TricycleDrivePluginPrivate::DRIVE_WHEEL] = _model->GetJoint("drive_wheel_joint");
    impl_->joints_[TricycleDrivePluginPrivate::FRONT_WHEEL_LEFT] = _model->GetJoint("front_wheel_l_joint");
    impl_->joints_[TricycleDrivePluginPrivate::FRONT_WHEEL_RIGHT] = _model->GetJoint("front_wheel_r_joint");
    // impl_->joints_[3] = _model->GetJoint("steering_joint");

    impl_->max_wheel_torque_ = 25;
    impl_->joints_[TricycleDrivePluginPrivate::DRIVE_WHEEL]->SetParam(
        "fmax", 0, impl_->max_wheel_torque_);

    auto publish_rate = 100;

    impl_->update_period_ = 1 / publish_rate;

    impl_->last_actuator_update_ = _model->GetWorld()->SimTime();

    // Create status publisher
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)), std::bind(&TricycleDrivePluginPrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

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
        std::bind(&TricycleDrivePluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "GAZEBO Driving plugin loaded successfully.");
  }

  void TricycleDrivePlugin::Reset()
  {
    std::lock_guard<std::mutex> scoped_lock(impl_->lock_);

    if (impl_->joints_[TricycleDrivePluginPrivate::DRIVE_WHEEL])
    {
      gazebo::common::Time current_time =
          impl_->joints_[TricycleDrivePluginPrivate::DRIVE_WHEEL]->GetWorld()->SimTime();
      impl_->joints_[TricycleDrivePluginPrivate::DRIVE_WHEEL]->SetParam(
          "fmax", 0, impl_->max_wheel_torque_);
      impl_->joints_[TricycleDrivePluginPrivate::DRIVE_WHEEL]->SetParam("vel", 0, 0.0);
      impl_->last_actuator_update_ = current_time;
      impl_->last_odom_update_ = current_time;
    }
    impl_->pose_encoder_.x = 0;
    impl_->pose_encoder_.y = 0;
    impl_->pose_encoder_.theta = 0;
    impl_->cmd_.linear.x = 0;
    impl_->cmd_.angular.z = 0;
  }

  // Link간 연결을 보여주는 Joint_state
  void TricycleDrivePluginPrivate::PublishWheelJointState(
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
  void TricycleDrivePluginPrivate::PublishWheelsTf(const gazebo::common::Time &_current_time)
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

  void TricycleDrivePluginPrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
  {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("TricycleDrivePluginPrivate::OnUpdate");
    IGN_PROFILE_BEGIN("UpdateOdometryEncoder");
#endif
    gazebo::common::Time current_time = _info.simTime;
    if (odom_source_ == ENCODER)
    {
      UpdateOdometryEncoder(current_time);
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    double seconds_since_last_update = (current_time - last_actuator_update_).Double();

    if (seconds_since_last_update < update_period_)
    {
      return;
    }

    if (publish_odom_)
    {
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
      PublishOdometryMsg(current_time);
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
    }

    if (publish_wheel_tf_)
    {
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
      PublishWheelsTf(current_time);
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
    }
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
    double target_wheel_rotation_speed = cmd_.linear.x / drive_wheel_radius_;
    lock.unlock();

#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("MotorController");
#endif
    MotorController(
        target_wheel_rotation_speed, seconds_since_last_update);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    //  RCLCPP_INFO(ros_node_->get_logger(),
    //  "v = %f, w = %f ", target_wheel_rotation_speed, target_steering_angle);

    last_actuator_update_ = _info.simTime;
  }

  void TricycleDrivePluginPrivate::MotorController(
      double target_speed, double dt)
  {
    double applied_speed = target_speed;

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
  }

  void TricycleDrivePluginPrivate::OnCmdVel(
      const geometry_msgs::msg::Twist::ConstSharedPtr cmd_msg)
  {
    std::lock_guard<std::mutex> scoped_lock(lock_);
    cmd_.linear.x = cmd_msg->linear.x;
    cmd_.angular.z = cmd_msg->angular.z;
  }

  void TricycleDrivePluginPrivate::UpdateOdometryEncoder(
      const gazebo::common::Time &_current_time)
  {
    // 마치 앞바퀴에 엔코더가 달려 해당 속도를 받아오는 것과 같이 속도를 읽어오는 부분
    // double vl = joints_[FRONT_WHEEL_LEFT]->GetVelocity(0);
    // RCLCPP_INFO(ros_node_->get_logger(), "%f", vl);
    // double vr = joints_[FRONT_WHEEL_RIGHT]->GetVelocity(0);

    double vd = joints_[DRIVE_WHEEL]->GetVelocity(0);

    double seconds_since_last_update = (_current_time - last_odom_update_).Double();
    last_odom_update_ = _current_time;

    // double b = wheel_separation_;

    // Book: Sigwart 2011 Autonomous Mobile Robots page:337
    // double sl = vl * (front_wheel_radius_)*seconds_since_last_update;
    // double sr = vr * (front_wheel_radius_)*seconds_since_last_update;

    double sd = vd * (drive_wheel_radius_)*seconds_since_last_update;

    // double dx = (sl + sr) / 2.0 * cos(pose_encoder_.theta + (sl - sr) / (2.0 * b));
    // double dy = (sl + sr) / 2.0 * sin(pose_encoder_.theta + (sl - sr) / (2.0 * b));

    double dxd = sd / 2.0 * sin(pose_encoder_.theta);
    double dyd = sd / 2.0 * cos(pose_encoder_.theta);

    // double dtheta = (sl - sr) / b;
    double dthetad = 0.0;

    // pose_encoder_.x += dx;
    // pose_encoder_.y += dy;
    // pose_encoder_.theta += dtheta;

    pose_encoder_.x += dxd;
    pose_encoder_.y += dyd;
    pose_encoder_.theta += dthetad;

    // double w = dtheta / seconds_since_last_update;

    double w = dthetad / seconds_since_last_update;

    tf2::Vector3 vt;
    vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);
    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    tf2::Quaternion qt;
    qt.setRPY(0, 0, pose_encoder_.theta);
    odom_.pose.pose.orientation = tf2::toMsg(qt);

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = dxd / seconds_since_last_update;
    odom_.twist.twist.linear.y = dyd / seconds_since_last_update;
  }

  void TricycleDrivePluginPrivate::PublishOdometryMsg(const gazebo::common::Time &_current_time)
  {
    rclcpp::Time current_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

    if (odom_source_ == WORLD)
    {
      // getting data form gazebo world
      ignition::math::Pose3d pose = model_->WorldPose();

      odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
      odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

      // get velocity in /odom frame
      ignition::math::Vector3d linear;
      linear = model_->WorldLinearVel();
      odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

      // convert velocity to child_frame_id (aka base_footprint)
      auto yaw = static_cast<float>(pose.Rot().Yaw());
      odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
      odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    }

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = current_time;
    msg.header.frame_id = odometry_frame_;
    msg.child_frame_id = robot_base_frame_;
    msg.transform.translation =
        gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
    msg.transform.rotation = odom_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(msg);

    // set header stamp
    odom_.header.stamp = current_time;

    odom_pub_->publish(odom_);
  }

  // void TricycleDrivePluginPrivate::OnCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  // {
  //   lin_vel_ = msg->linear.x;
  //   ang_vel_ = msg->angular.z;
  // }

  // void TricycleDrivePluginPrivate::OnUpdate()
  // {
  //   rclcpp::Time current_time = ros_node_->get_clock()->now();
  //   double dt = (current_time.seconds() - last_time_.seconds());
  //   last_time_ = current_time;

  //   // Calculate the wheel velocity and steering angle
  //   double drvie_wheel_velocity = lin_vel_ / drive_wheel_radius_;
  //   double front_wheel_velocity = lin_vel_ / front_wheel_radius_;
  //   double steering_angle = atan(wheel_base_ * ang_vel_ / lin_vel_);

  //   // Set joint positions
  //   static double drvie_wheel_position = 0.0;
  //   static double front_wheel_position = 0.0;

  //   drvie_wheel_position += drvie_wheel_velocity * dt;
  //   front_wheel_position += front_wheel_velocity * dt;

  //   // drive_wheel_joint_->SetVelocity(0, drvie_wheel_velocity);
  //   // steering_joint_->SetPosition(0, steering_angle);

  //   // Publish joint states
  //   joint_state_.header.stamp = ros_node_->get_clock()->now();
  //   for (std::size_t i = 0; i < joints_.size(); i++)
  //   {
  //     if (i == 0)
  //     {
  //       joint_state_.position[i] = drvie_wheel_position;
  //       joint_state_.velocity[i] = drvie_wheel_velocity;
  //       joint_state_.effort[i] = 0.0;
  //     }
  //     else
  //     {
  //       joint_state_.position[i] = front_wheel_position;
  //       joint_state_.velocity[i] = front_wheel_velocity;
  //       joint_state_.effort[i] = 0.0;
  //     }
  //   }
  //   joint_state_pub_->publish(joint_state_);

  //   // Publish odometry
  //   // double delta_x = lin_vel_ * cos(yaw_) * dt;
  //   // double delta_y = lin_vel_ * sin(yaw_) * dt;
  //   // double delta_yaw = ang_vel_ * dt;

  //   double delta_x = 0;
  //   double delta_y = lin_vel_ * dt;
  //   double delta_yaw = ang_vel_ * dt;

  //   // x_ += delta_x;
  //   y_ += delta_y;
  //   yaw_ += delta_yaw;

  //   nav_msgs::msg::Odometry odom_msg;
  //   odom_msg.header.stamp = ros_node_->get_clock()->now();
  //   odom_msg.header.frame_id = "odom";
  //   odom_msg.child_frame_id = "base_footprint";
  //   odom_msg.pose.pose.position.x = 0.0;
  //   odom_msg.pose.pose.position.y = y_;
  //   odom_msg.pose.pose.position.z = 0.0;
  //   tf2::Quaternion q;
  //   q.setRPY(0, 0, yaw_);
  //   tf2::convert(q, odom_.pose.pose.orientation);
  //   odom_msg.twist.twist.linear.x = lin_vel_;
  //   odom_msg.twist.twist.angular.z = ang_vel_;
  //   odom_pub_->publish(odom_msg);

  //   // Publish transform
  //   geometry_msgs::msg::TransformStamped odom_tf;
  //   odom_tf.header.stamp = ros_node_->get_clock()->now();
  //   odom_tf.header.frame_id = "odom";
  //   odom_tf.child_frame_id = "base_footprint";
  //   odom_tf.transform.translation.x = 0.0;
  //   odom_tf.transform.translation.y = y_;
  //   odom_tf.transform.translation.z = 0.0;
  //   tf2::convert(q, odom_.pose.pose.orientation);
  //   tf_broadcaster_->sendTransform(odom_tf);

  //   for (auto &joint : joints_)
  //   {
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
  //   // for (std::size_t i = 0; i < joints_.size(); i++)
  //   // {
  //   //   if (i == 0)
  //   //   {
  //   //     std::string frame = "drive_wheel_join";
  //   //     std::string parent_frame = "steering_part";

  //   //     geometry_msgs::msg::TransformStamped msg;
  //   //     msg.header.stamp = ros_node_->get_clock()->now();
  //   //     msg.header.frame_id = parent_frame;
  //   //     msg.child_frame_id = frame;
  //   //     msg.transform.translation.x = 0.0;
  //   //     msg.transform.translation.y = y_;
  //   //     msg.transform.translation.z = 0.0;
  //   //     tf2::convert(q, odom_.pose.pose.orientation);
  //   //     tf_broadcaster_->sendTransform(msg);
  //   //   }
  //   //   else if (i == 1)
  //   //   {
  //   //     std::string frame = "front_wheel_l_joint";
  //   //     std::string parent_frame = "base_link";

  //   //     geometry_msgs::msg::TransformStamped msg;
  //   //     msg.header.stamp = ros_node_->get_clock()->now();
  //   //     msg.header.frame_id = parent_frame;
  //   //     msg.child_frame_id = frame;
  //   //     msg.transform.translation.x = 0.0;
  //   //     msg.transform.translation.y = y_;
  //   //     msg.transform.translation.z = 0.0;
  //   //     tf2::convert(q, odom_.pose.pose.orientation);
  //   //     tf_broadcaster_->sendTransform(msg);
  //   //   }
  //   //   else
  //   //   {
  //   //     std::string frame = "front_wheel_r_joint";
  //   //     std::string parent_frame = "base_link";

  //   //     geometry_msgs::msg::TransformStamped msg;
  //   //     msg.header.stamp = ros_node_->get_clock()->now();
  //   //     msg.header.frame_id = parent_frame;
  //   //     msg.child_frame_id = frame;
  //   //     msg.transform.translation.x = 0.0;
  //   //     msg.transform.translation.y = y_;
  //   //     msg.transform.translation.z = 0.0;
  //   //     tf2::convert(q, odom_.pose.pose.orientation);
  //   //     tf_broadcaster_->sendTransform(msg);
  //   //   }
  //   //   // ignition::math::Pose3d pose = joint->GetChild()->RelativePose();
  //   // }

  //   // for (auto &joint : joints_)
  //   // {
  //   //   std::string frame = joint->GetName();
  //   //   std::string parent_frame = joint->GetParent()->GetName();

  //   //   ignition::math::Pose3d pose = joint->GetChild()->RelativePose();

  //   //   geometry_msgs::msg::TransformStamped msg;
  //   //   msg.header.stamp = ros_node_->get_clock()->now();
  //   //   msg.header.frame_id = parent_frame;
  //   //   msg.child_frame_id = frame;
  //   //   msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
  //   //   msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());
  //   //   transform_broadcaster_->sendTransform(msg);
  //   // }
  // }

  GZ_REGISTER_MODEL_PLUGIN(TricycleDrivePlugin)
} // namespace gazebo_ros