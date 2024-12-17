// Original Author: Philipp Wuestenberg <philipp.wuestenberg@tu-berlin.de>
// Maintainer: Jonathan Blixti <blixt013@umn.edu>
// Last Updated: November 2023

// Import header file
#include "agv_com/agv_can_bus.hpp"

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

// ?????? ?? ?? ??

AgvCanBus::AgvCanBus() : Node("AgvCanBus"),
                         drive_rpm_(0),
                         agv_direction_(1),
                         max_steering_speed_(100),
                         max_steering_angle_tol_(5),
                         applied_steering_speed(0),
                         mast_vel_(0.0),
                         hydraulic_direction_(0),
                         theta_(0.0)
{
  this->declare_parameter("CAN_INTERFACE", "can0");
  std::string can_socket = this->get_parameter("CAN_INTERFACE").as_string();

  int8_t qos_depth = 0;

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
  topicname_receive = "CAN/can0/receive";
  topicname_transmit = "CAN/can0/transmit";

  // can data ??
  subscription_ = this->create_subscription<can_msgs::msg::Frame>(topicname_receive, QOS_RKL10V, std::bind(&AgvCanBus::CanReceiver, this, std::placeholders::_1));
  // can data ???
  publisher_ = this->create_publisher<can_msgs::msg::Frame>(topicname_transmit, 10);
  cmdvel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", QOS_RKL10V, std::bind(&AgvCanBus::CmdVelCallback, this, std::placeholders::_1));

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", QOS_RKL10V);

  joint_state_.name.resize(2);
  joint_state_.position.resize(2);
  joint_state_.velocity.resize(2);
  joint_state_.effort.resize(2);

  joint_state_.name[0] = "drive_wheel_joint";
  joint_state_.name[1] = "steering_joint";

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", QOS_RKL10V);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  mast_vel_sub_ = this->create_subscription<agv_msgs::msg::ForkControl>(
      "/mast_vel", QOS_RKL10V, std::bind(&AgvCanBus::MastVelCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", QOS_RKL10V, std::bind(&AgvCanBus::ImuCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AgvCanBus::OnUpdate, this));
}

AgvCanBus::~AgvCanBus()
{
}

//
// Can Data Read
//

void AgvCanBus::CanReceiver(const can_msgs::msg::Frame::SharedPtr msg)
{
  can_msgs::msg::Frame msg1;
  msg1.id = msg->id;
  msg1.dlc = msg->dlc;
  msg1.is_extended = msg->is_extended;
  msg1.is_rtr = msg->is_rtr;
  msg1.is_error = msg->is_error;
  msg1.data = msg->data;

  if (msg1.id == DRIVER_TO_PC_CAN1)
  {
    CanRead1(msg1);
  }
  else if (msg1.id == DRIVER_TO_PC_CAN2)
  {
    CanRead2(msg1);
  }
  else if (msg1.id == DRIVER_TO_PC_CAN3)
  {
    CanRead3(msg1);
  }
  else if (msg1.id == DRIVER_TO_PC_CAN4)
  {
    CanRead4(msg1);
  }
  else if (msg1.id == DRIVER_TO_PC_CAN5)
  {
    CanRead5(msg1);
  }
  // test
  // else if (msg1.id == 7 || msg1.id == 10)
  // {
  //   CanReadBitTest(msg1);
  // }
}

void AgvCanBus::CanRead1(const can_msgs::msg::Frame msg)
{
  drive_motor_current_rpm_low_byte = msg.data[0];
  drive_motor_current_rpm_high_byte = msg.data[1];
  can_rpm_byte = combineBytes(drive_motor_current_rpm_low_byte, drive_motor_current_rpm_high_byte);
  can_rpm = static_cast<int16_t>(can_rpm_byte);

  steering_current_angle_low_byte = msg.data[2];
  steering_current_angle_high_byte = msg.data[3];
  can_steer_ang_byte = combineBytes(steering_current_angle_low_byte, steering_current_angle_high_byte);
  can_steer_ang_test = combineBytes(steering_current_angle_low_byte, steering_current_angle_high_byte);

  if (can_steer_ang_byte < 0x464f)
  {
    can_steer_ang = static_cast<int>(can_steer_ang_byte);
  }
  else
  {
    can_steer_ang_byte = (~can_steer_ang_byte) + 1;
    can_steer_ang = -static_cast<int>(can_steer_ang_byte);
  }

  drive_motor_temperature_low_byte = msg.data[4];
  drive_motor_temperature_high_byte = msg.data[5];
  can_drive_motor_temperature_byte = combineBytes(drive_motor_temperature_low_byte, drive_motor_temperature_high_byte);
  can_drive_motor_temperature = static_cast<int>(can_drive_motor_temperature_byte);

  steering_motor_temperature_low_byte = msg.data[6];
  steering_motor_temperature_high_byte = msg.data[7];
  can_steering_motor_temperature_byte = combineBytes(steering_motor_temperature_low_byte, steering_motor_temperature_high_byte);
  can_steering_motor_temperature = static_cast<int>(can_steering_motor_temperature_byte);
}

void AgvCanBus::CanRead2(const can_msgs::msg::Frame msg)
{
  drive_motor_current_low_byte = msg.data[0];
  drive_motor_current_high_byte = msg.data[1];
  can_drive_motor_current_byte = combineBytes(drive_motor_current_low_byte, drive_motor_current_high_byte);
  can_drive_motor_current = static_cast<int>(can_drive_motor_current_byte);

  steering_current_low_byte = msg.data[2];
  steering_current_high_byte = msg.data[3];
  can_steering_current_byte = combineBytes(steering_current_low_byte, steering_current_high_byte);
  can_steering_current = static_cast<int>(can_steering_current_byte);

  distance_fine_byte1 = msg.data[4];
  distance_fine_byte2 = msg.data[5];
  distance_fine_byte3 = msg.data[6];
  distance_fine_byte4 = msg.data[7];
  can_distance_fine_byte = (((static_cast<uint32_t>(distance_fine_byte1) << 24) | distance_fine_byte2) << 16 | distance_fine_byte3) << 8 | distance_fine_byte4;
  can_distance_fine = static_cast<long>(can_distance_fine_byte);
}

void AgvCanBus::CanRead3(const can_msgs::msg::Frame msg)
{
  traction_throttle_analog_byte = msg.data[0];
  can_traction_throttle_analog = static_cast<u_char>(traction_throttle_analog_byte);
  steer_analog_byte = msg.data[1];
  can_steer_analog = static_cast<u_char>(steer_analog_byte);
  drive_pd_val_byte = msg.data[2];
  can_drive_pd_val = static_cast<u_char>(drive_pd_val_byte);

  drive_interlock = msg.data[3] & 0x01;
  drive_emg_reverse = (msg.data[3] >> 1) & 0x01;
  forward = (msg.data[3] >> 2) & 0x01;
  reverse = (msg.data[3] >> 3) & 0x01;
  manual_auto_set = (msg.data[3] >> 7) & 0x01;

  em_break = msg.data[4] & 0x01;
  back_buzzer = (msg.data[4] >> 1) & 0x01;
  horn = (msg.data[4] >> 2) & 0x01;
  drive_main_coil = (msg.data[4] >> 3) & 0x01;

  drive_controller_fault_code_byte = msg.data[5];
  can_hydraulic_controller_fault_code = static_cast<u_char>(drive_controller_fault_code_byte);
  steering_controller_fault_code_byte = msg.data[6];
  can_steering_controller_fault_code = static_cast<u_char>(steering_controller_fault_code_byte);
  hydraulic_controller_fault_code_byte = msg.data[7];
  can_hydraulic_controller_fault_code = static_cast<u_char>(hydraulic_controller_fault_code_byte);
}

void AgvCanBus::CanRead4(const can_msgs::msg::Frame msg)
{
  hydraulic_motor_current_rpm_low_byte = msg.data[0];
  hydraulic_motor_current_rpm_high_byte = msg.data[1];
  can_hydraulic_motor_current_rpm_byte = combineBytes(hydraulic_motor_current_rpm_low_byte, hydraulic_motor_current_rpm_high_byte);
  can_hydraulic_motor_current_rpm = static_cast<uint>(can_hydraulic_motor_current_rpm_byte);

  hydraulic_motor_temperature_low_byte = msg.data[2];
  hydraulic_motor_temperature_high_byte = msg.data[3];
  can_hydraulic_motor_temperature_byte = combineBytes(hydraulic_motor_temperature_low_byte, hydraulic_motor_temperature_high_byte);
  can_hydraulic_motor_temperature = static_cast<int>(can_hydraulic_motor_temperature_byte);

  hydraulic_motor_current_low_byte = msg.data[4];
  hydraulic_motor_current_high_byte = msg.data[5];
  can_hydraulic_motor_current_byte = combineBytes(hydraulic_motor_current_low_byte, hydraulic_motor_current_high_byte);
  can_hydraulic_motor_current = static_cast<int>(can_hydraulic_motor_current_byte);

  forktip_sensor_right = msg.data[6] & 0x01;
  forktip_sensor_left = (msg.data[6] >> 1) & 0x01;
  detect_sensor_front_and_back = (msg.data[6] >> 2) & 0x01;
  detect_sensor_top_and_push = (msg.data[6] >> 3) & 0x01;
  bumper_switch = (msg.data[6] >> 4) & 0x01;
}

void AgvCanBus::CanRead5(const can_msgs::msg::Frame msg)
{
  lift_encoder_low_byte = msg.data[0];
  lift_encoder_high_byte = msg.data[1];
  can_lift_encoder_byte = combineBytes(lift_encoder_low_byte, lift_encoder_high_byte);
  can_lift_encoder = static_cast<uint>(can_lift_encoder_byte);
}

// Byte Combine Func
int16_t AgvCanBus::combineBytes(const uint8_t lowByte, const uint8_t highByte)
{

  int16_t combined = ((highByte) << 8) | lowByte;

  return combined;
}
//
// Can Data Write
//
void AgvCanBus::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  imu_.orientation.w = msg->orientation.w;
  imu_.orientation.x = msg->orientation.x;
  imu_.orientation.y = msg->orientation.y;
  imu_.orientation.z = msg->orientation.z;

  imu_.angular_velocity.x = msg->angular_velocity.x;
  imu_.angular_velocity.y = msg->angular_velocity.y;
  imu_.angular_velocity.z = msg->angular_velocity.z;

  imu_.linear_acceleration.x = msg->linear_acceleration.x;
  imu_.linear_acceleration.y = msg->linear_acceleration.y;
  imu_.linear_acceleration.z = msg->linear_acceleration.z;
}

void AgvCanBus::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_.linear.x = msg->linear.x;      // cmd_vel??? ?? ??? ??
  cmd_.angular.z = -(msg->angular.z); // cmd_vel??? ???? ??, ?? ???? ??
}

void AgvCanBus::OnUpdate()
{
  auto current_time = this->now();
  MotorController();
  UpdateOdometryEncoder();
  PublishWheelsTf(current_time);
  PublishOdometryMsg(current_time);
  PublishWheelJointState(current_time);
}

void AgvCanBus::PublishWheelsTf(const rclcpp::Time &current_time)
{
  // rclcpp::Time current_time = this->now();

  double dt = (this->now().seconds()) - current_time.seconds();

  static double wheel_pos = 0.0;
  double wheel_vel = can_rpm * M_PI / (30 * 16.755);

  wheel_pos += wheel_vel * dt;

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = current_time;
  msg.header.frame_id = "base_link";
  msg.child_frame_id = "steering_part";
  msg.transform.rotation.x = wheel_pos;
  tf_broadcaster_->sendTransform(msg);

  msg.header.stamp = current_time;
  msg.header.frame_id = "steering_part";
  msg.child_frame_id = "drive_wheel";
  msg.transform.rotation.z = (can_steer_ang / 100);
  tf_broadcaster_->sendTransform(msg);
}

void AgvCanBus::PublishWheelJointState(const rclcpp::Time &current_time)
{
  joint_state_.header.stamp = current_time;

  // joint_state_.header.frame_id = "base_link";

  double dt = (this->now().seconds()) - current_time.seconds();

  static double wheel_pos = 0.0;
  double wheel_vel = can_rpm * M_PI / (30 * 16.755);

  wheel_pos += wheel_vel * dt;

  joint_state_.position[0] = wheel_pos;
  joint_state_.velocity[0] = wheel_vel;

  joint_state_.position[1] = (can_steer_ang / 100);
  joint_state_.velocity[1] = 0.0;

  joint_state_pub_->publish(joint_state_);
}

void AgvCanBus::MotorController()
{
  std::vector<uint8_t> rpm_result(2);
  std::vector<uint8_t> ang_result(2);

  // lin_vel to RPM

  if (cmd_.linear.x > 0)
  {
    agv_direction_ = 1;
  }
  else if (cmd_.linear.x < 0)
  {
    agv_direction_ = 0;
    cmd_.linear.x = -cmd_.linear.x;
  }
  else if (cmd_.linear.x == 0.0)
  {
    agv_direction_ = 2;
    cmd_.linear.x = -cmd_.linear.x;
  }

  // (cmd_.linear.x * 60)[m/Min] / (0.32 * M_PI)       [Rev/Min]
  drive_rpm_ = 16.755 * (cmd_.linear.x * 60) / (0.32 * M_PI);
  drive_rpm_ = (uint)drive_rpm_;

  // ang_vel to steering
  int applied_angle = rad_to_dec(cmd_.angular.z);
  uint16_t applied_angle_byte;
  int16_t applied_angle_result;

  int current_angle = can_steer_ang;
  int diff_angle = current_angle - rad_to_dec(cmd_.angular.z);

  auto start = this->now().seconds();

  if (rad_to_dec(max_steering_angle_tol_) > 0)
  {
    // this means we will steer using steering speed
    if (fabs(diff_angle) < rad_to_dec(max_steering_angle_tol_))
    {
      applied_steering_speed = 0;
    }
    else
    {
      if (diff_angle > 0)
      {
        applied_steering_speed = -max_steering_speed_;
      }
      else if (diff_angle < 0)
      {
        applied_steering_speed = max_steering_speed_;
      }
      auto end = this->now().seconds();
      double seconds = end - start;

      applied_angle = current_angle + (applied_steering_speed * seconds);
    }
    applied_angle_byte = (uint16_t)applied_angle;
  }

  ang_result[0] = (applied_angle_byte & 0xff);
  ang_result[1] = ((applied_angle_byte >> 8) & 0xff);

  rpm_result[0] = (drive_rpm_ & 0xff);
  rpm_result[1] = ((drive_rpm_ >> 8) & 0xff);

  write_frame.id = PC_TO_DRIVER_CAN1;

  write_frame.dlc = 8;

  write_frame.data[0] = rpm_result[0];
  write_frame.data[1] = rpm_result[1];
  write_frame.data[2] = ang_result[0];
  write_frame.data[3] = ang_result[1];

  if (agv_direction_ == 1)
  {
    write_frame.data[4] = 13;
  }
  else if (agv_direction_ == 0)
  {
    write_frame.data[4] = 21;
  }
  else if (agv_direction_ == 2)
  {
    if (applied_angle != 0x00)
    {
      write_frame.data[4] = 13;
    }
    else
    {
      write_frame.data[4] = 5;
    }
  }
  publisher_->publish(write_frame);
}

void AgvCanBus::UpdateOdometryEncoder()
{
  // tf2::Quaternion quat;
  // tf2::fromMsg(imu_.orientation, quat);
  // double roll, pitch, yaw;
  // tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  // theta_ = yaw;

  tf2::Quaternion quat(imu_.orientation.x, imu_.orientation.y, imu_.orientation.z, imu_.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  double current_time = this->now().seconds();

  auto dt = (current_time - last_odom_update_);
  last_odom_update_ = current_time;

  double seconds_since_last_update = dt;
  static double last_theta = 0;
  // double theta = yaw;

  // double dtheta = last_theta - theta;


  // can_rpm(motor's rpm)/16.755 => wheel's rpm
  // drive_rpm_ = 16.755 * (cmd_.linear.x * 60) / (0.32 * M_PI);
  // v = rpm * 0.32 (wheel_dia) * pi / 60
  double vd = can_rpm * (0.16) * M_PI / (30 * 16.755);
  double sd = vd * seconds_since_last_update;
  // double tan_ang = tan((float)can_steer_ang * M_PI / 18000);
  // double delta_theta = (vd * tan_ang) * dt;

  double dx = sd * cos(theta_);
  double dy = sd * sin(theta_);

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  // theta_ += delta_theta;
  // pose_encoder_.theta = theta_;

  // RCLCPP_INFO(this->get_logger(), "-----------------------");
  // RCLCPP_INFO(this->get_logger(), "pose_encoder_x : %f", pose_encoder_.x);
  // RCLCPP_INFO(this->get_logger(), "pose_encoder_y : %f", pose_encoder_.y);
  // RCLCPP_INFO(this->get_logger(), "pose_encoder_theta : %f", pose_encoder_.theta);

  odom_.header.stamp = this->now();
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";

  odom_.pose.pose.position.x = pose_encoder_.x;
  odom_.pose.pose.position.y = pose_encoder_.y;
  odom_.pose.pose.position.z = 0.0;

  odom_.pose.pose.orientation.x = imu_.orientation.x;
  odom_.pose.pose.orientation.y = imu_.orientation.y;
  odom_.pose.pose.orientation.z = imu_.orientation.z;
  odom_.pose.pose.orientation.w = imu_.orientation.w;

  // RCLCPP_INFO(this->get_logger(), "-----------------------");
  // RCLCPP_INFO(this->get_logger(), "odom_.pose.pose.position.x : %f", odom_.pose.pose.position.x);
  // RCLCPP_INFO(this->get_logger(), "odom_.pose.pose.position.y : %f", odom_.pose.pose.position.y);
  // RCLCPP_INFO(this->get_logger(), "odom_.pose.pose.orientation.x : %f", odom_.pose.pose.orientation.x);
  // RCLCPP_INFO(this->get_logger(), "odom_.pose.pose.orientation.y : %f", odom_.pose.pose.orientation.y);
  // RCLCPP_INFO(this->get_logger(), "odom_.pose.pose.orientation.z : %f", odom_.pose.pose.orientation.z);
  // RCLCPP_INFO(this->get_logger(), "odom_.pose.pose.orientation.w : %f", odom_.pose.pose.orientation.w);

  // odom_.pose.pose.position.x = 0;
  // odom_.pose.pose.position.y = 0;
  // odom_.pose.pose.position.z = 0;

  // tf2::Quaternion qt;
  // qt.setRPY(0, 0, pose_encoder_.theta);
  // odom_.pose.pose.orientation = tf2::toMsg(qt);

  // odom_.twist.twist.angular.z = (vd) * tan(can_steer_ang * M_PI / 18000);
  odom_.twist.twist.angular.z = imu_.angular_velocity.z;
  odom_.twist.twist.linear.x = vd;
  odom_.twist.twist.linear.y = 0;

  // RCLCPP_INFO(this->get_logger(), "-----------------------");
  // RCLCPP_INFO(this->get_logger(), "odom_.twist.twist.linear.x : %f", odom_.twist.twist.linear.x);
  // RCLCPP_INFO(this->get_logger(), "odom_.twist.twist.linear.y : %f", odom_.twist.twist.linear.y);
  // RCLCPP_INFO(this->get_logger(), "odom_.twist.twist.angular.z : %f", odom_.twist.twist.angular.z);

  // odom_.twist.twist.angular.z = 0;
  // odom_.twist.twist.linear.x = 0;
  // odom_.twist.twist.linear.y = 0;

  // last_theta = theta;
}

void AgvCanBus::PublishOdometryMsg(const rclcpp::Time &current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = current_time;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_footprint";
  msg.transform.translation.x = odom_.pose.pose.position.x;
  msg.transform.translation.y = odom_.pose.pose.position.y;
  msg.transform.translation.z = odom_.pose.pose.position.z;
  msg.transform.rotation = imu_.orientation;

  tf_broadcaster_->sendTransform(msg);

  // set header stamp
  odom_.header.stamp = current_time;

  odom_pub_->publish(odom_);
}

void AgvCanBus::MastVelCallback(const agv_msgs::msg::ForkControl::ConstSharedPtr msg)
{
  mast_vel_ = 1000 * msg->fork_vel;
  MastTrans(mast_vel_);
}

void AgvCanBus::MastTrans(double target_speed)
{
  // std::stringstream out;

  std::vector<uint8_t> mast_result(2);

  u_char hydraluic_propor_vel;

  hydraluic_propor_vel = '115';
  // ?? ?? ??
  if (mast_vel_ > 0)
  {
    hydraulic_direction_ = 1;
    mast_result[0] = 0xE8;
    mast_result[1] = 0x03;
  }
  else if (mast_vel_ < 0)
  {
    hydraulic_direction_ = 0;
    mast_result[0] = 0x00;
    mast_result[1] = 0x00;
  }
  else if (mast_vel_ == 0)
  {
    hydraulic_direction_ = 2;
    mast_result[0] = 0x00;
    mast_result[1] = 0x00;
  }

  // out << std::string("mast_result[0] : ") << std::to_string(mast_result[0]) << std::endl;
  // out << std::string("mast_result[1] : ") << std::to_string(mast_result[1]) << std::endl;

  write_frame.id = PC_TO_DRIVER_CAN2;

  write_frame.dlc = 8;

  write_frame.data[0] = mast_result[0];
  write_frame.data[1] = mast_result[1];

  if (hydraulic_direction_ == 1) // ??
  {
    write_frame.data[2] = 7;
    // if (((write_frame.data[2] >> 1) & 0x01) != 0x01)
    // {
    //   write_frame.data[2] = write_frame.data[2] ^ 2;
    // }
    write_frame.data[3] = 1;
  }
  else if (hydraulic_direction_ == 0) // ??
  {
    write_frame.data[2] = 5;
    // if (((write_frame.data[2] >> 1) & 0x01) == 0x01)
    // {
    //   write_frame.data[2] = write_frame.data[2] ^ 2;
    // }
    write_frame.data[3] = 2;
  }
  else if (hydraulic_direction_ == 2) // ??
  {
    hydraluic_propor_vel = '40';
    if (((write_frame.data[2]) & 0x03) != 0x04)
    {
      write_frame.data[2] = 0x04;
    }
    write_frame.data[3] = 0x00;
  }

  write_frame.data[4] = hydraluic_propor_vel;
  hyd_test = write_frame.data[4];

  // out << std::string("-------------Write ID : ") << std::to_string(write_frame.id) << std::string("------------------") << std::endl;
  // out << std::string("hydraulicLowByte : ") << std::to_string(write_frame.data[0]) << std::endl;
  // out << std::string("hydraulicHighByte : ") << std::to_string(write_frame.data[1]) << std::endl;
  // out << std::string("hydraulicWorking : ") << std::to_string((write_frame.data[2] >> 1) & 0x01) << std::endl;
  // out << std::string("hydraulicForward : ") << std::to_string((write_frame.data[3]) & 0x01) << std::endl;
  // out << std::string("hydraulicBackward : ") << std::to_string((write_frame.data[3] >> 1) & 0x01) << std::endl;
  // out << std::string("hydraluic_propor_vel : ") << std::to_string(write_frame.data[4]) << std::endl;
  // out << std::string("hydraluic_propor_vel_char : ") << std::to_string(hyd_test) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());
  publisher_->publish(write_frame);

  // RCLCPP_INFO(ros_node_->get_logger(), "[%f]", joints_[0]->Position(0));
}

int AgvCanBus::rad_to_dec(float rad_ang_)
{
  int angle = rad_ang_ * 18000 / M_PI;

  return angle;
}

void AgvCanBus::DebugLog()
{
  std::stringstream out;
  // CanRead1
  // out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  // out << std::string("drive_motor_current_rpm_low_byte : ") << std::to_string(drive_motor_current_rpm_low_byte) << std::endl;
  // out << std::string("drive_motor_current_rpm_high_byte : ") << std::to_string(drive_motor_current_rpm_high_byte) << std::endl;
  // out << std::string(" => drive_motor_current_rpm_byte : ") << std::to_string(can_rpm_byte) << std::endl;
  // out << std::string(" ==> drive_motor_current_rpm_value : ") << std::to_string(can_rpm) << std::endl;

  // out << std::string("steering_current_angle_low_byte : ") << std::to_string(steering_current_angle_low_byte) << std::endl;
  // out << std::string("steering_current_angle_high_byte : ") << std::to_string(steering_current_angle_high_byte) << std::endl;
  // out << std::string(" => steering_current_angle_byte : ") << std::to_string(can_steer_ang_byte) << std::endl;
  // out << std::string(" ==> steering_current_angle_value : ") << std::to_string(can_steer_ang) << std::endl;

  // out << std::string("drive_motor_temperature_low_byte : ") << std::to_string(drive_motor_temperature_low_byte) << std::endl;
  // out << std::string("drive_motor_temperature_high_byte : ") << std::to_string(drive_motor_temperature_high_byte) << std::endl;
  // out << std::string(" => drive_motor_temperature_byte : ") << std::to_string(can_drive_motor_temperature_byte) << std::endl;
  // out << std::string(" ==> drive_motor_temperature_value : ") << std::to_string(can_drive_motor_temperature) << std::endl;

  // out << std::string("steering_motor_temperature_low_byte : ") << std::to_string(steering_motor_temperature_low_byte) << std::endl;
  // out << std::string("steering_motor_temperature_high_byte : ") << std::to_string(steering_motor_temperature_high_byte) << std::endl;
  // out << std::string(" => steering_motor_temperature_byte : ") << std::to_string(can_steering_motor_temperature_byte) << std::endl;
  // out << std::string(" ==> steering_motor_temperature_value : ") << std::to_string(can_steering_motor_temperature) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;

  // CanRead2
  //  out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  //  out << std::string("drive_motor_current_low_byte : ") << std::to_string(drive_motor_current_low_byte) << std::endl;
  //  out << std::string("drive_motor_current_high_byte : ") << std::to_string(drive_motor_current_high_byte) << std::endl;
  //  out << std::string(" => drive_motor_current : ") << std::to_string(can_drive_motor_current_byte) << std::endl;
  //  out << std::string(" ==> drive_motor_current_value : ") << std::to_string(can_drive_motor_current) << std::endl;

  // out << std::string("steering_current_low_byte : ") << std::to_string(steering_current_low_byte) << std::endl;
  // out << std::string("steering_current_high_byte : ") << std::to_string(steering_current_high_byte) << std::endl;
  // out << std::string(" => steering_current_byte : ") << std::to_string(can_steering_current_byte) << std::endl;
  // out << std::string(" ==> steering_current_byte_value : ") << std::to_string(can_steering_current_byte) << std::endl;

  // out << std::string("distance_fine_byte1 : ") << std::to_string(distance_fine_byte1) << std::endl;
  // out << std::string("distance_fine_byte2 : ") << std::to_string(distance_fine_byte2) << std::endl;
  // out << std::string("distance_fine_byte3 : ") << std::to_string(distance_fine_byte3) << std::endl;
  // out << std::string("distance_fine_byte4 : ") << std::to_string(distance_fine_byte4) << std::endl;
  // out << std::string(" => distance_fine_byte : ") << std::to_string(can_distance_fine_byte) << std::endl;
  // out << std::string(" ==> distance_fine_value : ") << std::to_string(can_distance_fine) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;

  // CanRead3
  //  out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  //  out << std::string("traction_throttle_analog : ") << std::to_string(traction_throttle_analog_byte) << std::endl;
  //  out << std::string(" ==> traction_throttle_analog_value : ") << std::to_string(can_traction_throttle_analog) << std::endl;

  // out << std::string("steer_analog : ") << std::to_string(steer_analog_byte) << std::endl;
  // out << std::string(" ==> steer_analog_value : ") << std::to_string(can_steer_analog) << std::endl;

  // out << std::string("drive_pd_val : ") << std::to_string(drive_pd_val_byte) << std::endl;
  // out << std::string(" ==> drive_pd_val_value : ") << std::to_string(can_drive_pd_val) << std::endl;

  // out << std::string("drive_interlock : ") << std::to_string(drive_interlock) << std::endl;
  // out << std::string("drive_emg_reverse : ") << std::to_string(drive_emg_reverse) << std::endl;
  // out << std::string("forward : ") << std::to_string(forward) << std::endl;
  // out << std::string("reverse : ") << std::to_string(reverse) << std::endl;
  // out << std::string("manual_auto_set : ") << std::to_string(manual_auto_set) << std::endl;
  // out << std::string("em_break : ") << std::to_string(em_break) << std::endl;
  // out << std::string("back_buzzer : ") << std::to_string(back_buzzer) << std::endl;
  // out << std::string("horn : ") << std::to_string(horn) << std::endl;
  // out << std::string("drive_main_coil : ") << std::to_string(drive_main_coil) << std::endl;

  // out << std::string("drive_controller_fault_code : ") << std::to_string(drive_controller_fault_code_byte) << std::endl;
  // out << std::string(" ==> drive_controller_fault_value : ") << std::to_string(can_hydraulic_controller_fault_code) << std::endl;

  // out << std::string("steering_controller_fault_code : ") << std::to_string(steering_controller_fault_code_byte) << std::endl;
  // out << std::string(" ==> steering_controller_fault_value : ") << std::to_string(can_steering_controller_fault_code) << std::endl;

  // out << std::string("hydraulic_controller_fault_code : ") << std::to_string(hydraulic_controller_fault_code_byte) << std::endl;
  // out << std::string(" ==> hydraulic_controller_fault_value : ") << std::to_string(can_distance_fine) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;

  // CanRead4
  //  out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  //  out << std::string("hydraulic_motor_current_rpm_low_byte : ") << std::to_string(hydraulic_motor_current_rpm_low_byte) << std::endl;
  //  out << std::string("hydraulic_motor_current_rpm_high_byte : ") << std::to_string(hydraulic_motor_current_rpm_high_byte) << std::endl;
  //  out << std::string(" => hydraulic_motor_current_rpm_byte : ") << std::to_string(can_hydraulic_motor_current_rpm_byte) << std::endl;
  //  out << std::string(" ==> hydraulic_motor_current_rpm_value : ") << std::to_string(can_hydraulic_motor_current_rpm) << std::endl;

  // out << std::string("hydraulic_motor_temperature_low_byte : ") << std::to_string(hydraulic_motor_temperature_low_byte) << std::endl;
  // out << std::string("hydraulic_motor_temperature_high_byte : ") << std::to_string(hydraulic_motor_temperature_high_byte) << std::endl;
  // out << std::string(" => hydraulic_motor_temperature_byte : ") << std::to_string(can_hydraulic_motor_temperature_byte) << std::endl;
  // out << std::string(" ==> hydraulic_motor_temperature_value : ") << std::to_string(can_hydraulic_motor_temperature) << std::endl;

  // out << std::string("hydraulic_motor_current_low_byte : ") << std::to_string(hydraulic_motor_current_low_byte) << std::endl;
  // out << std::string("hydraulic_motor_current_high_byte : ") << std::to_string(hydraulic_motor_current_high_byte) << std::endl;
  // out << std::string(" => hydraulic_motor_current_byte : ") << std::to_string(can_hydraulic_motor_current_byte) << std::endl;
  // out << std::string(" ==> hydraulic_motor_current_value : ") << std::to_string(can_hydraulic_motor_current) << std::endl;

  // out << std::string("forktip_sensor_right : ") << std::to_string(forktip_sensor_right) << std::endl;
  // out << std::string("forktip_sensor_left : ") << std::to_string(forktip_sensor_left) << std::endl;
  // out << std::string("detect_sensor_front_and_back : ") << std::to_string(detect_sensor_front_and_back) << std::endl;
  // out << std::string("detect_sensor_top_and_push : ") << std::to_string(detect_sensor_top_and_push) << std::endl;
  // out << std::string("bumper_switch : ") << std::to_string(bumper_switch) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;

  // CanRead5
  //  out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  //  out << std::string("lift_encoder_low_byte : ") << std::to_string(lift_encoder_low_byte) << std::endl;
  //  out << std::string("lift_encoder_high_byte : ") << std::to_string(lift_encoder_high_byte) << std::endl;
  //  out << std::string(" => lift_encoder_byte : ") << std::to_string(can_lift_encoder_byte) << std::endl;
  //  out << std::string(" ==> lift_encoder_value : ") << std::to_string(can_lift_encoder) << std::endl;
  //  out << std::string("--------------------------------------") << std::endl;

  // IMU
  //  out << std::string("--------------------------------------") << std::endl;
  //  out << std::string("Accel_x : ") << std::to_string(imu_.linear_acceleration.x) << std::endl;
  //  out << std::string("Accel_y : ") << std::to_string(imu_.linear_acceleration.y) << std::endl;
  //  out << std::string("Accel_z : ") << std::to_string(imu_.linear_acceleration.z) << std::endl;
  //  out << std::string("Gyro_x : ") << std::to_string(imu_.angular_velocity.x) << std::endl;
  //  out << std::string("Gyro_y : ") << std::to_string(imu_.angular_velocity.y) << std::endl;
  //  out << std::string("Gyro_z : ") << std::to_string(imu_.angular_velocity.z) << std::endl;
  //  out << std::string("Qaut_x : ") << std::to_string(imu_.orientation.x) << std::endl;
  //  out << std::string("Qaut_y : ") << std::to_string(imu_.orientation.y) << std::endl;
  //  out << std::string("Qaut_z : ") << std::to_string(imu_.orientation.z) << std::endl;
  //  out << std::string("Qaut_w : ") << std::to_string(imu_.orientation.w) << std::endl;
  //  out << std::string("--------------------------------------") << std::endl;

  // JointPub
  // out << std::string("joint_state_.position[0] : ") << std::to_string(joint_state_.position[0]) << std::endl;
  // out << std::string("joint_state_.position[1] : ") << std::to_string(joint_state_.position[1]) << std::endl;
  // out << std::string("joint_state_.velocity[0] : ") << std::to_string(joint_state_.velocity[0]) << std::endl;
  // out << std::string("joint_state_.velocity[1] : ") << std::to_string(joint_state_.velocity[1]) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;

  // MotorStatePub
  // out << std::string("-------------Write ID : ") << std::to_string(write_frame.id) << std::string("------------------") << std::endl;
  // out << std::string("driveLowByte : ") << std::to_string(write_frame.data[0]) << std::endl;
  // out << std::string("driveHighByte : ") << std::to_string(write_frame.data[1]) << std::endl;
  // out << std::string("steerLowByte : ") << std::to_string(write_frame.data[2]) << std::endl;
  // out << std::string("steerHighByte : ") << std::to_string(write_frame.data[3]) << std::endl;
  // out << std::string("=> value of steer : ") << std::to_string(applied_angle_byte) << std::endl;
  // out << std::string("=> value2 of steer : ") << std::to_string((int16_t)applied_angle_byte) << std::endl;
  // out << std::string("forward : ") << std::to_string((write_frame.data[4] >> 3) & 0x01) << std::endl;
  // out << std::string("backward : ") << std::to_string((write_frame.data[4] >> 4) & 0x01) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());

  RCLCPP_INFO(this->get_logger(), out.str().c_str());
}

// Main method of the node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AgvCanBus>();
  rclcpp::spin(node);
  // Free up any resources being used by the node
  rclcpp::shutdown();
  return 0;
}
