// Original Author: Philipp Wuestenberg <philipp.wuestenberg@tu-berlin.de>
// Maintainer: Jonathan Blixti <blixt013@umn.edu>
// Last Updated: November 2023

// Import header file
#include "agv_test_pack/agv_can_bus.hpp"
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>

AgvCanBus::AgvCanBus() : Node("AgvCanBus"),
                         lin_vel_(0.0), // 초기 선형 속도는 0으로 설정
                         ang_vel_(0.0),
                         drive_rpm_(0),
                         agv_direction_(1),
                         max_steering_speed_(1),
                         max_steering_angle_tol_(0.05),
                         applied_steering_speed(0),
                         mast_vel_(0.0),
                         hydraulic_direction_(0)
{
  this->declare_parameter("CAN_INTERFACE", "can0");
  std::string can_socket = this->get_parameter("CAN_INTERFACE").as_string();

  int8_t qos_depth = 0;

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
  topicname_receive = "CAN/can0/receive";
  topicname_transmit = "CAN/can0/transmit";

  // can data 수신
  subscription_ = this->create_subscription<can_msgs::msg::Frame>(topicname_receive, QOS_RKL10V, std::bind(&AgvCanBus::CanReceiver, this, std::placeholders::_1));
  // can data 송신용
  publisher_ = this->create_publisher<can_msgs::msg::Frame>(topicname_transmit, 10);
  cmdvel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", QOS_RKL10V, std::bind(&AgvCanBus::CmdVelCallback, this, std::placeholders::_1));

  mast_vel_sub_ = this->create_subscription<agv_msgs::msg::ForkControl>(
      "/mast_vel", QOS_RKL10V, std::bind(&AgvCanBus::MastVelCallback, this, std::placeholders::_1));

  AutoInterEn();
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
  can_rpm = static_cast<uint>(can_rpm_byte);

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

  std::stringstream out;
  out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  out << std::string("drive_motor_current_rpm_low_byte : ") << std::to_string(drive_motor_current_rpm_low_byte) << std::endl;
  out << std::string("drive_motor_current_rpm_high_byte : ") << std::to_string(drive_motor_current_rpm_high_byte) << std::endl;
  out << std::string(" => drive_motor_current_rpm_byte : ") << std::to_string(can_rpm_byte) << std::endl;
  out << std::string(" ==> drive_motor_current_rpm_value : ") << std::to_string(can_rpm) << std::endl;

  out << std::string("steering_current_angle_low_byte : ") << std::to_string(steering_current_angle_low_byte) << std::endl;
  out << std::string("steering_current_angle_high_byte : ") << std::to_string(steering_current_angle_high_byte) << std::endl;
  out << std::string(" => steering_current_angle_byte : ") << std::to_string(can_steer_ang_byte) << std::endl;
  out << std::string(" ==> steering_current_angle_value : ") << std::to_string(can_steer_ang) << std::endl;

  out << std::string("drive_motor_temperature_low_byte : ") << std::to_string(drive_motor_temperature_low_byte) << std::endl;
  out << std::string("drive_motor_temperature_high_byte : ") << std::to_string(drive_motor_temperature_high_byte) << std::endl;
  out << std::string(" => drive_motor_temperature_byte : ") << std::to_string(can_drive_motor_temperature_byte) << std::endl;
  out << std::string(" ==> drive_motor_temperature_value : ") << std::to_string(can_drive_motor_temperature) << std::endl;

  out << std::string("steering_motor_temperature_low_byte : ") << std::to_string(steering_motor_temperature_low_byte) << std::endl;
  out << std::string("steering_motor_temperature_high_byte : ") << std::to_string(steering_motor_temperature_high_byte) << std::endl;
  out << std::string(" => steering_motor_temperature_byte : ") << std::to_string(can_steering_motor_temperature_byte) << std::endl;
  out << std::string(" ==> steering_motor_temperature_value : ") << std::to_string(can_steering_motor_temperature) << std::endl;
  out << std::string("--------------------------------------") << std::endl;
  RCLCPP_INFO(this->get_logger(), out.str().c_str());
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

  // std::stringstream out;
  // out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  // out << std::string("drive_motor_current_low_byte : ") << std::to_string(drive_motor_current_low_byte) << std::endl;
  // out << std::string("drive_motor_current_high_byte : ") << std::to_string(drive_motor_current_high_byte) << std::endl;
  // out << std::string(" => drive_motor_current : ") << std::to_string(can_drive_motor_current_byte) << std::endl;
  // out << std::string(" ==> drive_motor_current_value : ") << std::to_string(can_drive_motor_current) << std::endl;

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
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());
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

  // std::stringstream out;
  // out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  // out << std::string("traction_throttle_analog : ") << std::to_string(traction_throttle_analog_byte) << std::endl;
  // out << std::string(" ==> traction_throttle_analog_value : ") << std::to_string(can_traction_throttle_analog) << std::endl;

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
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());
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

  // std::stringstream out;
  // out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  // out << std::string("hydraulic_motor_current_rpm_low_byte : ") << std::to_string(hydraulic_motor_current_rpm_low_byte) << std::endl;
  // out << std::string("hydraulic_motor_current_rpm_high_byte : ") << std::to_string(hydraulic_motor_current_rpm_high_byte) << std::endl;
  // out << std::string(" => hydraulic_motor_current_rpm_byte : ") << std::to_string(can_hydraulic_motor_current_rpm_byte) << std::endl;
  // out << std::string(" ==> hydraulic_motor_current_rpm_value : ") << std::to_string(can_hydraulic_motor_current_rpm) << std::endl;

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
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());
}

void AgvCanBus::CanRead5(const can_msgs::msg::Frame msg)
{
  lift_encoder_low_byte = msg.data[0];
  lift_encoder_high_byte = msg.data[1];
  can_lift_encoder_byte = combineBytes(lift_encoder_low_byte, lift_encoder_high_byte);
  can_lift_encoder = static_cast<uint>(can_lift_encoder_byte);

  // std::stringstream out;
  // out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
  // out << std::string("lift_encoder_low_byte : ") << std::to_string(lift_encoder_low_byte) << std::endl;
  // out << std::string("lift_encoder_high_byte : ") << std::to_string(lift_encoder_high_byte) << std::endl;
  // out << std::string(" => lift_encoder_byte : ") << std::to_string(can_lift_encoder_byte) << std::endl;
  // out << std::string(" ==> lift_encoder_value : ") << std::to_string(can_lift_encoder) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());
}

// Byte Combine Func
int16_t AgvCanBus::combineBytes(const uint8_t lowByte, const uint8_t highByte)
{

  int16_t combined = ((highByte) << 8) | lowByte;

  return combined;
}

// test debug 용
// void AgvCanBus::CanReadBitTest(const can_msgs::msg::Frame msg)
// {
//   int test_byte_val = msg.data[0];
//   bool test_bit_val1 = msg.data[0] & 0x01;
//   bool test_bit_val2 = (msg.data[0] >> 1) & 0x01;
//   bool test_bit_val3 = (msg.data[0] >> 2) & 0x01;
//   bool test_bit_val4 = (msg.data[0] >> 3) & 0x01;
//   bool test_bit_val5 = (msg.data[0] >> 4) & 0x01;
//   bool test_bit_val6 = (msg.data[0] >> 5) & 0x01;
//   bool test_bit_val7 = (msg.data[0] >> 6) & 0x01;
//   bool test_bit_val8 = (msg.data[0] >> 7) & 0x01;

//   std::stringstream out;
//   out << std::string("-------------Read ID : ") << std::to_string(msg.id) << std::string("------------------") << std::endl;
//   out << std::string("test_byte_val : ") << std::to_string(test_byte_val) << std::endl;
//   out << std::string("test_bit_val1;: ") << std::to_string(test_bit_val1) << std::endl;
//   out << std::string("test_bit_val2;: ") << std::to_string(test_bit_val2) << std::endl;
//   out << std::string("test_bit_val3;: ") << std::to_string(test_bit_val3) << std::endl;
//   out << std::string("test_bit_val4;: ") << std::to_string(test_bit_val4) << std::endl;
//   out << std::string("test_bit_val5;: ") << std::to_string(test_bit_val5) << std::endl;
//   out << std::string("test_bit_val6;: ") << std::to_string(test_bit_val6) << std::endl;
//   out << std::string("test_bit_val7;: ") << std::to_string(test_bit_val7) << std::endl;
//   out << std::string("test_bit_val8;: ") << std::to_string(test_bit_val8) << std::endl;
//   out << std::string("--------------------------------------") << std::endl;
//   RCLCPP_INFO(this->get_logger(), out.str().c_str());
// }

//
// Can Data Write
//
// cmd_vel 구독
void AgvCanBus::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  lin_vel_ = msg->linear.x;  // cmd_vel로부터 선형 속도를 갱신
  ang_vel_ = msg->angular.z; // cmd_vel로부터 각속도를 갱신, 현재 사용하지 않음

  CmdVelTrans();
}
// cnd_vel_lin -> rpm 변환
void AgvCanBus::CmdVelTrans()
{
  std::stringstream out;

  // rpm 전송 테스트 파일
  std::vector<uint8_t> rpm_result(2);
  std::vector<uint8_t> ang_result(2);

  // lin_vel to RPM

  if (lin_vel_ > 0)
  {
    agv_direction_ = 1;
  }
  else if (lin_vel_ < 0)
  {
    agv_direction_ = 0;
    lin_vel_ = -lin_vel_;
  }
  else if (lin_vel_ == 0.0)
  {
    agv_direction_ = 2;
    lin_vel_ = -lin_vel_;
  }


  // (lin_vel_ * 60)[m/Min] / (0.32 * M_PI)       [Rev/Min]
  drive_rpm_ = 16.755*(lin_vel_ * 60) / (0.32 * M_PI);
  drive_rpm_ = (uint)drive_rpm_;

  // ang_vel to steering
  int applied_angle = rad_to_dec(ang_vel_);
  uint16_t applied_angle_byte;
  int16_t applied_angle_result;

  int current_angle = can_steer_ang_test;
  int diff_angle = current_angle - rad_to_dec(ang_vel_);

  auto start = std::chrono::high_resolution_clock::now();

  // out << std::string("max_steering_angle_tol_ : ") << std::to_string(rad_to_dec(max_steering_angle_tol_)) << std::endl;
  if (rad_to_dec(max_steering_angle_tol_) > 0)
  {
    // this means we will steer using steering speed
    if (fabs(diff_angle) < rad_to_dec(max_steering_angle_tol_))
    {
      // we're withing angle tolerance
      applied_steering_speed = 0;
    }
    else
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
      auto end = std::chrono::high_resolution_clock::now();
      auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
      double seconds = std::chrono::duration<double>(dt).count();

      applied_angle = current_angle + applied_steering_speed * seconds;
    }

    // out << std::string("applied_angle : ") << std::to_string(applied_angle) << std::endl;
    applied_angle_byte = (uint16_t)applied_angle;

    // if (applied_angle < 0)
    // {
    //   applied_angle_byte = (~applied_angle_byte);
    //   applied_angle_result = -static_cast<int>(applied_angle_byte);
    // }
    // else
    // {
    //   applied_angle_result = static_cast<int>(applied_angle_byte);
    // }

    // applied_angle_result = (uint)applied_angle_result;
    // applied_angle_byte = applied_angle;

    // out << std::string("applied_angle : ") << std::to_string(applied_angle) << std::endl;
    // out << std::string("applied_angle_result : ") << std::to_string(applied_angle_result) << std::endl;
    out << std::string("applied_angle_byte : ") << std::to_string(applied_angle_byte) << std::endl;
  }

  ang_result[0] = (applied_angle_byte & 0xff);
  ang_result[1] = ((applied_angle_byte >> 8) & 0xff);

  // out << std::string("ang_result[0] : ") << std::to_string(ang_result[0]) << std::endl;
  // out << std::string("ang_result[1] : ") << std::to_string(ang_result[1]) << std::endl;

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
    write_frame.data[4] = 5;
  }

  // out << std::string("-------------Write ID : ") << std::to_string(write_frame.id) << std::string("------------------") << std::endl;
  // out << std::string("driveLowByte : ") << std::to_string(write_frame.data[0]) << std::endl;
  // out << std::string("driveHighByte : ") << std::to_string(write_frame.data[1]) << std::endl;
  // out << std::string("steerLowByte : ") << std::to_string(write_frame.data[2]) << std::endl;
  // out << std::string("steerHighByte : ") << std::to_string(write_frame.data[3]) << std::endl;
  // out << std::string("forward : ") << std::to_string((write_frame.data[4] >> 3) & 0x01) << std::endl;
  // out << std::string("backward : ") << std::to_string((write_frame.data[4] >> 4) & 0x01) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());
  publisher_->publish(write_frame);
}

void AgvCanBus::MastVelCallback(const agv_msgs::msg::ForkControl::ConstSharedPtr msg)
{
  mast_vel_ = 1000 * msg->fork_vel;
  MastTrans(mast_vel_);
}

void AgvCanBus::MastTrans(double target_speed)
{
  std::stringstream out;

  std::vector<uint8_t> mast_result(2);

  u_char hydraluic_propor_vel;

  hydraluic_propor_vel = '115';
  // 위치 변환 계산
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

  if (hydraulic_direction_ == 1) // 상승
  {
    write_frame.data[2] = 7;
    // if (((write_frame.data[2] >> 1) & 0x01) != 0x01)
    // {
    //   write_frame.data[2] = write_frame.data[2] ^ 2;
    // }
    write_frame.data[3] = 1;
  }
  else if (hydraulic_direction_ == 0) // 하강
  {
    write_frame.data[2] = 5;
    // if (((write_frame.data[2] >> 1) & 0x01) == 0x01)
    // {
    //   write_frame.data[2] = write_frame.data[2] ^ 2;
    // }
    write_frame.data[3] = 2;
  }
  else if (hydraulic_direction_ == 2) // 정지
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
  // publisher_->publish(write_frame);

  // RCLCPP_INFO(ros_node_->get_logger(), "[%f]", joints_[0]->Position(0));
}

int AgvCanBus::rad_to_dec(float rad_ang_)
{
  int angle = rad_ang_ * 18000 / M_1_PI;

  return angle;
}

// 변환한 data를 Can 송신용 ros wrapper로 보내기 위한 topic 구성 및 발행
void AgvCanBus::CanSend(uint lowByte, uint highByte, uint canId)
{
  write_frame.id = canId;

  write_frame.dlc = 8;

  write_frame.data[0] = lowByte;
  write_frame.data[1] = highByte;

  if (agv_direction_ == 1)
  {
    if (((write_frame.data[4] >> 3) & 0x01) != 0x01)
    {
      write_frame.data[4] = write_frame.data[4] ^ 24;
    }
  }
  else if (agv_direction_ == 0)
  {
    if (((write_frame.data[4] >> 4) & 0x01) != 0x01)
    {
      write_frame.data[4] = write_frame.data[4] ^ 24;
    }
  }

  // std::stringstream out;
  // out << std::string("-------------Write ID : ") << std::to_string(write_frame.id) << std::string("------------------") << std::endl;
  // out << std::string("driveLowByte : ") << std::to_string(write_frame.data[0]) << std::endl;
  // out << std::string("driveHighByte : ") << std::to_string(write_frame.data[1]) << std::endl;
  // out << std::string("forward : ") << std::to_string((write_frame.data[4] >> 3) & 0x01) << std::endl;
  // out << std::string("backward : ") << std::to_string((write_frame.data[4] >> 4) & 0x01) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());
  publisher_->publish(write_frame);
}

void AgvCanBus::AutoInterEn()
{
  write_frame.id = PC_TO_DRIVER_CAN1;

  write_frame.dlc = 8;

  write_frame.data[4] = 5;

  // std::stringstream out;
  // out << std::string("-------------Write ID : ") << std::to_string(write_frame.id) << std::string("------------------") << std::endl;
  // out << std::string("interlock : ") << std::to_string(write_frame.data[4] & 0x01) << std::endl;
  // out << std::string("auto : ") << std::to_string((write_frame.data[4] >> 2) & 0x01) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());
  publisher_->publish(write_frame);

  write_frame.id = PC_TO_DRIVER_CAN2;

  write_frame.dlc = 8;

  write_frame.data[2] = 5;

  // out << std::string("-------------Write ID : ") << std::to_string(write_frame.id) << std::string("------------------") << std::endl;
  // out << std::string("interlock : ") << std::to_string(write_frame.data[4] & 0x01) << std::endl;
  // out << std::string("auto : ") << std::to_string((write_frame.data[4] >> 2) & 0x01) << std::endl;
  // out << std::string("--------------------------------------") << std::endl;
  // RCLCPP_INFO(this->get_logger(), out.str().c_str());
  publisher_->publish(write_frame);
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
