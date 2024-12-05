#ifndef __AgvCanBus_HPP__
#define __AgvCanBus_HPP__

#include <linux/can/raw.h>

#include <cmath>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <agv_msgs/msg/fork_control.hpp>

// CAN ID define
#define DRIVER_TO_PC_CAN1 414
#define DRIVER_TO_PC_CAN2 670
#define DRIVER_TO_PC_CAN3 926
#define DRIVER_TO_PC_CAN4 1182
#define DRIVER_TO_PC_CAN5 432

#define PC_TO_DRIVER_CAN1 542
#define PC_TO_DRIVER_CAN2 798

class AgvCanBus : public rclcpp::Node
{
public:
  AgvCanBus(); // boost::asio::io_service& ios);
  ~AgvCanBus();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;


  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_subscriber_;
  rclcpp::Subscription<agv_msgs::msg::ForkControl>::SharedPtr mast_vel_sub_;


  can_msgs::msg::Frame current_frame;
  can_msgs::msg::Frame write_frame;

  void CanReceiver(const can_msgs::msg::Frame::SharedPtr msg);
  void CanRead1(const can_msgs::msg::Frame msg);
  void CanRead2(const can_msgs::msg::Frame msg);
  void CanRead3(const can_msgs::msg::Frame msg);
  void CanRead4(const can_msgs::msg::Frame msg);
  void CanRead5(const can_msgs::msg::Frame msg);
  // void CanReadBitTest(const can_msgs::msg::Frame msg);
  int16_t combineBytes(const uint8_t lowByte, const uint8_t highByte);

  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void MastVelCallback(const agv_msgs::msg::ForkControl::ConstSharedPtr msg);
  void CanSend(uint lowByte, uint highByte, uint canId);
  void AutoInterEn();
  void CmdVelTrans();
  void MastTrans(double target_speed);

  int rad_to_dec(float rad_ang_);

  struct can_frame frame;
  struct can_frame rec_frame;

  std::string topicname_receive;
  std::string topicname_transmit;

  // can1 read data
  uint8_t drive_motor_current_rpm_low_byte;
  uint8_t drive_motor_current_rpm_high_byte;
  uint8_t steering_current_angle_low_byte;
  uint8_t steering_current_angle_high_byte;
  uint8_t drive_motor_temperature_low_byte;
  uint8_t drive_motor_temperature_high_byte;
  uint8_t steering_motor_temperature_low_byte;
  uint8_t steering_motor_temperature_high_byte;

  // can2 read data
  uint8_t drive_motor_current_low_byte;
  uint8_t drive_motor_current_high_byte;
  uint8_t steering_current_low_byte;
  uint8_t steering_current_high_byte;
  uint8_t distance_fine_byte1;
  uint8_t distance_fine_byte2;
  uint8_t distance_fine_byte3;
  uint8_t distance_fine_byte4;

  // can3 read data
  uint8_t traction_throttle_analog_byte;
  uint8_t steer_analog_byte;
  uint8_t drive_pd_val_byte;
  bool drive_interlock;
  bool drive_emg_reverse;
  bool forward;
  bool reverse;
  bool manual_auto_set;
  bool em_break;
  bool back_buzzer;
  bool horn;
  bool drive_main_coil;
  uint8_t drive_controller_fault_code_byte;
  uint8_t steering_controller_fault_code_byte;
  uint8_t hydraulic_controller_fault_code_byte;

  // can4 read data
  uint8_t hydraulic_motor_current_rpm_low_byte;
  uint8_t hydraulic_motor_current_rpm_high_byte;
  uint8_t hydraulic_motor_temperature_low_byte;
  uint8_t hydraulic_motor_temperature_high_byte;
  uint8_t hydraulic_motor_current_low_byte;
  uint8_t hydraulic_motor_current_high_byte;
  bool forktip_sensor_right;
  bool forktip_sensor_left;
  bool detect_sensor_front_and_back;
  bool detect_sensor_top_and_push;
  bool bumper_switch;

  // can5 read data
  uint8_t lift_encoder_low_byte;
  uint8_t lift_encoder_high_byte;

  // Read Byte data Combine
  uint16_t can_rpm_byte;
  uint16_t can_steer_ang_byte;
  uint16_t can_drive_motor_temperature_byte;
  uint16_t can_steering_motor_temperature_byte;
  uint16_t can_drive_motor_current_byte;
  uint16_t can_steering_current_byte;
  uint16_t can_distance_fine_byte;
  uint16_t can_hydraulic_motor_current_rpm_byte;
  uint16_t can_hydraulic_motor_temperature_byte;
  uint16_t can_hydraulic_motor_current_byte;
  uint16_t can_lift_encoder_byte;

  // byte transform
  uint can_rpm;
  int16_t can_steer_ang_test;
  int can_steer_ang;
  int can_drive_motor_temperature;
  int can_steering_motor_temperature;
  int can_drive_motor_current;
  int can_steering_current;
  long can_distance_fine;
  u_char can_traction_throttle_analog;
  u_char can_steer_analog;
  u_char can_drive_pd_val;
  u_char can_drive_controller_fault_code;
  u_char can_steering_controller_fault_code;
  u_char can_hydraulic_controller_fault_code;
  uint can_hydraulic_motor_current_rpm;
  int can_hydraulic_motor_temperature;
  int can_hydraulic_motor_current;
  int can_lift_encoder;

  // vel 모터 제어를 위한 변수
  float lin_vel_;
  float ang_vel_;
  uint16_t drive_rpm_;
  int agv_direction_;
  double max_steering_speed_;
  double max_steering_angle_tol_;
  double applied_steering_speed;
  // false : 후진 true : 전진

  // mast 제어
  double mast_vel_;
  int hydraulic_direction_;
  u_char hyd_test;

  // can write

  // interlock / auto는 항상 1
  // interlock이 0이되는건 em눌렸을때
  // 전 후진 제어시 forward backward 필수
  // 0x31E byte2 유압 working을 넣어야 작동
  // => 상승때 byte3 bit0 1 bit1 0
  // => 하강때 byte3 bit0 0 bit1 1
  // break/push 미사용
  // 가속도 안씀

  // can read

  // 포크팁(좌/우) 각각
  // 파레트 디텍터 / 범퍼스위치
  // 리프트 엔코더 can_id 추후 제공 예정

  // 유압 모터 => 유량 =>
};
#endif
