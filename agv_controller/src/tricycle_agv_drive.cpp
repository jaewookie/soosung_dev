#include "agv_controller/tricycle_agv_drive.hpp"

using namespace std::chrono_literals;

TricycleAGVDrive::TricycleAGVDrive(const rclcpp::NodeOptions &node_options) : Node("tricycle_agv_drive_node", node_options), lin_vel_(0.0), ang_vel_(0.0), wheel_radius_(0.31), max_wheel_accel_(0.1), current_speed_(0.0), drive_rpm_(0.0), drive_wheel_speed_(0.0)
{

  RCLCPP_INFO(this->get_logger(), "Wheel Drive Controller initialized");

  // qos
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // sub_cmd
  cmdvel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", QOS_RKL10V, std::bind(&TricycleAGVDrive::cmd_vel_callback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", QOS_RKL10V);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  timer_ = this->create_wall_timer(10ms, std::bind(&TricycleAGVDrive::OnUpdate, this));

  // init
  last_update_time_ = this->now();
}

TricycleAGVDrive::~TricycleAGVDrive() {}

void TricycleAGVDrive::OnUpdate()
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  auto current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();

  MotorController(lin_vel_, dt);
  UpdateOdometry(dt);
  publishOdometryMsg(current_time);

  last_update_time_ = current_time;
}

void TricycleAGVDrive::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  cmd_ = *msg;

  lin_vel_ = cmd_.linear.x; // [m/s]
  ang_vel_ = cmd_.angular.z;

  RCLCPP_INFO(get_logger(), "target_speed = [%f]", lin_vel_);
  RCLCPP_INFO(get_logger(), "target_rpm = [%f]", drive_rpm_);
}

void TricycleAGVDrive::MotorController(double target_speed, double dt)
{
  // 나중에 current_speed에는 encoder에서 가져온 값을 이용 / 모터 사양에서 가속 설정 있는지 확인

  if (current_speed_ < target_speed)
  {
    current_speed_ += max_wheel_accel_ * dt;
  }
  else if (current_speed_ > target_speed)
  {
    current_speed_ -= max_wheel_accel_ * dt;
  }

  drive_wheel_speed_ = current_speed_ / wheel_radius_; //[rad/s]

  drive_rpm_ = 60 * lin_vel_ / (2 * M_PI * wheel_radius_); // [rpm] => 모터 드라이브로 전송할 데이터 중 하나 ( 모터 드라이브에서 가속도 설정이 가능하면 해당 목표 rpm 전송 )

  // drive_rpm_ = 60 * current_speed_ / (2 * M_PI * wheel_radius_); // [rpm] => 모터 드라이브로 전송할 데이터 중 하나 ( 모터 드라이브에서 가속도 설정이 없는 경우는 현재 속도에 따른 rpm을 연속적으로 전송 )

  RCLCPP_INFO(get_logger(), "current_lin_speed = [%f]", current_speed_);
  RCLCPP_INFO(get_logger(), "current_wheel_speed = [%f]", drive_wheel_speed_);
  RCLCPP_INFO(get_logger(), "current_rpm = [%f]", 60 * current_speed_ / (2 * M_PI * wheel_radius_));
}

void TricycleAGVDrive::UpdateOdometry(double dt)
{
  double v = current_speed_;

  double dx = 0.0;
  double dy = v * dt;

  double dtheta = 0.0;

  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

  double w = dtheta / dt;

  tf2::Vector3 vt;
  vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);
  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  tf2::Quaternion qt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  odom_.pose.pose.orientation = tf2::toMsg(qt);

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = dx / dt;
  odom_.twist.twist.linear.y = dy / dt;
}

void TricycleAGVDrive::publishOdometryMsg(const rclcpp::Time &current_time)
{

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = current_time;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_footprint";
  msg.transform.translation.x = odom_.pose.pose.position.x;
  msg.transform.translation.y = odom_.pose.pose.position.y;
  msg.transform.translation.z = odom_.pose.pose.position.z;
  msg.transform.rotation = odom_.pose.pose.orientation;

  tf_broadcaster_->sendTransform(msg);

  // set header stamp
  odom_.header.stamp = current_time;

  odom_pub_->publish(odom_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TricycleAGVDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
