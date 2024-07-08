#include "agv_controller/tricycle_agv_drive.hpp"

using namespace std::chrono_literals;

TricycleAGVDrive::TricycleAGVDrive() : Node("tricycle_agv_drive_node")
{
  this->declare_parameter<double>("max_wheel_accel", 0.1);
  this->declare_parameter<double>("max_wheel_decel", 0.1);
  this->declare_parameter<double>("max_wheel_speed_tol", 1.0);
  // this->declare_parameter<double>("wheel_separation", 0.9);
  this->declare_parameter<double>("drive_wheel_radius", 0.31);
  // this->declare_parameter<double>("front_wheel_radius", 0.25);
  this->declare_parameter<bool>("publish_odom", true);
  this->declare_parameter<bool>("publish_wheel_tf", true);
  this->declare_parameter<bool>("publish_wheel_joint_state", true);

  this->get_parameter("max_wheel_accel", max_wheel_accel_);
  this->get_parameter("max_wheel_decel", max_wheel_decel_);
  this->get_parameter("max_wheel_speed_tol", max_wheel_speed_tol_);
  // this->get_parameter("wheel_separation", wheel_separation_);
  this->get_parameter("drive_wheel_radius", drive_wheel_radius_);
  // this->get_parameter("front_wheel_radius", front_wheel_radius_);
  this->get_parameter("publish_odom", publish_odom_);
  this->get_parameter("publish_wheel_tf", publish_wheel_tf_);
  this->get_parameter("publish_wheel_joint_state", publish_wheel_joint_state_);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TricycleAGVDrive::onCmdVel, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  last_update_time_ = this->now();

  joint_state_.name.resize(3);
  joint_state_.position.resize(3);
  joint_state_.velocity.resize(3);
  joint_state_.effort.resize(3);

  joint_state_.name[0] = "drive_wheel_joint";
  joint_state_.name[1] = "front_wheel_l_joint";
  joint_state_.name[2] = "front_wheel_r_joint";

  // odom_.header.frame_id = "odom";
  // odom_.child_frame_id = "base_footprint";

  timer_ = this->create_wall_timer(10ms, std::bind(&TricycleAGVDrive::onUpdate, this));
}

void TricycleAGVDrive::onUpdate()
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  auto current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();

  updateOdometry(current_time);
  motorController(cmd_.linear.x, dt);

  if (publish_odom_)
  {
    publishOdometryMsg(current_time);
  }
  if (publish_wheel_tf_)
  {
    publishWheelTransforms(current_time);
  }
  if (publish_wheel_joint_state_)
  {
    publishWheelJointStates(current_time);
  }

  last_update_time_ = current_time;
}

void TricycleAGVDrive::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  cmd_ = *msg;
}

void TricycleAGVDrive::updateOdometry(const rclcpp::Time &current_time)
{
  double dt = (current_time - last_update_time_).seconds();
  // double drive_wheel_speed = cmd_.linear.x / drive_wheel_radius_;

  double linear_velocity = cmd_.linear.x;

  RCLCPP_INFO(get_logger(), "%f", linear_velocity);
  // pose_.x += linear_velocity * cos(pose_.theta) * dt;
  pose_.y += linear_velocity * cos(pose_.theta) * dt;
}

void TricycleAGVDrive::publishOdometryMsg(const rclcpp::Time &current_time)
{
  odom_.header.stamp = current_time;
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_footprint";
  odom_.pose.covariance[0] = 0.00001;
  odom_.pose.covariance[7] = 0.00001;
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = 0.001;
  odom_.pose.pose.position.x = 0.0;
  odom_.pose.pose.position.y = pose_.y;
  odom_.pose.pose.position.z = 0.0;

  tf2::Quaternion qt;
  qt.setRPY(0, 0, pose_.theta);
  odom_.pose.pose.orientation = tf2::toMsg(qt);

  odom_.twist.twist.linear.x = cmd_.linear.x;
  odom_.twist.twist.angular.z = 0;

  odom_pub_->publish(odom_);
}

void TricycleAGVDrive::publishWheelTransforms(const rclcpp::Time &current_time)
{
  static int count = 0;
  for (std::size_t i = 0; i < 3; i++)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;

    if (i == 0)
    {
      tf.header.frame_id = "steering_part";
    }
    else
    {
      tf.header.frame_id = "base_link";
    }
    tf.child_frame_id = joint_state_.name[i];

    if (count < 3)
    {
      RCLCPP_INFO(get_logger(), "%s", tf.header.frame_id.c_str());
      RCLCPP_INFO(get_logger(), "%s", tf.child_frame_id.c_str());
      count++;
    }

    tf2::Quaternion qt;
    qt.setRPY(0, 0, 0);
    tf.transform.rotation = tf2::toMsg(qt);
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    tf_broadcaster_->sendTransform(tf);
  }
}

void TricycleAGVDrive::publishWheelJointStates(const rclcpp::Time &current_time)
{
  joint_state_.header.stamp = current_time;
  for (std::size_t i = 0; i < 3; i++)
  {
    joint_state_.position[i] = 0;
    joint_state_.velocity[i] = cmd_.linear.x / drive_wheel_radius_;
    joint_state_.effort[i] = 0;
  }
  joint_state_pub_->publish(joint_state_);
}

void TricycleAGVDrive::motorController(double target_speed, double dt)
{
  double speed_error = target_speed;
  double accel = speed_error / dt;

  if (accel > max_wheel_accel_)
  {
    accel = max_wheel_accel_;
  }
  else if (accel < -max_wheel_decel_)
  {
    accel = -max_wheel_decel_;
  }

  double new_speed = accel * dt;

  if (new_speed > max_wheel_speed_tol_)
  {
    new_speed = max_wheel_speed_tol_;
  }
  else if (new_speed < -max_wheel_speed_tol_)
  {
    new_speed = -max_wheel_speed_tol_;
  }

  cmd_.linear.x = new_speed * drive_wheel_radius_;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TricycleAGVDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
