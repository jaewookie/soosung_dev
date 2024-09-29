#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>

//'/navigate_to_pose/_action/status'   action_msgs/msg/GoalStatusArray.status_list[count].status==4

class AgvGoalPlanner : public rclcpp::Node
{
public:
  enum
  {
    HORIZONTAL,
    VERTICAL
  };
  AgvGoalPlanner() : Node("agv_goal_pub"), goal_count(0), agv_status(0), agv_direction(VERTICAL)
  {
    // qos
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", QOS_RKL10V);
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", QOS_RKL10V);

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", QOS_RKL10V, std::bind(&AgvGoalPlanner::agv_yaw_data, this, std::placeholders::_1));

    goal_pose_origin_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose_ori", QOS_RKL10V, std::bind(&AgvGoalPlanner::goal_pos_ori_callback, this, std::placeholders::_1));

    goal_status_subscriber_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", QOS_RKL10V, std::bind(&AgvGoalPlanner::goal_status_callback, this, std::placeholders::_1));
  }

private:
  void publish_goal(float x__, float y__, float z__)
  {
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.stamp = this->now();
    message.header.frame_id = "map";

    // 입력 받은 목표 위치 설정
    message.pose.position.x = x__;
    message.pose.position.y = y__;
    message.pose.position.z = 0.0;

    // 회전 없이 목표지점 설정
    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = z__;
    message.pose.orientation.w = 1.0;

    goal_pose_pub_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published goal: x=%f, y=%f", message.pose.position.x, message.pose.position.y);
  }
  void goal_pos_ori_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    geometry_msgs::msg::PoseStamped ori_;
    ori_ = *pose_msg;
    ori_x_ = ori_.pose.position.x;
    ori_y_ = ori_.pose.position.y;
    x_ = ori_x_ + 1;
    y_ = 0;
    publish_goal(x_, y_, 0.0);
  }
  // action_msgs/msg/GoalStatusArray.status_list[count].status==4
  void goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    action_msgs::msg::GoalStatusArray goal_;
    goal_ = *msg;
    // RCLCPP_INFO(this->get_logger(), "count : [%d]", goal_count);
    // RCLCPP_INFO(this->get_logger(), "%d", goal_.status_list[goal_count].status);
    agv_status = goal_.status_list[goal_count].status;
    if (agv_status == 4)
    {
      goal_count++;
    }
  }

  void agv_turning()
  {
    geometry_msgs::msg::Twist cmd_;
    if (yaw_ > 0.88 * (-M_PI / 4)) // 0.88 실험 값 사용함
    {
      cmd_.linear.x = 0.2;
      cmd_.angular.z = -M_PI / 2;
      // RCLCPP_INFO(this->get_logger(), "yaw=%f", yaw_);
    }
    else
    {
      cmd_.linear.x = 0;
      cmd_.angular.z = 0;
      sleep(1);
      publish_goal(ori_x_, ori_y_, -M_PI / 4);
    }
    cmd_pub_->publish(cmd_);
    // RCLCPP_INFO(this->get_logger(), "Published vel: x=%f, z=%f", cmd_.linear.x, cmd_.angular.z);
  }

  void agv_yaw_data(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu imu_;
    imu_ = *msg;
    yaw_ = imu_.orientation.z;
    RCLCPP_INFO(this->get_logger(), "%f", yaw_);
    if(fabs(cos(yaw_)) > (1/sqrt(2))){
      agv_direction = VERTICAL;
      // RCLCPP_INFO(this->get_logger(), "VERTICAL!!!!!!!!!!!!");
    }else{
      agv_direction = HORIZONTAL;
      // RCLCPP_INFO(this->get_logger(), "HORIZONTAL!!!!!!!!!!!!");
    }

    if (agv_status == 4)
    {
      agv_turning();
    }
  }

  // pub
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  // sub
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_origin_subscriber_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  // rclcpp::TimerBase::SharedPtr timer_;

  float ori_x_, ori_y_;
  float x_, y_;
  float yaw_;
  int goal_count;
  int agv_status;
  int agv_direction;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvGoalPlanner>());
  rclcpp::shutdown();
  return 0;
}
