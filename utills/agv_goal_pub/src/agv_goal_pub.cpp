#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

//'/navigate_to_pose/_action/status'   action_msgs/msg/GoalStatusArray.status_list[count].status==4

class AgvGoalPub : public rclcpp::Node
{
public:
  AgvGoalPub() : Node("agv_goal_pub"), goal_count(0)
  {
    // qos
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_ori", QOS_RKL10V);
    get_input();
    goal_status_subscriber_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", QOS_RKL10V, std::bind(&AgvGoalPub::goal_status_callback, this, std::placeholders::_1));
  }

private:
  void get_input()
  {
    // 사용자로부터 좌표 입력 받기
    std::cout << "Enter the x coordinate for the goal: ";
    std::cin >> x_;
    std::cout << "Enter the y coordinate for the goal: ";
    std::cin >> y_;
    publish_goal();
  }
  void publish_goal()
  {
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.stamp = this->now();
    message.header.frame_id = "map";

    // 입력 받은 목표 위치 설정
    message.pose.position.x = x_;
    message.pose.position.y = y_;
    message.pose.position.z = 0.0;

    // 회전 없이 목표지점 설정
    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published goal: x=%f, y=%f", message.pose.position.x, message.pose.position.y);
  }
  // action_msgs/msg/GoalStatusArray.status_list[count].status==4
  void goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    action_msgs::msg::GoalStatusArray goal_;
    goal_ = *msg;
    RCLCPP_INFO(this->get_logger(), "count : [%d]", goal_count);
    RCLCPP_INFO(this->get_logger(), "%d", goal_.status_list[goal_count].status);
    if (goal_.status_list[goal_count].status == 4)
    {
      goal_count++;
      get_input();
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr goal_status_subscriber_;
  float x_, y_;
  int goal_count;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvGoalPub>());
  rclcpp::shutdown();
  return 0;
}
